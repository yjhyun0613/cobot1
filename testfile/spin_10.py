import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String  # 💡 String 메시지 타입 추가
from dsr_msgs2.srv import DrlStart
import DR_init
import time

class ScanRunnerNode(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)
        self.state_pub = self.create_publisher(Bool, f'/{namespace}/checking_state', 10)
        
        # --- 💡 추가된 부분: progress_status 구독 ---
        self.trigger_scan = False
        self.is_scanning = False # 스캔 중 중복 실행 방지용 플래그
        
        self.progress_sub = self.create_subscription(
            String,
            f'/{namespace}/progress_status',
            self.progress_callback,
            10
        )
        # --------------------------------------------
        
        service_name = f'/{namespace}/drl/drl_start'
        self.drl_client = self.create_client(DrlStart, service_name)
        
        while not self.drl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} 서비스 대기 중...')
            
        self.get_logger().info('✅ 토픽 퍼블리셔 및 DRL 서비스 클라이언트 준비 완료')

    def progress_callback(self, msg):
        # 💡 "02" 신호가 들어오고, 현재 스캔 중이 아닐 때만 트리거 작동
        if msg.data == "02":
            if not self.is_scanning and not self.trigger_scan:
                self.get_logger().info('수신된 신호 [02]: 스캔 시퀀스를 시작합니다.')
                self.trigger_scan = True
            else:
                self.get_logger().info('수신된 신호 [02]: 이미 스캔이 진행 중이거나 예약되어 무시됩니다.')

    def publish_state(self, state: bool):
        msg = Bool()
        msg.data = state
        self.state_pub.publish(msg)
        status_text = "🟢 ON (데이터 기록 시작)" if state else "🔴 OFF (데이터 기록 종료)"
        self.get_logger().info(f'신호 전송: {status_text}')

    def send_drl_code(self, drl_script):
        self.get_logger().info('DRL 코드를 전송합니다...')
        req = DrlStart.Request()
        req.robot_system = 0        
        req.code = drl_script       
        self.drl_client.call_async(req)


def main(args=None):
    ROBOT_ID = "dsr01" 
    ROBOT_MODEL = "m0609"
    ROBOT_TOOL = "Tool Weight"
    ROBOT_TCP = "GripperDA_v1"
    
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    rclpy.init(args=args)
    node = ScanRunnerNode('scan_runner_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
     
    try:
        from DSR_ROBOT2 import (
            set_robot_mode, ROBOT_MODE_AUTONOMOUS, ROBOT_MODE_MANUAL,
            set_tool, set_tcp, movej, movel, get_robot_state
        )

        print("\n로봇 초기화 중...")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(1.0) 

        print("\n🔄 대기 모드: /dsr01/progress_status 토픽에서 '02' 신호를 기다립니다...")

        # 💡 [핵심] 1회성 실행이 아닌 무한 루프 형태로 변경
        while rclpy.ok():
            # 토픽 콜백 처리를 위해 spin_once 호출
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # trigger_scan 플래그가 켜지면 궤적 실행
            if node.trigger_scan:
                node.is_scanning = True
                node.trigger_scan = False
                
                print("\n🚀 초기 위치(반구 위쪽)로 이동 중...")
                p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
                
                # 원주 구속 시 특이점 발작을 막기 위한 ry=179도 설정
                rx, ry, rz = 0.0, 179.0, -90.0
                p_top = [429.48, 74.81, 200.00, rx, ry, rz]

                movej(p_ready, vel=100.0, acc=100.0)
                movel(p_top, vel=50.0, acc=100.0) 
                time.sleep(0.5)

                movel(p_top, vel=50.0, acc=100.0)
                node.publish_state(True)
                time.sleep(0.2) 
                
                # ==========================================================
                # [Step 3] 원주 구속(ori=2) + 지그재그 회전 적용 DRL
                # ==========================================================
                scan_drl_code = """
R = 48.0            
TOTAL_LAYERS = 5    
CENTER_X = 429.48   
CENTER_Y = 74.81    
CENTER_Z = 50.0    

set_singularity_handling(1)
stiff = [500, 500, 500, 100, 100, 100]
task_compliance_ctrl(stiff)

# ry=179.0 (특이점 회피)
rx, ry = 0.0, 179.0

for i in range(1, TOTAL_LAYERS + 1):
    angle_rad = d2r(90.0 * (1.0 - float(i)/TOTAL_LAYERS))
    curr_z = CENTER_Z + R * sin(angle_rad)
    curr_r = R * cos(angle_rad)
    
    # ori=2로 인해 꼬인 조인트를 지그재그로 완벽하게 풀며 이동
    if (i % 2) == 1:
        # [홀수 층] 시계 방향 스캔: 90도에서 시작하여 -270도로 꼬임
        rz = 270.0
        p_via_y = CENTER_Y - curr_r # 남쪽 경유
    else:
        # [짝수 층] 반시계 방향 스캔: 꼬인 상태(-270도) 그대로 시작해서 90도로 다시 풀림
        rz = -90.0
        p_via_y = CENTER_Y + curr_r # 북쪽 경유

    p_start = posx(CENTER_X + curr_r, CENTER_Y, curr_z, rx, ry, rz)
    p_tgt   = posx(CENTER_X - curr_r, CENTER_Y, curr_z, rx, ry, rz)
    p_via   = posx(CENTER_X, p_via_y, curr_z, rx, ry, rz)

    # 부드러운 블렌딩(r=10.0) 적용
    movel(p_start, v=40, a=80, r=10.0)
    
    # ori=2 적용 완수!
    movec(p_via, p_tgt, v=50, a=100, angle=360.0, ori=2)

release_compliance_ctrl()
"""
                node.send_drl_code(scan_drl_code)
                
                print("\n🛸 ori=2 원주 구속 스캐닝 진행 중...")
                time.sleep(1.0) 
                
                # 로봇이 동작을 완료할 때까지 대기
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.01) # 대기 중에도 통신 유지
                    state = get_robot_state()
                    if state == 1:
                        break
                    time.sleep(0.2)

                node.publish_state(False)
                print("\n✅ 데이터 기록 종료.")
                
                print("\n🔄 스캔 완료. 다음 '02' 신호를 대기합니다...")
                node.is_scanning = False # 다음 스캔을 위해 플래그 해제
                time.sleep(1.0) 

    except Exception as e:
        print(f"\n🚨 에러 발생: {e}")
        node.publish_state(False) 
        
    finally:
        print("✅ 노드를 안전하게 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()