import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from dsr_msgs2.srv import DrlStart
import DR_init
import time
import math

class ScanRunnerNode(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)
        self.state_pub = self.create_publisher(Bool, f'/{namespace}/checking_state', 10)
        self.progress_pub = self.create_publisher(String, f'/{namespace}/progress_status', 10)
        
        self.trigger_scan = False
        self.is_scanning = False 
        
        self.progress_sub = self.create_subscription(
            String,
            f'/{namespace}/progress_status',
            self.progress_callback,
            10
        )
        
        service_name = f'/{namespace}/drl/drl_start'
        self.drl_client = self.create_client(DrlStart, service_name)
        
        while not self.drl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} 서비스 대기 중...')
            
        self.get_logger().info('✅ 토픽 퍼블리셔 및 DRL 서비스 클라이언트 준비 완료')

    def progress_callback(self, msg):
        if msg.data == "02":
            if not self.is_scanning and not self.trigger_scan:
                self.get_logger().info('수신된 신호 [02]: 스캔 시퀀스를 시작합니다.')
                self.trigger_scan = True

    def publish_state(self, state: bool):
        msg = Bool()
        msg.data = state
        self.state_pub.publish(msg)
        status_text = "🟢 ON (데이터 기록 시작)" if state else "🔴 OFF (데이터 기록 종료)"
        self.get_logger().info(f'신호 전송: {status_text}')

    def publish_completion(self):
        msg = String()
        msg.data = "03"
        self.progress_pub.publish(msg)
        self.get_logger().info('신호 전송: 🏁 [03] 스캔 전체 완료')

    def send_drl_code(self, drl_script):
        self.get_logger().info('DRL 코드를 전송합니다...')
        req = DrlStart.Request()
        req.robot_system = 0        
        req.code = drl_script       
        self.drl_client.call_async(req)

def main(args=None):
    # 💡 설정 변수
    R = 49.0            
    TOTAL_LAYERS = 5    
    CENTER_X = 424.15  
    CENTER_Y = 75.45    
    CENTER_Z = 59.0
    MAX_TILT_ANGLE = 30.0  # 최종 바깥쪽 기울기 각도

    ROBOT_ID = "dsr01" 
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = "m0609"
    
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
        set_tool("Tool Weight")
        set_tcp("GripperDA_v1")
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(1.0) 

        print("\n🔄 대기 모드: /dsr01/progress_status 토픽에서 '02' 신호를 기다립니다...")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if node.trigger_scan:
                node.is_scanning = True
                node.trigger_scan = False
                
                print("\n🚀 대기 위치(반구 위쪽)로 이동 중...")
                p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
                
                # 수직 대기 상태
                rx_top, ry_top, rz_top = 0.0, 179.0, 270.0
                p_top = [CENTER_X, CENTER_Y, 200.00, rx_top, ry_top, rz_top]

                movej(p_ready, vel=100.0, acc=100.0)
                movel(p_top, vel=50.0, acc=100.0) 
                time.sleep(0.5)

                # ==========================================================
                # [사전 이동] 1층의 시작 지점과 기울기를 계산하여 미리 이동
                # ==========================================================
                first_layer_tilt = MAX_TILT_ANGLE * (1.0 / TOTAL_LAYERS)
                angle_rad = math.radians(90.0 * (1.0 - 1.0/TOTAL_LAYERS))
                curr_z = CENTER_Z + R * math.sin(angle_rad)
                curr_r = R * math.cos(angle_rad)
                
                # ry(Y축 회전)를 깎아서 바깥쪽으로 올바르게 기울임
                ry_1 = -179.0 + first_layer_tilt
                p_start_1 = [CENTER_X + curr_r, CENTER_Y, curr_z, 0.0, ry_1, 270.0]
                
                print(f"\n📍 1층 스캔 지점(바깥 기울기 {first_layer_tilt:.1f}도)으로 대기 이동...")
                movel(p_start_1, vel=40.0, acc=80.0)
                time.sleep(0.5)

                # 이동 완료 후 기록 시작 신호 전송
                node.publish_state(True)
                time.sleep(0.2) 
                
                # ==========================================================
                # 💡 [DRL 실행] 올바른 방향 눕기 + 특이점 회피(반원 2개 쪼개기) 적용
                # ==========================================================
                scan_drl_code = f"""
R = {R}            
TOTAL_LAYERS = {TOTAL_LAYERS}    
CENTER_X = {CENTER_X}   
CENTER_Y = {CENTER_Y}    
CENTER_Z = {CENTER_Z} 
MAX_ANGLE = {MAX_TILT_ANGLE}

set_singularity_handling(1)
stiff = [500, 500, 500, 100, 100, 100]
task_compliance_ctrl(stiff)

rx = 0.0

for i in range(1, TOTAL_LAYERS + 1):
    angle_rad = d2r(90.0 * (1.0 - float(i)/TOTAL_LAYERS))
    curr_z = CENTER_Z + R * sin(angle_rad)
    curr_r = R * cos(angle_rad)
    
    # 층마다 Y축(ry)을 깎아서 바깥쪽으로 점점 눕게 설정 (안쪽 꺾임 문제 해결!)
    layer_tilt = MAX_ANGLE * (float(i)/TOTAL_LAYERS)
    ry = -180+layer_tilt  
    
    # 궤적 방향(시계/반시계)에 따른 경유지 2개 설정
    if (i % 2) == 1:
        rz = 270.0
        p_via1_y = CENTER_Y - curr_r  
        p_via2_y = CENTER_Y + curr_r  
    else:
        rz = -90.0
        p_via1_y = CENTER_Y + curr_r  
        p_via2_y = CENTER_Y - curr_r  

    # 원을 반으로 가르기 위한 4개의 포인트 정의
    p_start = posx(CENTER_X + curr_r, CENTER_Y, curr_z, rx, ry, rz)
    p_via1  = posx(CENTER_X, p_via1_y, curr_z, rx, ry, rz)
    p_mid   = posx(CENTER_X - curr_r, CENTER_Y, curr_z, rx, ry, rz)
    p_via2  = posx(CENTER_X, p_via2_y, curr_z, rx, ry, rz)
    p_end   = posx(CENTER_X + curr_r, CENTER_Y, curr_z, rx, ry, rz)

    movel(p_start, v=40, a=80, r=10.0)
    
    # 💡 360도를 180도짜리 반원 2개로 나누어 그리기 (3층 멈춤 에러 완벽 회피)
    movec(p_via1, p_mid, v=20, a=20, angle=180.0, r=10.0, ori=2)
    movec(p_via2, p_end, v=20, a=20, angle=180.0, r=10.0, ori=2)

release_compliance_ctrl()
"""
                node.send_drl_code(scan_drl_code)
                
                print("\n🛸 스캐닝 진행 중...")
                time.sleep(2.0) # 로봇이 움직이기 시작할 여유 시간 부여
                
                # ==========================================================
                # 💡 [핵심] 동작 완료 감지 로직 강화 (가짜 정지 무시)
                # ==========================================================
                stop_count = 0
                required_stop_time = 3.0 # 로봇이 3초 연속으로 멈춰있어야 종료로 인정
                check_interval = 0.5
                
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.01)
                    state = get_robot_state()
                    
                    if state == 1: # 로봇이 멈춘 상태(Idle)
                        stop_count += check_interval
                        if stop_count >= required_stop_time:
                            break
                    else:
                        stop_count = 0 # 로봇이 다시 움직이면 카운터 리셋!
                    
                    time.sleep(check_interval)

                node.publish_state(False)
                node.publish_completion()
                print("\n✅ 모든 프로세스 및 데이터 기록 종료.")
                
                print("\n🔄 다음 '02' 신호를 대기합니다...")
                node.is_scanning = False 

    except Exception as e:
        print(f"\n🚨 에러 발생: {e}")
        node.publish_state(False) 
        
    finally:
        print("✅ 노드를 안전하게 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()