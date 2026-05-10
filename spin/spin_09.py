import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from dsr_msgs2.srv import DrlStart
import DR_init
import time

class ScanRunnerNode(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)
        self.state_pub = self.create_publisher(Bool, f'/{namespace}/checking_state', 10)
        
        service_name = f'/{namespace}/drl/drl_start'
        self.drl_client = self.create_client(DrlStart, service_name)
        
        while not self.drl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} 서비스 대기 중...')
            
        self.get_logger().info('✅ 토픽 퍼블리셔 및 DRL 서비스 클라이언트 준비 완료')

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

        print("\n🚀 초기 위치(반구 위쪽)로 이동 중...")
        p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        # 💡 원주 구속 시 특이점 발작을 막기 위한 ry=179도 설정
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
    
    # 💡 ori=2로 인해 꼬인 조인트를 지그재그로 완벽하게 풀며 이동
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
    
    # 💡 ori=2 적용 완수!
    movec(p_via, p_tgt, v=50, a=100, angle=360.0, ori=2)

release_compliance_ctrl()
"""
        node.send_drl_code(scan_drl_code)
        
        print("\n🛸 ori=2 원주 구속 스캐닝 진행 중...")
        time.sleep(1.0) 
        
        while True:
            state = get_robot_state()
            if state == 1:
                break
            time.sleep(0.2)

        node.publish_state(False)
        print("\n✅ 데이터 기록 종료.")
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