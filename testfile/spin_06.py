import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from dsr_msgs2.srv import DrlStart
import DR_init
import time

# ==========================================
# [노드 클래스 정의] 상태 신호 발행 및 DRL 서비스 클라이언트
# ==========================================
class ScanRunnerNode(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)
        
        # 1. 상태(True/False) 토픽 퍼블리셔
        self.state_pub = self.create_publisher(Bool, f'/{namespace}/checking_state', 10)
        
        # 2. DRL 실행 서비스 클라이언트 생성
        service_name = f'/{namespace}/drl/drl_start'
        self.drl_client = self.create_client(DrlStart, service_name)
        
        # 서비스가 켜질 때까지 대기
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
        self.get_logger().info('DRL 코드를 서비스(DrlStart)를 통해 전송합니다...')
        req = DrlStart.Request()
        req.robot_system = 0        # 0: Real Robot 또는 일반 시뮬레이터
        req.code = drl_script       # DRL 문자열 삽입
        
        # 비동기로 서비스 호출
        self.drl_client.call_async(req)


# ==========================================
# [메인 함수] 로봇 이동 및 토픽 연동 로직
# ==========================================
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

        print("\n로봇 초기화 및 권한 설정 중...")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(1.0) 

        # ==========================================================
        # [Step 1] 파이썬 코드로 시작 위치(Top)까지 이동
        # ==========================================================
        print("\n🚀 초기 위치(꼭대기)로 이동 중... (이 궤적은 기록 안 됨)")
        p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        # 💡 조인트 6번 리밋 회피를 위해 시작 회전각(rz)을 90도로 변경
        rx, ry, rz = 0.0, 180.0, 90.0
        p_top = [450.00, 0.00, 250.00, rx, ry, rz]

        movej(p_ready, vel=100.0, acc=100.0)
        movel(p_top, vel=50.0, acc=100.0) 
        time.sleep(0.5)

        # ==========================================================
        # [Step 2] 시작 위치 도착! 신호 ON (이제부터 기록 시작)
        # ==========================================================
        node.publish_state(True)
        time.sleep(0.2) 

        # ==========================================================
        # [Step 3] 5층 반구 스캐닝 DRL 생성 및 서비스 호출
        # ==========================================================
        scan_drl_code = """
set_singularity_handling(1)

rx = 0.0
ry = 180.0
rz = 90.0  # 💡 DRL 내부의 시작 회전각도 90도로 동일하게 맞춰줌

p_start_1 = posx(465.45, 0.00, 247.55, rx, ry, rz)
p_via_1   = posx(450.00, 15.45, 247.55, rx, ry, rz)
p_tgt_1   = posx(434.55, 0.00, 247.55, rx, ry, rz)

movel(p_start_1, v=40, a=80, r=5.0)
movec(p_via_1, p_tgt_1, v=50, a=100, angle=360.0, ori=2, r=5.0)

# --- [2층] ---
p_start_2 = posx(479.39, 0.00, 240.45, rx, ry, rz)
p_via_2   = posx(450.00, -29.39, 240.45, rx, ry, rz)
p_tgt_2   = posx(420.61, 0.00, 240.45, rx, ry, rz)

movel(p_start_2, v=40, a=80, r=5.0)
movec(p_via_2, p_tgt_2, v=50, a=100, angle=360.0, ori=2, r=5.0)

# --- [3층] ---
p_start_3 = posx(490.45, 0.00, 229.39, rx, ry, rz)
p_via_3   = posx(450.00, 40.45, 229.39, rx, ry, rz)
p_tgt_3   = posx(409.55, 0.00, 229.39, rx, ry, rz)

movel(p_start_3, v=40, a=80, r=5.0)
movec(p_via_3, p_tgt_3, v=50, a=100, angle=360.0, ori=2, r=5.0)

# --- [4층] ---
p_start_4 = posx(497.55, 0.00, 215.45, rx, ry, rz)
p_via_4   = posx(450.00, -47.55, 215.45, rx, ry, rz)
p_tgt_4   = posx(402.45, 0.00, 215.45, rx, ry, rz)

movel(p_start_4, v=40, a=80, r=5.0)
movec(p_via_4, p_tgt_4, v=50, a=100, angle=360.0, ori=2, r=5.0)

# --- [5층] (바닥면) ---
p_start_5 = posx(500.00, 0.00, 200.00, rx, ry, rz)
p_via_5   = posx(450.00, 50.00, 200.00, rx, ry, rz)
p_tgt_5   = posx(400.00, 0.00, 200.00, rx, ry, rz)

movel(p_start_5, v=40, a=80)
movec(p_via_5, p_tgt_5, v=50, a=100, angle=360.0, ori=2)
"""
        # ROS 2 서비스 클라이언트를 이용해 전송
        node.send_drl_code(scan_drl_code)
        
        print("\n🛸 5층 반구 나선형 스캐닝 진행 중... (데이터 기록 중!)")
        time.sleep(1.0) # 제어기가 코드를 받아 실행을 시작할 여유 시간
        
        # 폴링(Polling) 로직: 로봇이 스캐닝(5층)을 완전히 마칠 때까지 대기
        while True:
            state = get_robot_state()
            if state == 1: # 1: ROBOT_STATE_IDLE
                break
            time.sleep(0.2)

        # ==========================================================
        # 💡 [Step 4] 스캐닝 완료! 복귀하기 '전'에 기록 종료 신호 OFF
        # (기존에 5번 스텝이랑 순서가 뒤집혀 있어서 수정했습니다)
        # ==========================================================
        node.publish_state(False)
        print("\n✅ 데이터 기록을 먼저 종료합니다.")
        time.sleep(1.0) # 노드들이 numpy 파일을 안전하게 저장할 시간 확보


    except Exception as e:
        print(f"\n🚨 에러 발생: {e}")
        node.publish_state(False) 
        
    finally:
        print("✅ 노드를 안전하게 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()