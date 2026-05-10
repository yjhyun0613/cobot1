import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import DR_init
import time
import sys

# ==========================================
# [노드 클래스 정의] DRL 코드를 전송하는 역할
# ==========================================
class DrlRunnerNode(Node):
    def __init__(self, node_name, namespace):
        # 상속받은 Node 클래스 초기화 (네임스페이스 적용)
        super().__init__(node_name, namespace=namespace)
        
        # DRL 스크립트 실행 서비스 클라이언트 생성 (동적으로 네임스페이스 적용)
        service_name = f'/{namespace}/drl/drl_start'
        self.drl_client = self.create_client(DrlStart, service_name)
        
        while not self.drl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} 서비스 대기 중...')

    def send_drl_code(self):
        self.get_logger().info('DRL 코드 전송 중...')
        
        # 파이썬 문자열로 DRL 코드 작성
        my_drl_script = """
# ==========================================================
# [DART Studio 실행용] 반구 표면 스무스 나선형 (수직 + 원주구속)
# 툴 방향: 항상 바닥과 수직 (아래 방향)
# 특이사항: ori=1 적용으로 툴이 돌면서 앞면은 중심을 향함
# ==========================================================

# 1. 초기 안전 자세 설정
p_ready = posj(0, 0, 90, 0, 90, 0)
movej(p_ready, v=100, a=100)

# ==========================================================
# 💡 툴 자세 고정 변수
# rx, ry는 바닥을 보도록 고정하고, rz는 0으로 둡니다.
# (ori=1이 활성화되면 rz는 로봇이 알아서 계산하며 돌립니다!)
# ==========================================================
rx = 0.0
ry = 180.0
rz = 0.0

# 2. 반구 꼭대기 (Top)
p_top = posx(450.00, 0.00, 250.00, rx, ry, rz)
movel(p_top, v=50, a=100, r=5.0)

# ==========================================================
# 3. 층별 하강 스캐닝 시작 (ori=1 추가!)
# ==========================================================

# --- [1층] ---
p_start_1 = posx(465.45, 0.00, 247.55, rx, ry, rz)
p_via_1   = posx(450.00, 15.45, 247.55, rx, ry, rz)
p_tgt_1   = posx(434.55, 0.00, 247.55, rx, ry, rz)

movel(p_start_1, v=40, a=80, r=5.0)
# 💡 정방향(CW) + 원주 구속(ori=1) 추가
movec(p_via_1, p_tgt_1, v=50, a=100, angle=360.0, ori=2, r=5.0)


# --- [2층] ---
p_start_2 = posx(479.39, 0.00, 240.45, rx, ry, rz)
p_via_2   = posx(450.00, -29.39, 240.45, rx, ry, rz) # 꼬임 방지 (Y 반대)
p_tgt_2   = posx(420.61, 0.00, 240.45, rx, ry, rz)

movel(p_start_2, v=40, a=80, r=5.0)
# 💡 역방향(CCW) + 원주 구속(ori=1) 추가
movec(p_via_2, p_tgt_2, v=50, a=100, angle=360.0, ori=2, r=5.0)


# --- [3층] ---
p_start_3 = posx(490.45, 0.00, 229.39, rx, ry, rz)
p_via_3   = posx(450.00, 40.45, 229.39, rx, ry, rz)
p_tgt_3   = posx(409.55, 0.00, 229.39, rx, ry, rz)

movel(p_start_3, v=40, a=80, r=5.0)
# 💡 정방향(CW) + 원주 구속(ori=1) 추가
movec(p_via_3, p_tgt_3, v=50, a=100, angle=360.0, ori=2, r=5.0)


# --- [4층] ---
p_start_4 = posx(497.55, 0.00, 215.45, rx, ry, rz)
p_via_4   = posx(450.00, -47.55, 215.45, rx, ry, rz) # 꼬임 방지 (Y 반대)
p_tgt_4   = posx(402.45, 0.00, 215.45, rx, ry, rz)

movel(p_start_4, v=40, a=80, r=5.0)
# 💡 역방향(CCW) + 원주 구속(ori=1) 추가
movec(p_via_4, p_tgt_4, v=50, a=100, angle=360.0, ori=2, r=5.0)


# --- [5층] (바닥면) ---
p_start_5 = posx(500.00, 0.00, 200.00, rx, ry, rz)
p_via_5   = posx(450.00, 50.00, 200.00, rx, ry, rz)
p_tgt_5   = posx(400.00, 0.00, 200.00, rx, ry, rz)

# 마지막 층은 정확히 서야 하므로 블렌딩(r) 제거
movel(p_start_5, v=40, a=80)
# 💡 정방향(CW) + 원주 구속(ori=1) 추가
movec(p_via_5, p_tgt_5, v=50, a=100, angle=360.0, ori=2)

# 4. 종료 후 안전 위치 복귀
movel(p_top, v=50, a=100)
"""
        req = DrlStart.Request()
        req.robot_system = 0        # 0: Real Robot 또는 일반 시뮬레이터
        req.code = my_drl_script    # 작성한 스크립트를 그대로 삽입
        
        self.drl_client.call_async(req)
        self.get_logger().info('전송 완료! 로봇이 순정 원주구속자세로 움직입니다.')


# ==========================================
# [메인 함수] 뼈대 로직과 노드 실행
# ==========================================
def main(args=None):
    # 1. 로봇 환경 변수 설정
    ROBOT_ID = "dsr01" # 환경에 맞게 네임스페이스 확인 후 수정 (예: dsr01)
    ROBOT_MODEL = "m0609"
    ROBOT_TOOL = "Tool Weight"
    ROBOT_TCP = "GripperDA_v1"
    
    # 2. 내부 변수 초기화 및 ROS 2 노드 생성
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    rclpy.init(args=args)
    
    # 위에서 정의한 클래스를 통해 노드 객체 생성
    node = DrlRunnerNode('drl_runner_node', namespace=ROBOT_ID)
    
    # 생성된 노드 객체를 DR_init에 연결 (매우 중요!)
    DR_init.__dsr__node = node

    try:
        # 3. DSR_ROBOT2 임포트 (DR_init 설정 완료 후 임포트해야 함)
        from DSR_ROBOT2 import (
            set_robot_mode, ROBOT_MODE_AUTONOMOUS, ROBOT_MODE_MANUAL,
            get_robot_mode, set_tool, set_tcp, get_tool, get_tcp
        )

        # 4. 로봇 제어 권한 획득 및 설정
        print("로봇 초기화 및 권한 설정을 시작합니다...")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(2.0)  # 설정 반영 대기

        # 설정 상태 출력
        print("#" * 50)
        print(f"ROBOT_ID: {ROBOT_ID}")
        print(f"ROBOT_TCP: {get_tcp()}") 
        print(f"ROBOT_MODE (0:수동, 1:자동) : {get_robot_mode()}")
        print("#" * 50)

        # 5. DRL 스크립트 전송 실행
        node.send_drl_code()
        
        # 명령이 컨트롤러 큐에 안정적으로 넘어갈 시간 확보
        time.sleep(2.0)

    except Exception as e:
        print(f"에러 발생: {e}")
        
    finally:
        # 6. 종료 및 정리
        print("작업 완료. 노드를 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()