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
        super().__init__(node_name, namespace=namespace)
        service_name = f'/{namespace}/drl/drl_start'
        self.drl_client = self.create_client(DrlStart, service_name)
        
        while not self.drl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} 서비스 대기 중...')

    def send_drl_code(self):
        self.get_logger().info('DRL 코드 전송 중...')
        
        my_drl_script = """
set_tool("Tool Weight")
set_tcp("GripperDA_v1")

# 1. 초기 대기 위치 이동
p_ready = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
movej(p_ready, v=100.0, a=100.0)

# 2. 내려가기 직전의 허공(Z=100)으로 이동
p_start = posx(367.16, 49.33, 100.0, 0.0, 180.0, 90.0)
movel(p_start, v=50.0, a=100.0)
sleep(1.0)

# 3. 허공에서의 툴 무게(영점) 측정
base_force = get_tool_force(0)
zero_z = base_force[2] # 툴의 무게 때문에 발생하는 Z축 가짜 힘

# 4. 순응 제어 켬 (Z축 강성 100)
stiff = [3000, 3000, 100, 100, 100, 100] 
task_compliance_ctrl(stiff)

# 5. 바닥 탐색: -15N의 힘으로 스르륵 하강
fd_down = [0.0, 0.0, -25.0, 0.0, 0.0, 0.0]
fdir = [0, 0, 1, 0, 0, 0]
set_desired_force(fd_down, fdir, 1)

# 6. 바닥 터치 감지 (8N 저항)
while True:
    cur_force = get_tool_force(0)
    diff = abs(cur_force[2] - zero_z)
    
    if diff > 8.0: 
        break
    sleep(0.001)

# 7. 안착한 상태에서의 바닥 Z 좌표 읽기
pos_cur, sol = get_current_posx()
floor_z = pos_cur[2]

# 8. 💡 하강용 -15N 힘 제어 끄기
release_force()

# 8.5 💡 [핵심] 툴 무게 버티기 (영점 보상 켜기)
# 아까 허공에서 잰 무게(zero_z)만큼만 힘을 주어 툴이 쳐지지 않고 무중력처럼 떠있게 만듭니다.
fd_zero = [0.0, 0.0, zero_z-4, 0.0, 0.0, 0.0]
fdir_z = [0, 0, 1, 0, 0, 0]
set_desired_force(fd_zero, fdir_z, 0) # 0: Base 좌표계 기준

# 9. 툴 무게를 스스로 버티는 상태에서 전진!
# 로봇은 무중력 상태이므로, 바닥보다 2mm 아래(target_z)로 지정한 순수한 스프링의 장력만으로 바닥을 가볍게 훑고 갑니다.
target_z = floor_z - 10
p_scrape = posx(476.41, 49.33, target_z, 20.0, 180.0, 90.0)
movel(p_scrape, v=20.0, a=40.0)

# 10. 작업 완료 후 제어 해제 및 복귀
release_force()             # 영점 보상 끄기
release_compliance_ctrl()   # 순응 제어 끄기

p_end = posx(367.16, 49.33, 100.0, 0.0, 180.0, 0.0)
movel(p_end, v=50.0, a=100.0)
"""
        req = DrlStart.Request()
        req.robot_system = 0        
        req.code = my_drl_script    
        
        self.drl_client.call_async(req)
        self.get_logger().info('전송 완료! 로봇이 바닥을 찾고 훑기 시작합니다.')

# ==========================================
# [메인 함수] 뼈대 로직과 노드 실행
# ==========================================
def main(args=None):
    ROBOT_ID = "dsr01" 
    ROBOT_MODEL = "m0609"
    ROBOT_TOOL = "Tool Weight"
    ROBOT_TCP = "GripperDA_v1"
    
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    rclpy.init(args=args)
    node = DrlRunnerNode('drl_runner_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_robot_mode, ROBOT_MODE_AUTONOMOUS, ROBOT_MODE_MANUAL,
            get_robot_mode, set_tool, set_tcp, get_tool, get_tcp
        )

        print("로봇 초기화 및 권한 설정을 시작합니다...")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(2.0)  

        print("#" * 50)
        print(f"ROBOT_ID: {ROBOT_ID}")
        print(f"ROBOT_TCP: {get_tcp()}") 
        print(f"ROBOT_MODE (0:수동, 1:자동) : {get_robot_mode()}")
        print("#" * 50)

        node.send_drl_code()
        time.sleep(2.0)

    except Exception as e:
        print(f"에러 발생: {e}")
        
    finally:
        print("작업 완료. 노드를 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()