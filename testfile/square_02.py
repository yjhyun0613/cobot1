import rclpy
import DR_init
import time
from std_msgs.msg import Bool

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = ""
ROBOT_TCP = ""

# 이동 속도 및 가속도
VELOCITY = 150
ACC = 150

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)
    
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print("#" * 50)


def perform_task(node):
    """Z축을 400mm로 고정하고 사각형 그리며 상태 퍼블리시"""
    print("Performing square drawing task...")
    from DSR_ROBOT2 import posx, movej, movel

    # checking_state 퍼블리셔 생성
    state_pub = node.create_publisher(Bool, 'checking_state', 10)
    msg = Bool()

    # 대기 위치
    JReady = [0, 0, 90, 0, 90, 0]
    
    # 사각형의 4개 꼭짓점 좌표 정의 (단위: mm)
    p1 = posx([500, -100, 400, 150, 179, 150]) # 우측 하단 (시작점)
    p2 = posx([500,  100, 400, 150, 179, 150]) # 우측 상단
    p3 = posx([310,  100, 400, 150, 179, 150]) # 좌측 상단
    p4 = posx([300, -100, 400, 150, 179, 150]) # 좌측 하단

    while rclpy.ok():       
        # 1. 안전한 대기 위치로 조인트 이동
        print("Moving to Ready Position...")
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        # 2. 사각형의 시작점(p1)으로 선형 이동하여 진입
        print("Moving to Start Point (p1)...")
        movel(p1, vel=VELOCITY, acc=ACC)
        
        # p1 도착 확인 후 True 발행
        msg.data = True
        state_pub.publish(msg)
        print("--> Published checking_state: True (Data collection START)")
        
        # 3. 사각형 그리기 궤적 (p1 -> p2 -> p3 -> p4 -> p1)
        print("Drawing Square...")
        movel(p2, vel=VELOCITY, acc=ACC)
        movel(p3, vel=VELOCITY, acc=ACC)
        movel(p4, vel=VELOCITY, acc=ACC)
        movel(p1, vel=VELOCITY, acc=ACC) # 다시 시작점으로 돌아와서 사각형 완성
        
        # 사각형 완성 후 False 발행
        msg.data = False
        state_pub.publish(msg)
        print("--> Published checking_state: False (Data collection STOP & Save)")
        
        print("Square completed. Restarting loop...\n")
        time.sleep(1) # 다음 반복 전 1초 대기


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("draw_square", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task(node) # 퍼블리셔 사용을 위해 node 객체 전달
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()