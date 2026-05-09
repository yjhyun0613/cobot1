import rclpy
import DR_init
import time
from std_msgs.msg import Bool

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 50
ACC = 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, get_robot_mode, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)

def draw_normal_circle(node):
    print("⭕ Drawing NORMAL Circle...")
    # 💡 원호 그리기 전용 명령어인 'movec'를 추가로 불러옵니다!
    from DSR_ROBOT2 import posx, movej, movel, movec, amovel

    state_pub = node.create_publisher(Bool, '/dsr01/checking_state', 10)
    msg = Bool()

    JReady = [0, 0, 90, 0, 90, 0]
    
    # 지름 50mm (반지름 25mm) 원을 그리기 위한 핵심 3포인트
    # 중심점(X:475, Y:0)을 기준으로 설정했습니다.
    p_start = posx([400, 0, 200, 90, 179, 90])   # 1. 시작점 (우측 끝)
    p_via = posx([450, 50, 200, 90, 179, 179])    # 2. 경유점 (상단)
    p_target = posx([500, 0, 200, 90, 179, 270])  # 3. 도착점 (좌측 끝)

    while rclpy.ok():
        print("Moving to Start...")
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        # 시작점까지는 직선(movel)으로 진입합니다.
        movel(p_start, vel=VELOCITY, acc=ACC) 
        
        # 데이터 수집 시작 신호
        msg.data = True
        state_pub.publish(msg)
        print("--> Published checking_state: True")
        
        # ⭕ 부드러운 원 그리기 (movec)
        # 현재 위치(p_start)에서 p_via를 거쳐 p_target으로 가되, 
        # angle=360 옵션을 주어 멈추지 않고 1바퀴를 완전히 돌아 다시 시작점으로 옵니다.
        movec(p_via, p_target, vel=VELOCITY, acc=ACC, angle=360)
        movel(p_start, vel=VELOCITY, acc=ACC)  # 원을 완성한 후 시작점으로 돌아옵니다.
        
        # 데이터 수집 종료 신호
        msg.data = False
        state_pub.publish(msg)
        print("--> Published checking_state: False")
        
        print("Loop finished. Restarting in 3 seconds...\n")
        time.sleep(3)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("draw_normal_circle_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        draw_normal_circle(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()