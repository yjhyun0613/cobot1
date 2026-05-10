import rclpy
import DR_init
import time
from std_msgs.msg import Bool

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = ""
ROBOT_TCP = ""

VELOCITY = 100
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

def draw_normal_square(node):
    print("✅ Drawing NORMAL Square...")
    from DSR_ROBOT2 import posx, movej, movel

    state_pub = node.create_publisher(Bool, '/dsr01/checking_state', 10)
    msg = Bool()

    JReady = [0, 0, 90, 0, 90, 0]
    
    # 50x50 사각형의 정상적인 4개 꼭짓점
    p_start = posx([500, -25, 400, 150, 179, 150]) # 1. 우측 하단 (시작점)
    p_tr = posx([500, 25, 400, 150, 179, 150])     # 2. 우측 상단
    p_tl = posx([450, 25, 400, 150, 179, 150])     # 3. 좌측 상단
    p_bl = posx([450, -25, 400, 150, 179, 150])    # 4. 좌측 하단

    while rclpy.ok():
        print("Moving to Start...")
        movej(JReady, vel=VELOCITY, acc=ACC)
        movel(p_start, vel=VELOCITY, acc=ACC)
        
        # 데이터 수집 시작 신호 (True)
        msg.data = True
        state_pub.publish(msg)
        print("--> Published checking_state: True")
        
        # 궤적 그리기 (정상 사각형)
        movel(p_tr, vel=VELOCITY, acc=ACC)
        movel(p_tl, vel=VELOCITY, acc=ACC)
        movel(p_bl, vel=VELOCITY, acc=ACC)
        movel(p_start, vel=VELOCITY, acc=ACC)
        
        # 데이터 수집 종료 신호 (False)
        msg.data = False
        state_pub.publish(msg)
        print("--> Published checking_state: False")
        
        print("Loop finished. Restarting in 3 seconds...\n")
        time.sleep(3)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("draw_normal_square_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        draw_normal_square(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()