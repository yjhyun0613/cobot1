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

def draw_error_square(node):
    print("⚠️ Drawing Square with an intentional ERROR...")
    from DSR_ROBOT2 import posx, movej, movel

    state_pub = node.create_publisher(Bool, '/dsr01/checking_state', 10)
    msg = Bool()

    JReady = [0, 0, 90, 0, 90, 0]
    
    # 50x50 사각형의 주요 포인트들
    p_start = posx([500, -25, 400, 150, 179, 150]) # 우측 하단
    p_tr = posx([500, 25, 400, 150, 179, 150])    # 우측 상단
    
    # 💥 에러 발생 지점: 우측 변으로 가다가 바깥쪽(X=505)으로 15mm 튀어나감
    p_error = posx([510, 0, 400, 150, 179, 150])  
    
    p_tl = posx([450, 25, 400, 150, 179, 150])    # 좌측 상단
    p_bl = posx([450, -25, 400, 150, 179, 150])   # 좌측 하단

    while rclpy.ok():
        print("Moving to Start...")
        movej(JReady, vel=VELOCITY, acc=ACC)
        movel(p_start, vel=VELOCITY, acc=ACC)
        
        # 데이터 수집 시작 신호 (True)
        msg.data = True
        state_pub.publish(msg)
        print("--> Published checking_state: True")
        
        # 궤적 그리기 (에러 포인트 포함)
        movel(p_error, vel=VELOCITY, acc=ACC) # 툭 튀어나가는 부분!
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
    node = rclpy.create_node("draw_error_square_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        draw_error_square(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()