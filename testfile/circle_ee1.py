import rclpy
import DR_init
import time
from std_msgs.msg import Bool

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
# 💡 툴 없이 실행하기 위해 비워둠
ROBOT_TOOL = ""
ROBOT_TCP = ""

VELOCITY = 100
ACC = 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)

def draw_triple_circles(node):
    print("⭕ Drawing TRIPLE Circles (Step: 10mm)...")
    from DSR_ROBOT2 import posx, movej, movel, movec

    state_pub = node.create_publisher(Bool, '/dsr01/checking_state', 10)
    msg = Bool()

    JReady = [0, 0, 90, 0, 90, 0]
    
    # 💡 원의 중심 좌표 (X: 450, Y: 0, Z: 400)
    CENTER_X = 450.0
    CENTER_Y = 0.0
    Z_HEIGHT = 400.0
    # 자세 (A, B, C)
    RX, RY, RZ = 150.0, 179.0, 150.0

    # 💡 반지름 리스트 (50mm, 40mm, 24mm 총 3개)
    radii = [50.0, 40.0, 24.0]

    while rclpy.ok():
        # 홈 위치로 이동
        movej(JReady, vel=VELOCITY, acc=ACC)
        msg.data = True
        state_pub.publish(msg)

        for radius in radii:
            print(f"--> Current Radius: {radius}mm")
            
            # 💡 반지름에 따른 3포인트 자동 계산
            # 1. 시작점: 중심에서 X축으로 +반지름만큼 이동
            p_start = posx([CENTER_X + radius, CENTER_Y, Z_HEIGHT, RX, RY, RZ])
            # 2. 경유점: 중심에서 Y축으로 +반지름만큼 이동
            p_via = posx([CENTER_X, CENTER_Y + radius, Z_HEIGHT, RX, RY, RZ])
            # 3. 도착점: 중심에서 X축으로 -반지름만큼 이동
            p_target = posx([CENTER_X - radius, CENTER_Y, Z_HEIGHT, RX, RY, RZ])

            # 시작점으로 진입 (movel)
            movel(p_start, vel=VELOCITY, acc=ACC) 
            
            # ⭕ 원 그리기 (movec로 360도 회전)
            movec(p_via, p_target, vel=VELOCITY, acc=ACC, angle=360)
            time.sleep(0.5) # 각 원 사이의 짧은 대기
        
        msg.data = False
        state_pub.publish(msg)

        print("All 3 circles finished. Restarting loop in 3 seconds...\n")
        time.sleep(3)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("draw_triple_circle_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        draw_triple_circles(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()