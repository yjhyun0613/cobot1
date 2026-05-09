import rclpy
import DR_init
import time

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

def draw_stacked_circles(node):
    print("🌪️ Drawing Stacked Circles (Z: 150mm -> 300mm)...")
    from DSR_ROBOT2 import posx, movej, movel, movec

    JReady = [0, 0, 90, 0, 90, 0]

    print("Moving to Ready Position...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 💡 Python의 range 함수를 사용하여 150부터 300까지 5씩 증가시킵니다.
    # (301로 적어야 300까지 포함되어 실행됩니다)
    for current_z in range(150, 301, 5):
        print(f"📍 Current Layer: Z = {current_z} mm")

        # 반복문을 돌 때마다 Z값(current_z)만 바뀌고 X, Y는 동일한 포인트를 생성합니다.
        p_start  = posx([500,  0, current_z, 150, 179, 150])
        p_via    = posx([475, 25, current_z, 150, 179, 150])
        p_target = posx([450,  0, current_z, 150, 179, 150])

        # 1. 현재 높이의 시작점(우측 끝)으로 이동
        # 원을 한 바퀴 다 그리고 나면, 이 명령어를 통해 수직으로 5mm 징~ 하고 올라갑니다.
        movel(p_start, vel=VELOCITY, acc=ACC)
        
        # 2. 해당 높이에서 360도 완벽한 원 그리기
        movec(p_via, p_target, vel=VELOCITY, acc=ACC, angle=360)

    print("✅ All layers completed successfully! (Reached Z = 300mm)")
    
    # 작업이 끝나면 안전하게 원래 대기 위치로 복귀
    print("Returning to Home...")
    movej(JReady, vel=VELOCITY, acc=ACC)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("draw_stacked_circles_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        draw_stacked_circles(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()