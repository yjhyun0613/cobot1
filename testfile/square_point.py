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
    time.sleep(0.5)
    set_tool(ROBOT_TOOL)
    time.sleep(0.5)
    set_tcp(ROBOT_TCP)
    time.sleep(0.5)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)

def draw_grid_100_points(node):
    print("🔲 Starting 100-Point Grid (Stop & Go)...")
    from DSR_ROBOT2 import posx, movej, movel

    JReady = [0, 0, 90, 0, 90, 0]
    print("Moving to Ready Position...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 💡 Z 높이는 300으로 고정, 로봇이 바닥을 똑바로 내려다보는 자세(0, 180, 0)
    Z_HEIGHT = 300
    RX, RY, RZ = 0, 180, 0
    waypoints = []

    # X축 10번 (400~490), Y축 10번 (-50~40) 반복하여 총 100개의 점 생성
    x_values = list(range(400, 500, 10))

    for i, x in enumerate(x_values):
        # 💡 [핵심 최적화] 지그재그(스네이크) 패턴 이동
        # 짝수 줄은 정방향으로, 홀수 줄은 역방향으로 생성하여 로봇의 이동 낭비를 없앱니다.
        if i % 2 == 0:
            y_values = list(range(-50, 50, 10))   # 정방향 (-50 부터 40 까지)
        else:
            y_values = list(range(40, -51, -10))  # 역방향 (40 부터 -50 까지)

        for y in y_values:
            waypoints.append(posx([x, y, Z_HEIGHT, RX, RY, RZ]))

    print(f"✅ Created {len(waypoints)} grid points. Starting movement...")

    # 첫 번째 점으로 먼저 이동
    movel(waypoints[0], vel=VELOCITY, acc=ACC)
    time.sleep(1) # 시작 전 잠깐 숨 고르기

    # ==========================================
    # 100개의 점을 순회하며 이동 후 잠깐 멈춤 (Stop & Go)
    # ==========================================
    for i, target_pos in enumerate(waypoints):
        # 현재 향하는 X, Y 좌표를 터미널에 출력
        print(f"📍 Point {i+1}/100 -> Moving to: X={target_pos[0]}, Y={target_pos[1]}")
        
        # 해당 위치로 부드럽게 이동
        movel(target_pos, vel=VELOCITY, acc=ACC)
        
        # 💡 도착하면 0.5초 대기 (이 시간을 늘리거나 줄여서 멈추는 느낌을 조절하세요)
        time.sleep(0.5) 

    print("✅ 100 Points completed! Returning to Home...")
    movej(JReady, vel=VELOCITY, acc=ACC)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("grid_100_points_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        draw_grid_100_points(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()