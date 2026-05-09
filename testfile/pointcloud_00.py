import rclpy
import DR_init
import time
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"   
ROBOT_TCP = "GripperDA_v1"   

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

def run_soft_woodpecker(node):
    print("🪶 Starting Ultimate Soft Woodpecker Scanner...")
    from DSR_ROBOT2 import (
        movel, movej, posx, get_current_posx, wait, 
        task_compliance_ctrl, set_desired_force, release_compliance_ctrl
    )

    JReady = [0, 0, 90, 0, 90, 0]
    movej(JReady, vel=100, acc=100)

    # 1. 탐색 영역 및 안전 고도 설정
    SAFE_Z = 150 # 바닥과 충분히 떨어진 안전 고도 (이동할 때의 높이)
    RX, RY, RZ = 0, 180, 0 # 수직 하방 자세
    
    # 누르는 힘 설정 (15N으로 부드럽게 누름)
    TOUCH_FORCE = -15.0 

    point_cloud = []
    waypoints = []

    # 2. 100x100 지그재그 웨이포인트 생성 (가로 10 x 세로 10)
    x_values = list(range(400, 500, 10))
    for i, x in enumerate(x_values):
        if i % 2 == 0:
            y_values = list(range(-50, 50, 10))
        else:
            y_values = list(range(40, -51, -10))
        for y in y_values:
            waypoints.append(posx([x, y, SAFE_Z, RX, RY, RZ]))

    print(f"✅ Created {len(waypoints)} grid points. Start Scanning!")

    # 첫 번째 점 상공으로 이동
    movel(waypoints[0], vel=100, acc=100)
    wait(1.0)

    # ==========================================
    # 3. 딱따구리 모션 시작 (100번 반복)
    # ==========================================
    for i, target_pos in enumerate(waypoints):
        x, y = target_pos[0], target_pos[1]
        print(f"📍 Point {i+1}/100 -> Scanning X: {x}, Y: {y}")
        
        # 1) 해당 좌표의 안전 고도로 이동
        movel(target_pos, vel=100, acc=100)
        
        # 2) 순응 제어 ON (Z축만 말랑하게 500)
        task_compliance_ctrl([3000, 3000, 500, 3000, 3000, 3000])
        wait(0.5) # 관절이 부드러워질 시간
        
        # 3) 아래 방향(Z)으로 15N 힘 가하기 (부드러운 하강 시작)
        # amovel 없이 오직 이 힘만으로 로봇이 스스로 내려갑니다!
        set_desired_force(fd=[0, 0, TOUCH_FORCE, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
        
        # 4) 바닥 터치 감지 (Z 고도가 더 이상 안 내려가면 바닥!)
        wait(0.5) # 출발 시간 확보
        prev_z = get_current_posx()[0][2]
        stop_count = 0
        
        while True:
            current_z = get_current_posx()[0][2]
            dz = abs(current_z - prev_z)
            
            # 0.1초 동안 0.5mm 이하로 움직였다면 멈춘 것 (바닥 도달)
            if dz < 0.5: 
                stop_count += 1
            else:
                stop_count = 0 # 계속 내려가고 있으면 리셋
                
            # 3번 연속(0.3초) 멈춰있으면 확실한 터치로 인정!
            if stop_count >= 3:
                break
                
            prev_z = current_z
            wait(0.1)

        # 5) 터치된 좌표 기록
        hit_pos = get_current_posx()[0]
        point_cloud.append(hit_pos[:3])
        print(f"   🎯 Touched at Z: {hit_pos[2]:.1f}mm")
        
        # 6) 순응 제어 풀고 다시 안전 고도로 복귀 (딱따구리 머리 들기)
        release_compliance_ctrl()
        wait(0.2)
        movel(target_pos, vel=100, acc=100)

    # ==========================================
    # 4. 3D 렌더링
    # ==========================================
    print("✅ 모든 포인트 스캔 완료! 3D 렌더링을 시작합니다.")
    movej(JReady, vel=100, acc=100)

    pts_array = np.array(point_cloud)
    xs, ys, zs = pts_array[:, 0], pts_array[:, 1], pts_array[:, 2]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(xs, ys, zs, c=zs, cmap='plasma', marker='o', s=30)
    
    # 1:1:1 비율 고정
    max_range = np.array([np.ptp(xs), np.ptp(ys), np.ptp(zs)]).max() / 2.0
    mid_x = (np.max(xs) + np.min(xs)) * 0.5
    mid_y = (np.max(ys) + np.min(ys)) * 0.5
    mid_z = (np.max(zs) + np.min(zs)) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.set_title('Soft Woodpecker 3D Scan')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    fig.colorbar(scatter, shrink=0.5, aspect=5, label='Z Height (mm)')
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("soft_woodpecker_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        run_soft_woodpecker(node)
    except KeyboardInterrupt:
        pass
    finally:
        from DSR_ROBOT2 import release_compliance_ctrl
        try: release_compliance_ctrl()
        except: pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()