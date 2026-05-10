import rclpy
import DR_init
import time
import numpy as np
import os
from std_msgs.msg import Bool

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 100
ACC = 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, get_robot_mode, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_MANUAL)
    time.sleep(0.5)  # 💡 수동 모드로 완전히 넘어갈 때까지 대기

    set_tool(ROBOT_TOOL)
    time.sleep(0.5)  # 💡 컨트롤러가 툴 무게를 레지스터에 기록할 시간 대기

    set_tcp(ROBOT_TCP)
    time.sleep(0.5)  # 💡 TCP 길이를 시스템에 적용할 시간 대기

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)

def get_transform_matrix(x, y, z, rx=0, ry=0, rz=0):
    cx, cy, cz = np.cos([rx, ry, rz])
    sx, sy, sz = np.sin([rx, ry, rz])
    return np.array([
        [cy*cz, cz*sx*sy - cx*sz, sx*sz + cx*cz*sy, x],
        [cy*sz, cx*cz + sx*sy*sz, cx*sy*sz - cz*sx, y],
        [-sy,   cy*sx,            cx*cy,            z],
        [0,     0,                0,                1]
    ])

def forward_kinematics(q):
    T01 = get_transform_matrix(0, 0, 0.1345, 0, 0, q[0])
    T12 = get_transform_matrix(0, 0.0062, 0, 0, -1.571, -1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[1])
    T23 = get_transform_matrix(0.411, 0, 0, 0, 0, 1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[2])
    T34 = get_transform_matrix(0, -0.368, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[3])
    T45 = get_transform_matrix(0, 0, 0, -1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[4])
    T56 = get_transform_matrix(0, -0.121, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[5])
    
    # 💡 [추가된 부분] 툴 끝단 기준 좌표를 얻기 위한 툴 길이 적용 (단위: 미터)
    TOOL_LENGTH_Z = 0.200 # <-- 실제 툴의 총 길이에 맞춰 수정해주세요! (현재 20cm 가정)
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]

def create_baseline(node):
    print("🛠️ Creating Baseline Data (50x50 Square)...")
    from DSR_ROBOT2 import posx, movej, movel, wait, get_current_posj

    state_pub = node.create_publisher(Bool, '/dsr01/checking_state', 10)
    msg = Bool()

    JReady = [0, 0, 90, 0, 90, 0]
    
    # 💡 [수정된 부분] Z축 높이를 400 -> 200으로 낮춤
    waypoints = []
    
    # 밑변 (X: 500, Y: -25 -> 25)
    for y in np.linspace(-25, 25, 10): waypoints.append(posx([500, y, 200, 150, 179, 150]))
    # 우측변 (X: 500 -> 450, Y: 25)
    for x in np.linspace(500, 450, 10): waypoints.append(posx([x, 25, 200, 150, 179, 150]))
    # 윗변 (X: 450, Y: 25 -> -25)
    for y in np.linspace(25, -25, 10): waypoints.append(posx([450, y, 200, 150, 179, 150]))
    # 좌측변 (X: 450 -> 500, Y: -25)
    for x in np.linspace(450, 500, 10): waypoints.append(posx([x, -25, 200, 150, 179, 150]))
    
    # 마지막 점을 첫 점과 연결
    waypoints.append(posx([500, -25, 200, 150, 179, 150]))

    baseline_points = []
    
    # 1. 대기 위치 및 시작점 이동
    print("Moving to Ready Position and Start Point...")
    movej(JReady, vel=VELOCITY, acc=ACC)
    movel(waypoints[0], vel=VELOCITY, acc=ACC)
    
    # 수집 시작 신호
    msg.data = True
    state_pub.publish(msg)
    print("--> Published checking_state: True (Data collection START)")

    # 2. 웨이포인트를 따라 촘촘하게 이동하며 좌표 직접 계산 후 저장
    print("Drawing Square and saving coordinates...")
    for target_pos in waypoints:
        movel(target_pos, vel=VELOCITY, acc=ACC) 
        
        # 현재 조인트 각도를 얻어와서 끝단 좌표 계산
        current_joint = get_current_posj()
        current_joint_rad = np.deg2rad(current_joint)
        pos = forward_kinematics(current_joint_rad)
        baseline_points.append(pos)
        time.sleep(0.05) 

    msg.data = False
    state_pub.publish(msg)
    print("--> Published checking_state: False (Data collection STOP)")

    # 3. numpy 배열로 저장
    np.save('baseline.npy', np.array(baseline_points))
    print("✅ Baseline data successfully saved to 'baseline.npy'")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("create_baseline_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        create_baseline(node)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()