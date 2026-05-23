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
    time.sleep(0.5)

    set_tool(ROBOT_TOOL)
    time.sleep(0.5)

    set_tcp(ROBOT_TCP)
    time.sleep(0.5)

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
    
    # 부품 끝단 기준 (Z축 길이 적용)
    TOOL_LENGTH_Z = 0.200 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]

def create_baseline(node):
    print("🛠️ Creating Baseline Data (50x50 Square) at 100Hz...")
    # 💡 amovel(비동기 이동)과 check_motion(이동 상태 확인)을 추가로 임포트합니다.
    from DSR_ROBOT2 import posx, movej, movel, amovel, check_motion, get_current_posj

    state_pub = node.create_publisher(Bool, '/dsr01/checking_state', 10)
    msg = Bool()

    JReady = [0, 0, 90, 0, 90, 0]
    waypoints = []
    
    # 50x50 사각형 (높이 200)
    for y in np.linspace(-25, 25, 10): waypoints.append(posx([500, y, 200, 150, 179, 150]))
    for x in np.linspace(500, 450, 10): waypoints.append(posx([x, 25, 200, 150, 179, 150]))
    for y in np.linspace(25, -25, 10): waypoints.append(posx([450, y, 200, 150, 179, 150]))
    for x in np.linspace(450, 500, 10): waypoints.append(posx([x, -25, 200, 150, 179, 150]))
    waypoints.append(posx([500, -25, 200, 150, 179, 150]))

    baseline_points = []
    
    # 1. 대기 위치 및 시작점 이동 (이때는 동기식 movel 사용)
    print("Moving to Ready Position and Start Point...")
    movej(JReady, vel=VELOCITY, acc=ACC)
    movel(waypoints[0], vel=VELOCITY, acc=ACC)
    
    # 수집 시작 신호
    msg.data = True
    state_pub.publish(msg)
    print("--> Published checking_state: True (Data collection START)")

    # 2. 100Hz(0.01초) 주기로 궤적 데이터 실시간 수집
    print("Drawing Square and recording coordinates at 100Hz...")
    for target_pos in waypoints:
        amovel(target_pos, vel=VELOCITY, acc=ACC) # 💡 비동기 명령: 로봇에게 목적지를 던져주고 코드는 바로 아래로 넘어감
        
        # 💡 로봇이 움직이는(check_motion()이 1인) 동안 0.01초 간격으로 무한 기록!
        while check_motion() == 1:
            current_joint = get_current_posj()
            current_joint_rad = np.deg2rad(current_joint)
            pos = forward_kinematics(current_joint_rad)
            baseline_points.append(pos)
            
            time.sleep(0.01) # 💡 100Hz 제어 주기 적용 (1초에 100번 sleep)

    msg.data = False
    state_pub.publish(msg)
    print("--> Published checking_state: False (Data collection STOP)")

    # 3. numpy 배열로 저장
    np.save('baseline.npy', np.array(baseline_points))
    print(f"✅ Baseline data successfully saved! (Total {len(baseline_points)} points collected at 100Hz)")

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