import rclpy
import DR_init
import time
import numpy as np
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
    
    # 💡 [툴 길이 적용] 그리퍼(228.6) + 툴(29.5) = 258.1mm (0.2581m)
    TOOL_LENGTH_Z = 0.2581 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]

def record_cylinder_baseline(node):
    print("🌪️ Recording Cylinder Baseline (Z: 150 -> 300, 100Hz)...")
    from DSR_ROBOT2 import posx, movej, amovel, amovec, check_motion, get_current_posj

    state_pub = node.create_publisher(Bool, '/dsr01/checking_state', 10)
    msg = Bool()

    JReady = [0, 0, 90, 0, 90, 0]
    print("Moving to Ready Position...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    baseline_points = []
    
    # ==========================================
    # 🟢 1. 전체 기록 시작 신호 (맨 아래층 시작 전)
    # ==========================================
    msg.data = True
    state_pub.publish(msg)
    print("--> Published checking_state: True (Data collection START)")

    # ==========================================
    # 2. 10mm 간격으로 층을 올리며 100Hz 기록
    # ==========================================
    for current_z in range(200, 301, 10):
        print(f"📍 Recording Layer: Z = {current_z} mm")

        p_start  = posx([500,  0, current_z, 150, 179, 150])
        p_via    = posx([475, 25, current_z, 150, 179, 150])
        p_target = posx([450,  0, current_z, 150, 179, 150])

        # A. 해당 층의 시작점으로 이동 (상승 구간 기록)
        amovel(p_start, vel=VELOCITY, acc=ACC)
        while check_motion() == 1:
            q = np.deg2rad(get_current_posj())
            baseline_points.append(forward_kinematics(q))
            time.sleep(0.01) # 100Hz
            
        # B. 360도 원호 그리기 (원형 구간 기록)
        amovec(p_via, p_target, vel=VELOCITY, acc=ACC, angle=360)
        while check_motion() == 1:
            q = np.deg2rad(get_current_posj())
            baseline_points.append(forward_kinematics(q))
            time.sleep(0.01) # 100Hz

    # ==========================================
    # 🔴 3. 전체 기록 종료 신호 (맨 위층 종료 후)
    # ==========================================
    msg.data = False
    state_pub.publish(msg)
    print("--> Published checking_state: False (Data collection STOP)")

    # 4. 데이터 저장
    np.save('baseline.npy', np.array(baseline_points))
    print(f"✅ Baseline successfully saved! (Total {len(baseline_points)} points collected)")

    print("Returning to Home...")
    movej(JReady, vel=VELOCITY, acc=ACC)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("record_cylinder_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        record_cylinder_baseline(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()