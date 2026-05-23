import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import DR_init
import time
import math
import numpy as np

# =================================================================
# [1] 환경 설정 (Configuration)
# =================================================================
ROBOT_ID    = "dsr01" 
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"

# Z축 회전(Yaw) 각도를 임의로 지정 (도 단위)
TARGET_RZ = -180.0 

# =================================================================
# [2] 벡터 기반 자세 제어 수학 함수
# =================================================================
def normalize(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-6 else v

def rot_to_zyz(R):
    beta = math.acos(max(min(R[2, 2], 1.0), -1.0))
    if abs(beta) < 1e-6:
        alpha, gamma = 0.0, math.atan2(R[1, 0], R[0, 0])
    else:
        alpha, gamma = math.atan2(R[1, 2], R[0, 2]), math.atan2(R[2, 1], -R[2, 0])
    return [math.degrees(alpha), math.degrees(beta), math.degrees(gamma)]

def wrap_angle(angle):
    while angle > 180.0: angle -= 360.0
    while angle < -180.0: angle += 360.0
    return angle

def get_orientation_from_vector(target_vec):
    # 1. 수신받은 방향 벡터 정규화
    pure_normal = normalize(np.array(target_vec))
    
    # 2. 로봇 툴의 Z축 방향 결정
    # 법선 벡터(표면에서 바깥으로 나오는 방향)가 주어졌을 때, 
    # 로봇이 그 표면을 정면으로 바라보려면 Z축 방향은 법선 벡터의 반대(-pure_normal)가 되어야 합니다.
    z_axis_final = -pure_normal

    # 3. Y축, X축 직교 벡터 계산으로 회전 행렬 구성
    up = np.array([0.0, 1.0, 0.0]) if abs(z_axis_final[2]) > 0.95 else np.array([0.0, 0.0, 1.0])
    x_axis = normalize(np.cross(up, z_axis_final))
    y_axis = normalize(np.cross(z_axis_final, x_axis))
    
    # 4. 구성된 회전 행렬을 로봇이 사용하는 ZYZ 오일러 각도로 변환
    rx, ry, rz_raw = rot_to_zyz(np.column_stack((x_axis, y_axis, z_axis_final)))
    
    return wrap_angle(rx), wrap_angle(ry), rz_raw

# =================================================================
# [3] 로봇 실행 노드 (ROS2 Subscriber)
# =================================================================
class VectorMoveNode(Node):
    def __init__(self):
        super().__init__('vector_move_node', namespace=ROBOT_ID)
        
        # Float32MultiArray 메시지 구독
        self.subscription = self.create_subscription(
            Float32MultiArray,
            f'/{ROBOT_ID}/target_vector_pose',
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        self.is_moving = False
        print(f"\n📡 ROS 2 토픽 수신 대기 중... 토픽명: `/{ROBOT_ID}/target_vector_pose`")
        print("예시 퍼블리시 명령어:")
        print(f'ros2 topic pub --once /{ROBOT_ID}/target_vector_pose std_msgs/msg/Float32MultiArray "{{data: [300.0, 100.0, 100.0, 0.0, 0.0, 1.0]}}"')

    def pose_callback(self, msg):
        if self.is_moving:
            print("\n⚠️ 현재 이동 중입니다. 새 메시지 무시됨.")
            return
            
        data = msg.data
        if len(data) < 6:
            print(f"\n⚠️ 배열 길이가 짧습니다. (필요: 6, 수신: {len(data)})")
            return
            
        target_pos = data[0:3]
        target_vec = data[3:6]
        
        print(f"\n📥 수신된 목표 좌표 (X,Y,Z): {target_pos}")
        print(f"📥 수신된 방향 벡터 (Vx,Vy,Vz): {target_vec}")
        
        self.is_moving = True
        self.execute_move(target_pos, target_vec)
        self.is_moving = False

    def execute_move(self, target_pos, target_vec):
        from DSR_ROBOT2 import movej, movel
        
        # 벡터를 기반으로 Rx, Ry 자세 각도 계산
        rx, ry, _ = get_orientation_from_vector(target_vec)
        rz = TARGET_RZ
        
        target_pose = [target_pos[0], target_pos[1], target_pos[2], rx, ry, rz]
        
        print(f"🧮 계산된 로봇 목표 자세 (X, Y, Z, Rx, Ry, Rz):")
        print(f"[{target_pose[0]:.2f}, {target_pose[1]:.2f}, {target_pose[2]:.2f}, {target_pose[3]:.2f}, {target_pose[4]:.2f}, {target_pose[5]:.2f}]")
        print("🚀 해당 위치와 자세로 이동을 시작합니다...")
        
        movel(target_pose, vel=50.0, acc=50.0)
        print("✅ 목표 지점 도착 완료! 다음 명령 대기 중...")

def main(args=None):
    DR_init.__dsr__id, DR_init.__dsr__model = ROBOT_ID, ROBOT_MODEL
    rclpy.init(args=args)
    node = VectorMoveNode()
    DR_init.__dsr__node = node
     
    try:
        from DSR_ROBOT2 import set_robot_mode, set_tool, set_tcp, ROBOT_MODE_AUTONOMOUS, ROBOT_MODE_MANUAL
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        
        # 1. 초기 자세로 한 번 이동
        print("\n🚀 시작 전 안전 자세(Ready)로 이동합니다...")
        from DSR_ROBOT2 import movej
        movej([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=60.0, acc=60.0)
        
        # 2. 메시지가 올 때마다 callback 함수 실행되도록 스핀
        rclpy.spin(node)
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
