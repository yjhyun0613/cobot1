import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

# ==========================================
# 1. 수학 계산 함수 (정기구학 및 회전각 변환)
# ==========================================
def dh_transform_matrix(theta, a, d, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics_m0609(joint_angles):
    d1, a2, a3 = 0.135, 0.411, 0.368
    d4, d5, d6 = 0.120, 0.120, 0.333

    dh_params = [
        [joint_angles[0],  0, d1,  np.pi/2],
        [joint_angles[1], a2,  0,        0],
        [joint_angles[2], a3,  0,        0],
        [joint_angles[3],  0, d4,  np.pi/2],
        [joint_angles[4],  0, d5, -np.pi/2],
        [joint_angles[5],  0, d6,        0]
    ]

    T_base_to_end = np.eye(4)
    for params in dh_params:
        theta, a, d, alpha = params
        T_link = dh_transform_matrix(theta, a, d, alpha)
        T_base_to_end = np.dot(T_base_to_end, T_link)
        
    return T_base_to_end

def rotation_matrix_to_euler(R):
    """
    3x3 회전 행렬을 Roll(Rx), Pitch(Ry), Yaw(Rz) 각도(라디안)로 변환합니다.
    (Z-Y-X 오일러 각도 컨벤션 기준)
    """
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6 # 특이점(Gimbal lock) 확인

    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0

    return rx, ry, rz

# ==========================================
# 2. ROS 2 노드 클래스
# ==========================================
class M0609KinematicsNode(Node):
    def __init__(self):
        super().__init__('m0609_fk_node')
        
        self.subscription = self.create_subscription(
            JointState,
            '/dsr01/joint_states', 
            self.joint_callback,
            10
        )
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        self.get_logger().info('M0609 정기구학 노드가 시작되었습니다. (위치 및 회전각 출력 대기 중...)')

    def joint_callback(self, msg):
        try:
            joint_dict = dict(zip(msg.name, msg.position))
            current_angles = [joint_dict[name] for name in self.joint_names]
            
            # 정기구학 계산 (4x4 변환 행렬 도출)
            final_transform = forward_kinematics_m0609(current_angles)
            
            # 1. 위치(X, Y, Z) 추출 (4번째 열)
            x = final_transform[0, 3]
            y = final_transform[1, 3]
            z = final_transform[2, 3]
            
            # 2. 회전각(A, B, C) 추출
            # 왼쪽 위 3x3 부분만 잘라내어 회전 행렬로 사용
            R = final_transform[0:3, 0:3]
            rx, ry, rz = rotation_matrix_to_euler(R)
            
            # 직관적인 확인을 위해 라디안을 도(Degree)로 변환
            roll_A = np.rad2deg(rx)
            pitch_B = np.rad2deg(ry)
            yaw_C = np.rad2deg(rz)
            
            # 터미널에 실시간 출력
            self.get_logger().info(
                f'위치(m) -> X: {x:.4f}, Y: {y:.4f}, Z: {z:.4f} | '
                f'회전(°) -> A(Rx): {roll_A:.2f}, B(Ry): {pitch_B:.2f}, C(Rz): {yaw_C:.2f}'
            )
            
        except KeyError as e:
            self.get_logger().warning(f'조인트 이름을 찾을 수 없습니다: {e}')

# ==========================================
# 3. 메인 함수
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    node = M0609KinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료합니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()