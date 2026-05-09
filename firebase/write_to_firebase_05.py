import time
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState 

# =========================================================
# 1. 수학 계산 함수 (정기구학 및 오일러 각도 변환)
# =========================================================
def dh_transform_matrix(theta, a, d, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics_m0609(joint_angles):
    d1, a2, a3 = 0.135, 0.411, 0.368
    d4, d5, d6 = 0.120, 0.120, 0.120

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
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0
    return rx, ry, rz

# =========================================================
# 2. 메인 ROS 2 노드
# =========================================================
class DoosanFirebaseNode(Node):
    def __init__(self):
        super().__init__('doosan_firebase_node')
        
        # Firebase Admin SDK 초기화 (HTML의 경로와 일치시킴)
        SERVICE_ACCOUNT_KEY_PATH = "/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json" 
        DATABASE_URL = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app" 
        
        if not firebase_admin._apps:
            try:
                cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
                firebase_admin.initialize_app(cred, {
                    'databaseURL': DATABASE_URL
                })
                self.get_logger().info("Firebase 앱 초기화 성공")
            except Exception as e:
                self.get_logger().error(f"Firebase 초기화 실패: {e}")

        # HTML이 구독하는 경로와 동일하게 설정
        self.ref = db.reference('m0609/status')
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        self.subscription = self.create_subscription(
            JointState,          
            '/dsr01/joint_states',
            self.robot_state_callback, 
            10                   
        )
        self.get_logger().info("로봇 데이터 구독 및 정기구학 연산을 시작합니다...")

    def robot_state_callback(self, msg):
        if msg.position and len(msg.position) >= 6:
            try:
                joint_dict = dict(zip(msg.name, msg.position))
                current_angles = [joint_dict[name] for name in self.joint_names]
                
                # 정기구학 계산
                T = forward_kinematics_m0609(current_angles)
                
                # X, Y, Z (mm)
                x, y, z = T[0, 3] * 1000, T[1, 3] * 1000, T[2, 3] * 1000
                
                # A, B, C (Degree)
                R = T[0:3, 0:3]
                rx, ry, rz = rotation_matrix_to_euler(R)
                a, b, c = np.rad2deg(rx), np.rad2deg(ry), np.rad2deg(rz)

                # HTML 코드의 syncCoordinates 함수 구조에 맞춰 업데이트
                self.ref.update({
                    # 조인트 정보 (UI 표시를 위해 Degree로 변환)
                    'joint1_position': round(current_angles[0], 4),
                    'joint2_position': round(current_angles[1], 4),
                    'joint3_position': round(current_angles[2], 4),
                    'joint4_position': round(current_angles[3], 4),
                    'joint5_position': round(current_angles[4], 4),
                    'joint6_position': round(current_angles[5], 4),
                    # Cartesian 정보 (XYZABC 순서의 배열)
                    'pose': [
                        round(x, 3), round(y, 3), round(z, 3),
                        round(a, 3), round(b, 3), round(c, 3)
                    ],
                    'system_status': "실시간 연결 중",
                    'latest_log': f"현재 위치: X={x:.1f}, Y={y:.1f}, Z={z:.1f}",
                    'last_update_timestamp': time.time()
                })
                
            except Exception as e:
                self.get_logger().error(f"오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DoosanFirebaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()