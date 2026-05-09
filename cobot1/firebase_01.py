import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import time

# 🔥 파이어베이스는 이 파일에서만 관리합니다!
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

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
    
    TOOL_LENGTH_Z = 0.0 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    
    # 💡 미터(m) 단위 결과를 1000 곱해서 밀리미터(mm)로 변환! (그래프 찌그러짐 방지)
    pos_meters = (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]
    return pos_meters * 1000.0

class FirebaseBridgeNode(Node):
    def __init__(self):
        super().__init__('firebase_bridge')
        self.is_active = False
        self.last_fb_time = 0.0
        # 💡 baseline_path를 위한 trajectory_cache 리스트 완전히 제거됨

        # ==========================================
        # 🔥 Firebase 초기화 (통신병 세팅)
        # ==========================================
        try:
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.db_pose_ref = db.reference('robot/dsr01/current_pose')
            self.get_logger().info('📻 통신 방송국(Firebase Bridge) 켜짐. 대기 중...')
        except Exception as e:
            self.get_logger().error(f'Firebase 연결 실패: {e}')

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)

    def state_callback(self, msg):
        # 🟢 시작 신호가 떨어지면 방송 시작
        if msg.data == True and not self.is_active:
            self.get_logger().info('📡 로봇 동작 감지됨! 웹 생중계 시작.')
            self.is_active = True
            db.reference('robot/dsr01/status').set({'is_scanning': True, 'timestamp': time.time()})
            
        # 🔴 종료 신호가 떨어지면 방송 종료
        elif msg.data == False and self.is_active:
            self.get_logger().info('🛑 동작 종료. 웹 생중계를 중지합니다.')
            self.is_active = False
            db.reference('robot/dsr01/status').set({'is_scanning': False, 'timestamp': time.time()})

    def joint_callback(self, msg):
        if not self.is_active: return
        
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            
            # 💡 0.1초(10Hz)마다 파이어베이스에 실시간 생중계만 수행
            current_time = time.time()
            if current_time - self.last_fb_time > 0.1:
                self.db_pose_ref.set({
                    'x': float(current_pos[0]),
                    'y': float(current_pos[1]),
                    'z': float(current_pos[2])
                })
                self.last_fb_time = current_time
                
        except Exception: pass

def main(args=None):
    rclpy.init(args=args)
    node = FirebaseBridgeNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()