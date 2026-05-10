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
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56)[:3, 3]

class FirebaseBridgeNode(Node):
    def __init__(self):
        super().__init__('firebase_bridge')
        self.is_active = False
        self.last_fb_time = 0.0
        self.trajectory_cache = [] # 통신병이 웹 팀원을 위해 전체 궤적도 잠시 기억해둠

        # ==========================================
        # 🔥 Firebase 초기화 (통신병 세팅)
        # ==========================================
        try:
            # 🚨 본인 키 경로와 URL 필수 수정
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.db_pose_ref = db.reference('robot/dsr01/current_pose')
            self.get_logger().info('📻 통신 방송국(Firebase Bridge) 켜짐. 대기 중...')
        except Exception as e:
            self.get_logger().error(f'Firebase 연결 실패: {e}')

        # 로봇 관절값과 상태 토픽을 똑같이 구독 (엿듣기)
        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)

    def state_callback(self, msg):
        # 🟢 시작 신호가 떨어지면 방송 시작
        if msg.data == True and not self.is_active:
            self.get_logger().info('📡 로봇 동작 감지됨! 웹 생중계 시작.')
            self.is_active = True
            self.trajectory_cache = [] # 이전 궤적 캐시 비우기
            db.reference('robot/dsr01/status').set({'is_scanning': True, 'timestamp': time.time()})
            
        # 🔴 종료 신호가 떨어지면 방송 종료 및 Baseline 한방 업로드
        elif msg.data == False and self.is_active:
            self.get_logger().info('🛑 동작 종료. Baseline 궤적을 웹으로 일괄 업로드합니다.')
            self.is_active = True # 업로드 완료 전까지 락 걸기
            
            db.reference('robot/dsr01/status').set({'is_scanning': False, 'timestamp': time.time()})
            
            # 기록된 궤적이 있으면 파이어베이스에 한 방에 쏨
            if self.trajectory_cache:
                db.reference('robot/dsr01/baseline_path').set(self.trajectory_cache)
                self.get_logger().info('✅ 웹 팀원용 Baseline 업로드 완벽 성공!')
            
            self.is_active = False

    def joint_callback(self, msg):
        if not self.is_active: return
        
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            
            pose_dict = {
                'x': float(current_pos[0]),
                'y': float(current_pos[1]),
                'z': float(current_pos[2])
            }
            
            # 나중에 한방에 쏘기 위해 캐시에 저장
            self.trajectory_cache.append(pose_dict)
            
            # 💡 0.1초(10Hz)마다 파이어베이스에 실시간 생중계
            current_time = time.time()
            if current_time - self.last_fb_time > 0.1:
                self.db_pose_ref.set(pose_dict)
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