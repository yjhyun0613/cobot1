import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import time
import threading
import queue  # 💡 비동기 통신용 큐 추가

import firebase_admin
from firebase_admin import credentials, db

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
    
    TOOL_LENGTH_Z = 0.2586 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    
    pos_meters = (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]
    return pos_meters * 1000.0

class FirebaseBridgeNode(Node):
    def __init__(self):
        super().__init__('firebase_bridge')
        self.is_active = False
        self.record_enable = False 
        self.last_fb_time = 0.0

        try:
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-7f5d874f48.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.get_logger().info('📻 통신 방송국(Firebase Bridge) 켜짐. 대기 중...')
        except Exception as e:
            self.get_logger().error(f'Firebase 연결 실패: {e}')

        # 💡 [핵심] 파이어베이스 비동기 전송을 위한 큐와 워커 스레드
        self.fb_queue = queue.Queue()
        self.fb_worker = threading.Thread(target=self.firebase_set_worker, daemon=True)
        self.fb_worker.start()

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.enable_sub = self.create_subscription(Bool, '/dsr01/record_enable', self.enable_callback, 10)

    # =========================================================
    # 🚚 [백그라운드] 파이어베이스 전담 택배기사 (경로와 데이터를 받아서 처리)
    # =========================================================
    def firebase_set_worker(self):
        while rclpy.ok():
            try:
                ref_path, data = self.fb_queue.get(timeout=1.0)
                db.reference(ref_path).set(data)
            except queue.Empty:
                continue
            except Exception as e:
                pass # 일시적인 네트워크 에러 무시

    # =========================================================
    def enable_callback(self, msg):
        self.record_enable = msg.data

    def state_callback(self, msg):
        if msg.data == True and not self.is_active:
            self.get_logger().info('📡 로봇 동작 감지됨! 웹 생중계 시작.')
            self.is_active = True
            self.record_enable = False
            # 💡 [비동기] 큐에 던짐
            self.fb_queue.put(('robot/dsr01/status', {'is_scanning': True, 'timestamp': time.time()}))
            
        elif msg.data == False and self.is_active:
            self.get_logger().info('🛑 동작 종료. 웹 생중계를 중지합니다.')
            self.is_active = False
            self.record_enable = False
            # 💡 [비동기] 큐에 던짐
            self.fb_queue.put(('robot/dsr01/status', {'is_scanning': False, 'timestamp': time.time()}))

    def joint_callback(self, msg):
        if not self.is_active or not self.record_enable: 
            return
        
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            
            current_time = time.time()
            if current_time - self.last_fb_time > 0.33:  
                # 💡 [비동기] 큐에 위치 데이터를 던짐 (ROS 2 지연 0%)
                pos_data = {
                    'x': float(current_pos[0]),
                    'y': float(current_pos[1]),
                    'z': float(current_pos[2])
                }
                self.fb_queue.put(('robot/dsr01/current_pose', pos_data))
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