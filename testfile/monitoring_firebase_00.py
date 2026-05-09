import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import plotly.graph_objects as go
from scipy.spatial import cKDTree
import os
import datetime
import time

# 🔥 Firebase 연동 라이브러리
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

IMAGE_DIR = '/home/yoon/cobot_ws/src/cobot1/image'

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

class TrajectoryMonitorNode(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.is_checking = False
        self.trajectory_points = []
        self.error_detected = False
        self.ERROR_THRESHOLD = 0.005 # 5mm 이탈 기준
        self.last_fb_time = 0.0  

        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)

        # ==========================================
        # 🔥 Firebase 초기화
        # ==========================================
        try:
            cred = credentials.Certificate('/home/yoon/cobot_ws/firebase_key.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://[본인프로젝트명]-default-rtdb.asia-southeast1.firebasedatabase.app/'
                })
            self.db_pose_ref = db.reference('robot/dsr01/current_pose')
            self.get_logger().info('🔥 Firebase 연결 완료 (검사 모드)')
        except Exception as e:
            self.get_logger().error(f'Firebase 연결 실패: {e}')

        # PC 로컬 검사용 KDTree 로드
        try:
            self.baseline_points = np.load('baseline.npy')
            self.kdtree = cKDTree(self.baseline_points) 
            self.get_logger().info('✅ 로컬 baseline.npy 로드 완료. 오차 검사 준비!')
        except FileNotFoundError:
            self.get_logger().error('❌ baseline.npy 파일이 없습니다.')
            raise SystemExit

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)

    def state_callback(self, msg):
        if msg.data == True and not self.is_checking:
            self.get_logger().info('🟢 검사 시작 (실시간 오차 감시 및 방송 중...)')
            self.is_checking = True
            self.error_detected = False
            self.trajectory_points = []
            db.reference('robot/dsr01/status').set({'is_scanning': True, 'timestamp': time.time()})
            
        elif msg.data == False and self.is_checking:
            self.get_logger().info('🔴 검사 종료')
            self.is_checking = False
            db.reference('robot/dsr01/status').set({'is_scanning': False, 'timestamp': time.time()})

    def joint_callback(self, msg):
        if not self.is_checking or self.error_detected: return
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            self.trajectory_points.append(current_pos)
            
            # 1. 🔥 Firebase로 0.1초마다 좌표 송신 (웹 팀원용)
            current_time = time.time()
            if current_time - self.last_fb_time > 0.1:
                self.db_pose_ref.set({
                    'x': float(current_pos[0]),
                    'y': float(current_pos[1]),
                    'z': float(current_pos[2])
                })
                self.last_fb_time = current_time

            # 2. 💻 내 PC에서 실시간 오차 검사 (KDTree)
            min_distance, _ = self.kdtree.query(current_pos)
            if min_distance > self.ERROR_THRESHOLD:
                self.error_detected = True
                self.get_logger().error(f'🚨 이탈 감지! 오차: {min_distance*1000:.1f}mm')
                self.save_optimized_image(current_pos, min_distance)
                
        except Exception: pass

    def save_optimized_image(self, error_pos, error_dist):
        fig = go.Figure()
        fig.add_trace(go.Scatter3d(
            x=self.baseline_points[:, 0], y=self.baseline_points[:, 1], z=self.baseline_points[:, 2],
            mode='lines', line=dict(color='gray', width=2, dash='dash'), name='Baseline', opacity=0.5
        ))
        
        if self.trajectory_points:
            pts = np.array(self.trajectory_points)
            fig.add_trace(go.Scatter3d(
                x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
                mode='lines', line=dict(color='blue', width=4), name='Real Trajectory'
            ))
            
        fig.add_trace(go.Scatter3d(
            x=[error_pos[0]], y=[error_pos[1]], z=[error_pos[2]],
            mode='markers', marker=dict(color='red', size=8), name='ERROR DETECTED'
        ))
        
        fig.update_layout(title=f'🚨 Error Detected: {error_dist*1000:.1f}mm', scene=dict(aspectmode='data'))
        filename = f'error_{datetime.datetime.now().strftime("%H%M%S")}.html'
        fig.write_html(os.path.join(IMAGE_DIR, filename))

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMonitorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()