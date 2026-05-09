import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import plotly.graph_objects as go
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

class BaselineRecorderNode(Node):
    def __init__(self):
        super().__init__('baseline_recorder')
        self.is_recording = False
        self.trajectory_points = []
        
        # 💡 전송 속도 조절용 타이머 변수
        self.last_fb_time = 0.0  

        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)

        # ==========================================
        # 🔥 Firebase 초기화
        # ==========================================
        try:
            # 본인의 json 키 파일 경로와 프로젝트 URL로 수정해주세요!
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.db_pose_ref = db.reference('robot/dsr01/current_pose')
            self.get_logger().info('🔥 Firebase 연결 완료 (기록 모드)')
        except Exception as e:
            self.get_logger().error(f'Firebase 연결 실패: {e}')

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)

    def state_callback(self, msg):
        if msg.data == True and not self.is_recording:
            self.get_logger().info('🟢 기록 시작 (Firebase 실시간 전송 중...)')
            self.is_recording = True
            self.trajectory_points = []
            db.reference('robot/dsr01/status').set({'is_scanning': True, 'timestamp': time.time()})
            
        elif msg.data == False and self.is_recording:
            self.get_logger().info('🔴 기록 종료 및 데이터 저장 중...')
            self.is_recording = False
            db.reference('robot/dsr01/status').set({'is_scanning': False, 'timestamp': time.time()})
            self.save_baseline_and_upload()

    def joint_callback(self, msg):
        if not self.is_recording: return
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            self.trajectory_points.append(current_pos)
            
            # ==========================================
            # 💡 0.1초(10Hz)마다 파이어베이스에 좌표 전송
            # ==========================================
            current_time = time.time()
            if current_time - self.last_fb_time > 0.1:
                self.db_pose_ref.set({
                    'x': float(current_pos[0]),
                    'y': float(current_pos[1]),
                    'z': float(current_pos[2])
                })
                self.last_fb_time = current_time
                
        except Exception: pass

    def save_baseline_and_upload(self):
        if not self.trajectory_points: return
        pts = np.array(self.trajectory_points)
        
        # 1. PC에 npy 저장
        np.save('baseline.npy', pts)
        
        # 2. Firebase에 웹 팀원용 Baseline 전체 궤적 업로드
        try:
            self.get_logger().info('🔥 웹 시각화용 Baseline 궤적 업로드 중...')
            baseline_data = [{'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2])} for p in pts]
            db.reference('robot/dsr01/baseline_path').set(baseline_data)
            self.get_logger().info('✅ Firebase Baseline 업로드 성공!')
        except Exception as e:
            self.get_logger().error(f'Firebase 업로드 실패: {e}')
        
        # 3. Plotly 3D HTML 렌더링 (이전 코드와 동일)
        fig = go.Figure()
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='lines', line=dict(color='green', width=4), name='Baseline Path'
        ))
        fig.update_layout(title='Recorded Baseline', scene=dict(aspectmode='data'))
        filename = f'baseline_{datetime.datetime.now().strftime("%H%M%S")}.html'
        fig.write_html(os.path.join(IMAGE_DIR, filename))
        self.get_logger().info(f'📸 HTML 리포트 저장 완료')

def main(args=None):
    rclpy.init(args=args)
    node = BaselineRecorderNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()