import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import plotly.graph_objects as go
import os
import datetime
import requests
import json

# 이미지 저장 경로
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
    # DSR-M0609 DH Parameters (단위: m)
    T01 = get_transform_matrix(0, 0, 0.1345, 0, 0, q[0])
    T12 = get_transform_matrix(0, 0.0062, 0, 0, -1.571, -1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[1])
    T23 = get_transform_matrix(0.411, 0, 0, 0, 0, 1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[2])
    T34 = get_transform_matrix(0, -0.368, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[3])
    T45 = get_transform_matrix(0, 0, 0, -1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[4])
    T56 = get_transform_matrix(0, -0.121, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[5])
    
    # 💡 그리퍼(228.6mm) + 툴(30mm) = 263.6mm (0.2586m) 반영
    # Z축으로 일직선이며 각도 꺾임이 없으므로 Z축 이동값만 적용
    TOOL_LENGTH_Z = 0.2586 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    
    # 최종 TCP(Tool Center Point) 위치 계산
    pos_meters = (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]
    return pos_meters * 1000.0 # mm 단위 변환

class PCTestRecorderNode(Node):
    def __init__(self):
        super().__init__('pc_test_recorder')
        self.is_recording = False
        self.trajectory_points = []
        
        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.get_logger().info('📡 툴 끝단 기준 기록 대기 중... (필터링 제거 모드)')

    def state_callback(self, msg):
        if msg.data == True and not self.is_recording:
            self.get_logger().info('🟢 기록 시작')
            self.is_recording = True
            self.trajectory_points = []
        elif msg.data == False and self.is_recording:
            self.get_logger().info('🔴 기록 종료 및 저장 중...')
            self.is_recording = False
            self.save_data_and_plot()

    def joint_callback(self, msg):
        if not self.is_recording: return
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            self.trajectory_points.append(forward_kinematics(q))
        except KeyError: pass

    def save_data_and_plot(self):
        if not self.trajectory_points: 
            self.get_logger().error('저장할 데이터가 없습니다.')
            return

        # 💡 필터링 없이 모든 수집 데이터 사용
        pts = np.array(self.trajectory_points)

        # npy 파일 저장 (학습용 기준 데이터)
        np.save('baseline.npy', pts)
        self.get_logger().info(f'✅ npy 저장 완료 (총 {len(pts)}개 점)')

        # 파이어베이스 실시간 DB 업로드
        self.upload_to_firebase(pts)

        # Plotly 3D HTML 리포트 생성
        self.generate_html_report(pts)

    def upload_to_firebase(self, pts):
        try:
            pts_reduced = pts[::2] # 웹 부하 감소를 위한 샘플링
            data = {
                "x": pts_reduced[:, 0].tolist(),
                "y": pts_reduced[:, 1].tolist(),
                "z": pts_reduced[:, 2].tolist()
            }
            url = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app/robot/dsr01/baseline.json"
            requests.put(url, data=json.dumps(data), timeout=5.0)
            self.get_logger().info("✅ Firebase Baseline 업데이트 완료")
        except Exception as e:
            self.get_logger().error(f"🚨 Firebase 업로드 실패: {e}")

    def generate_html_report(self, pts):
        fig = go.Figure()

        # 실제 스캐닝 영역 Mesh
        fig.add_trace(go.Mesh3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            opacity=0.4, color='cyan', name='Scanned Surface'
        ))

        # 툴 끝단 이동 경로 Line
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='lines', line=dict(color='blue', width=4), name='Tool Tip Path'
        ))
        
        fig.update_layout(
            title='Tool Tip Trajectory Analysis (mm)',
            scene=dict(aspectmode='data'),
            margin=dict(l=0, r=0, b=0, t=40)
        )
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'tool_tip_trace_{timestamp}.html'
        fig.write_html(os.path.join(IMAGE_DIR, filename))
        self.get_logger().info(f'✅ HTML 리포트 저장 완료: {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = PCTestRecorderNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()