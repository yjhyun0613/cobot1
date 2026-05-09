import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
from scipy.spatial import cKDTree
import plotly.graph_objects as go
import os
import datetime

# 리포트 저장 경로
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
    # DSR-M0609 DH Parameters 기반 포워드 키네마틱스
    T01 = get_transform_matrix(0, 0, 0.1345, 0, 0, q[0])
    T12 = get_transform_matrix(0, 0.0062, 0, 0, -1.571, -1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[1])
    T23 = get_transform_matrix(0.411, 0, 0, 0, 0, 1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[2])
    T34 = get_transform_matrix(0, -0.368, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[3])
    T45 = get_transform_matrix(0, 0, 0, -1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[4])
    T56 = get_transform_matrix(0, -0.121, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[5])
    
    # 💡 결과를 밀리미터(mm) 단위로 변환
    pos_meters = (T01 @ T12 @ T23 @ T34 @ T45 @ T56)[:3, 3]
    return pos_meters * 1000.0

class TrajectoryMonitorNode(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.is_checking = False
        self.trajectory_points = []
        self.error_detected = False
        self.ERROR_THRESHOLD = 5.0 # 5mm 이탈 기준
        
        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)

        # 1. 기준 데이터 로드 및 KDTree 생성
        try:
            # 파일 경로를 사용자 환경에 맞춰 확인해주세요
            self.baseline_points = np.load('/home/yoon/cobot_ws/src/cobot1/cobot1/baseline.npy')
            self.kdtree = cKDTree(self.baseline_points) 
            self.get_logger().info(f'✅ baseline.npy 로드 완료 ({len(self.baseline_points)} pts)')
        except FileNotFoundError:
            self.get_logger().error('❌ baseline.npy 파일이 없습니다. 먼저 기록을 진행하세요.')
            raise SystemExit

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.get_logger().info('📡 실시간 궤적 모니터링 대기 중...')

    def state_callback(self, msg):
        if msg.data == True and not self.is_checking:
            self.get_logger().info('🟢 검사 시작 (오차 감시 중)')
            self.is_checking = True
            self.error_detected = False
            self.trajectory_points = []
        elif msg.data == False and self.is_checking:
            self.get_logger().info('🔴 검사 종료')
            self.is_checking = False

    def joint_callback(self, msg):
        if not self.is_checking or self.error_detected: return
        
        try:
            # 현재 좌표 계산
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            
            # 💡 유령 점(0,0,0) 필터링
            if current_pos[2] < 100.0: return
            
            self.trajectory_points.append(current_pos)
            
            # 2. 실시간 근접 거리 계산[cite: 5]
            min_distance, _ = self.kdtree.query(current_pos)
            
            if min_distance > self.ERROR_THRESHOLD:
                self.error_detected = True
                self.get_logger().error(f'🚨 이탈 감지! 오차: {min_distance:.1f}mm')
                self.save_html_report(current_pos, min_distance)
                
        except Exception as e:
            self.get_logger().error(f'계산 에러: {e}')

    def save_html_report(self, error_pos, error_dist):
        # 3. Plotly를 이용한 3D 에러 리포트 생성[cite: 2]
        pts = np.array(self.trajectory_points)
        fig = go.Figure()

        # [기준 궤적 - 회색 점선]
        fig.add_trace(go.Scatter3d(
            x=self.baseline_points[:, 0], y=self.baseline_points[:, 1], z=self.baseline_points[:, 2],
            mode='lines', line=dict(color='lightgray', width=2, dash='dash'), name='Baseline'
        ))

        # [실제 이동 궤적 - 파란색]
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='lines', line=dict(color='blue', width=4), name='Actual Path'
        ))

        # [에러 발생 지점 - 빨간색 큰 점][cite: 3]
        fig.add_trace(go.Scatter3d(
            x=[error_pos[0]], y=[error_pos[1]], z=[error_pos[2]],
            mode='markers', marker=dict(color='red', size=8, symbol='diamond'),
            name=f'ERROR ({error_dist:.1f}mm)'
        ))

        fig.update_layout(
            title=f'Trajectory Deviation Report (Max Error: {error_dist:.1f}mm)',
            scene=dict(aspectmode='data'),
            margin=dict(l=0, r=0, b=0, t=40)
        )

        filename = f'error_report_{datetime.datetime.now().strftime("%H%M%S")}.html'
        fig.write_html(os.path.join(IMAGE_DIR, filename))
        self.get_logger().info(f'📸 에러 리포트 저장 완료: {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()