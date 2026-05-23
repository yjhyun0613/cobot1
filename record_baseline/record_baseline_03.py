import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import plotly.graph_objects as go  # 💡 Matplotlib 대신 Plotly 사용
import os
import datetime

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
    T01 = get_transform_matrix(0, 0, 0.1345, 0, 0, q[0])
    T12 = get_transform_matrix(0, 0.0062, 0, 0, -1.571, -1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[1])
    T23 = get_transform_matrix(0.411, 0, 0, 0, 0, 1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[2])
    T34 = get_transform_matrix(0, -0.368, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[3])
    T45 = get_transform_matrix(0, 0, 0, -1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[4])
    T56 = get_transform_matrix(0, -0.121, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[5])
    
    TOOL_LENGTH_Z = 0.0 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]

class BaselineRecorderNode(Node):
    def __init__(self):
        super().__init__('baseline_recorder')
        self.is_recording = False
        self.trajectory_points = []
        
        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.get_logger().info('📡 기준 데이터 수집 대기 중... (Plotly 3D HTML 저장 모드)')

    def state_callback(self, msg):
        if msg.data == True and not self.is_recording:
            self.get_logger().info('🟢 수집 시작')
            self.is_recording = True
            self.trajectory_points = []
        elif msg.data == False and self.is_recording:
            self.get_logger().info('🔴 수집 종료 및 저장 중...')
            self.is_recording = False
            self.save_baseline_with_image()

    def joint_callback(self, msg):
        if not self.is_recording: return
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            self.trajectory_points.append(forward_kinematics(q))
        except KeyError: pass

    def save_baseline_with_image(self):
        if not self.trajectory_points: return

        pts = pts[pts[:, 2] > 100.0]

        pts = np.array(self.trajectory_points)
        np.save('baseline.npy', pts)
        
        fig = go.Figure()
    
        # ==========================================================
        # 💡 [추가된 부분] 가상의 반구 표면(Mesh/Surface) 렌더링
        # ==========================================================
        C_X, C_Y, C_Z = 450.0, 0.0, 200.0  # 반구 중심점
        R = 50.0                           # 반구 반지름

        # 1. 촘촘한 그물망(Grid) 각도 데이터 생성
        u = np.linspace(0, 2 * np.pi, 50)   # 원둘레 360도 (50조각)
        v = np.linspace(0, np.pi / 2, 25)   # 꼭대기에서 바닥까지 90도 (25조각)
        u_grid, v_grid = np.meshgrid(u, v)

        # 2. 구면 좌표계를 이용해 X, Y, Z 좌표 계산
        surf_x = C_X + R * np.cos(u_grid) * np.sin(v_grid)
        surf_y = C_Y + R * np.sin(u_grid) * np.sin(v_grid)
        surf_z = C_Z + R * np.cos(v_grid)

        # 3. 곡면(Surface) 트레이스 추가
        fig.add_trace(go.Surface(
            x=surf_x, y=surf_y, z=surf_z,
            opacity=0.4,              # 💡 반투명도 (0.0 투명 ~ 1.0 불투명) - 선이 잘 보이도록 낮춤
            colorscale='Blues',       # 면 색상 (Blues, Ice, Viridis 등 선택 가능)
            showscale=False,          # 화면 옆에 뜨는 컬러바 막대기 숨김
            name='Target Surface',
            hoverinfo='skip'          # 마우스 올렸을 때 곡면 데이터는 안 뜨게 해서 궤적 분석에 방해 안되게 함
        ))
        # ==========================================================

        # 기존 궤적 그리기 (초록색 실선)
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='lines',
            line=dict(color='green', width=4),
            name='Baseline Path'
        ))
        
        # 비율 고정
        fig.update_layout(
            title='Recorded Baseline Trajectory (Interactive Dome)',
            scene=dict(aspectmode='data'),
            margin=dict(l=0, r=0, b=0, t=40)
        )
        
        # HTML 저장
        filename = f'baseline_dome_{datetime.datetime.now().strftime("%H%M%S")}.html'
        fig.write_html(os.path.join(IMAGE_DIR, filename))
        
        self.get_logger().info(f'✅ 데이터 및 3D 웹 뷰어 저장 완료: {filename}')
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