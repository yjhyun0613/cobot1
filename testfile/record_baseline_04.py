import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import plotly.graph_objects as go
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
    
    # 미터(m) 단위 결과를 1000 곱해서 밀리미터(mm)로 변환
    pos_meters = (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]
    return pos_meters * 1000.0

class PCTestRecorderNode(Node):
    def __init__(self):
        super().__init__('pc_test_recorder')
        self.is_recording = False
        self.trajectory_points = []
        
        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.get_logger().info('📡 PC 테스트용 기록 대기 중... (npy + html 저장)')

    def state_callback(self, msg):
        if msg.data == True and not self.is_recording:
            self.get_logger().info('🟢 기록 시작 (좌표 수집 중...)')
            self.is_recording = True
            self.trajectory_points = []
        elif msg.data == False and self.is_recording:
            self.get_logger().info('🔴 기록 종료. 데이터 정제 및 파일 저장 시작!')
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

        # 1. 넘파이 배열로 만든 후 바닥(Z < 100)에 찍힌 유령 점(0,0,0) 제거
        pts = np.array(self.trajectory_points)
        pts = pts[pts[:, 2] > 100.0] 

        # 2. npy 파일 저장
        np.save('baseline.npy', pts)
        self.get_logger().info(f'✅ npy 저장 완료 (데이터 수: {len(pts)})')

        # 3. Plotly 3D HTML 저장 (가짜 반구 없이 오직 실제 데이터만으로 렌더링)
        fig = go.Figure()

        # [실제 데이터로 만든 3D 면(Mesh)]
        # 로봇이 훑고 간 점들을 이어붙여 실제 스캐닝 영역을 반투명한 면으로 보여줍니다.
        fig.add_trace(go.Mesh3d(
            x=pts[:, 0], 
            y=pts[:, 1], 
            z=pts[:, 2],
            opacity=0.3,           
            color='lightblue',    
            name='Real Scanned Mesh'
        ))

        # [실제 궤적 선(Line)]
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], 
            y=pts[:, 1], 
            z=pts[:, 2],
            mode='lines',
            line=dict(color='blue', width=4),
            name='Trajectory Path'
        ))
        
        # 비율 1:1:1 고정
        fig.update_layout(
            title='Real Scanned Trajectory & Mesh (mm)',
            scene=dict(aspectmode='data'),
            margin=dict(l=0, r=0, b=0, t=40)
        )
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'pc_test_{timestamp}.html'
        fig.write_html(os.path.join(IMAGE_DIR, filename))
        
        self.get_logger().info(f'✅ HTML 3D 웹 뷰어 저장 완료: {filename}')

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