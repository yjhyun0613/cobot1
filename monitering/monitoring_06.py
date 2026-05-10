import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
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
    T01 = get_transform_matrix(0, 0, 0.1345, 0, 0, q[0])
    T12 = get_transform_matrix(0, 0.0062, 0, 0, -1.571, -1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[1])
    T23 = get_transform_matrix(0.411, 0, 0, 0, 0, 1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[2])
    T34 = get_transform_matrix(0, -0.368, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[3])
    T45 = get_transform_matrix(0, 0, 0, -1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[4])
    T56 = get_transform_matrix(0, -0.121, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[5])
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56)[:3, 3] * 1000.0

class TrajectoryMonitorNode(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.is_checking = False
        
        # 데이터 저장용 리스트
        self.trajectory_points = []
        self.distances = []
        
        self.ERROR_THRESHOLD = 5.0  # 5mm 기준
        self.FAIL_RATE_LIMIT = 10.0 # 10% 기준
        
        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)

        try:
            self.baseline_points = np.load('/home/yoon/cobot_ws/src/cobot1/cobot1/baseline.npy')
            self.kdtree = cKDTree(self.baseline_points) 
            self.get_logger().info(f'✅ 기준 데이터 로드 완료 ({len(self.baseline_points)} pts)')
        except FileNotFoundError:
            self.get_logger().error('❌ baseline.npy 파일이 없습니다.')
            raise SystemExit

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.result_pub = self.create_publisher(String, '/inspection_result', 10)
        
        self.get_logger().info('📡 종합 분석 모드 준비 완료 (스캔 종료 후 판정)')

    def state_callback(self, msg):
        if msg.data == True and not self.is_checking:
            self.get_logger().info('🟢 검사 데이터 수집 시작...')
            self.is_checking = True
            self.trajectory_points = []
            self.distances = []
            
        elif msg.data == False and self.is_checking:
            self.is_checking = False
            self.get_logger().info('🔴 스캐닝 종료. 데이터 분석 중...')
            self.perform_final_inspection()

    def joint_callback(self, msg):
        if not self.is_checking: return
        
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            
            if current_pos[2] < 1.0: return # 유령 데이터 필터링
            
            # 실시간 오차 계산 및 저장
            dist, _ = self.kdtree.query(current_pos)
            self.trajectory_points.append(current_pos)
            self.distances.append(dist)
                
        except Exception as e:
            self.get_logger().error(f'계산 에러: {e}')

    def perform_final_inspection(self):
        """스캔 종료 후 전체 데이터를 분석하여 합격/불량을 판정합니다."""
        if not self.distances:
            self.get_logger().warn('⚠️ 수집된 데이터가 없습니다.')
            return

        total_pts = len(self.distances)
        dist_array = np.array(self.distances)
        
        # 불량 포인트(5mm 초과) 계산
        fail_pts = np.sum(dist_array > self.ERROR_THRESHOLD)
        fail_rate = (fail_pts / total_pts) * 100
        max_err = np.max(dist_array)
        mean_err = np.mean(dist_array)

        # 판정 로직
        result = "PASS" if fail_rate <= self.FAIL_RATE_LIMIT else "FAIL"
        
        # 결과 메시지 발행
        res_msg = String()
        res_msg.data = f"{result} (Rate: {fail_rate:.1f}%)"
        self.result_pub.publish(res_msg)

        self.get_logger().info(f'📊 [최종 판정: {result}]')
        self.get_logger().info(f'   - 불량률: {fail_rate:.1f}% (기준: {self.FAIL_RATE_LIMIT}%)')
        self.get_logger().info(f'   - 최대 오차: {max_err:.1f}mm / 평균 오차: {mean_err:.1f}mm')

        # 분석 리포트(HTML) 생성
        self.save_final_report(result, fail_rate, max_err, mean_err)

    def save_final_report(self, result, rate, max_err, mean_err):
        pts = np.array(self.trajectory_points)
        dists = np.array(self.distances)
        
        fig = go.Figure()

        # 1. 기준 궤적 (회색 점선)
        fig.add_trace(go.Scatter3d(
            x=self.baseline_points[:, 0], y=self.baseline_points[:, 1], z=self.baseline_points[:, 2],
            mode='lines', line=dict(color='rgba(150,150,150,0.5)', width=2, dash='dot'),
            name='Baseline'
        ))

        # 2. 실제 궤적 (오차에 따른 컬러 맵핑)
        # 오차가 0에 가까우면 Blue, 5mm 이상이면 Red
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='markers', 
            marker=dict(
                size=3,
                color=dists,
                colorscale='Jet', # Blue-Cyan-Yellow-Red 순서
                cmin=0, cmax=self.ERROR_THRESHOLD,
                colorbar=dict(title="Error (mm)"),
                showscale=True
            ),
            name='Actual Path (Color-coded)'
        ))

        # 리포트 레이아웃 설정
        res_color = "green" if result == "PASS" else "red"
        fig.update_layout(
            title=f"Inspection Report: <span style='color:{res_color}'>{result}</span><br>" +
                  f"Outlier Rate: {rate:.1f}% | Max Error: {max_err:.1f}mm | Mean Error: {mean_err:.1f}mm",
            scene=dict(
                aspectmode='cube', # 💡 비율 1:1:1 고정
                xaxis_title='X (mm)', yaxis_title='Y (mm)', zaxis_title='Z (mm)'
            ),
            margin=dict(l=0, r=0, b=0, t=80)
        )

        filename = f'report_{result}_{datetime.datetime.now().strftime("%m%d_%H%M")}.html'
        fig.write_html(os.path.join(IMAGE_DIR, filename))
        self.get_logger().info(f'📸 종합 리포트 저장 완료: {filename}')

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