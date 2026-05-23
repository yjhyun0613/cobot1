import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import numpy as np
from scipy.spatial import Delaunay
import trimesh
import plotly.graph_objects as go
import os
import datetime

import firebase_admin
from firebase_admin import credentials, storage, db

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
    
    TOOL_LENGTH_Z = 0.2586 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3] * 1000.0

class TrajectoryMonitorNode(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.is_checking = False
        
        # 💡 [핵심 추가] 궤적 기록을 잠시 멈추는 일시정지 스위치 변수
        self.record_enable = False 
        
        self.trajectory_points = []
        self.distances = []
        
        self.ERROR_THRESHOLD = 2.0  
        self.FAIL_RATE_LIMIT = 5.0 
        
        if not os.path.exists(IMAGE_DIR): os.makedirs(IMAGE_DIR)

        try:
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-7f5d874f48.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'storageBucket': 'rokey-d-2-4c32a.firebasestorage.app',
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app/'
                })
            self.get_logger().info('☁️ Firebase 연동 완료')
        except Exception as e:
            self.get_logger().error(f'Firebase 오류: {e}')

        try:
            self.baseline_points = np.load('/home/yoon/cobot_ws/src/cobot1/cobot1/baseline.npy')
            tri = Delaunay(self.baseline_points[:, :2])
            self.baseline_mesh = trimesh.Trimesh(vertices=self.baseline_points, faces=tri.simplices)
        except FileNotFoundError:
            self.get_logger().error('❌ baseline.npy 없음')
            raise SystemExit

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        
        # 💡 [핵심 추가] 로봇 제어기에서 쏴주는 '일시정지' 신호를 받는 구독자 생성
        self.enable_sub = self.create_subscription(Bool, '/dsr01/record_enable', self.enable_callback, 10)
        
        self.result_pub = self.create_publisher(String, '/dsr01/inspection_result', 10)
        self.get_logger().info('📡 스위치 기반 기록 모드 (일시정지 기능 활성화)')

    # 💡 [핵심 추가] 스위치를 끄고 켜는 콜백 함수
    def enable_callback(self, msg):
        self.record_enable = msg.data

    def state_callback(self, msg):
        if msg.data == True and not self.is_checking:
            self.is_checking = True
            self.record_enable = False # 초기에는 끈 상태로 시작
            self.trajectory_points = []
            self.distances = []
        elif msg.data == False and self.is_checking:
            self.is_checking = False
            self.record_enable = False
            self.get_logger().info('🔴 분석 시작...')
            self.perform_final_inspection()

    def joint_callback(self, msg):
        # 💡 [핵심 변경] 전체 상태(is_checking)가 켜져 있어도, record_enable이 False면 기록 안 함!
        if not self.is_checking or not self.record_enable: 
            return
            
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            
            closest_points, distances, triangle_ids = self.baseline_mesh.nearest.on_surface([current_pos])
            self.trajectory_points.append(current_pos)
            self.distances.append(distances[0])
        except Exception as e: pass

    def perform_final_inspection(self):
        if not self.distances: return
        total_pts = len(self.distances)
        dist_array = np.array(self.distances)
        fail_rate = (np.sum(dist_array > self.ERROR_THRESHOLD) / total_pts) * 100
        result = "PASS" if fail_rate <= self.FAIL_RATE_LIMIT else "FAIL"
        
        res_msg = String()
        res_msg.data = result
        self.result_pub.publish(res_msg)

        if result == "FAIL":
            self.save_and_upload_report(result, fail_rate, np.max(dist_array))
        else:
            self.get_logger().info(f'✅ 합격({fail_rate:.1f}%)')

    def save_and_upload_report(self, result, rate, max_err):
        pts = np.array(self.trajectory_points)
        dists = np.array(self.distances)
        fig = go.Figure()

        fig.add_trace(go.Mesh3d(
            x=self.baseline_mesh.vertices[:, 0], y=self.baseline_mesh.vertices[:, 1], z=self.baseline_mesh.vertices[:, 2],
            i=self.baseline_mesh.faces[:, 0], j=self.baseline_mesh.faces[:, 1], k=self.baseline_mesh.faces[:, 2], 
            color='lightgray', opacity=0.3, name='Baseline Surface'
        ))

        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='markers+lines', line=dict(color='rgba(0,0,0,0.1)', width=1),
            marker=dict(size=3, color=dists, colorscale='Jet', cmin=0, cmax=self.ERROR_THRESHOLD, colorbar=dict(title="오차(mm)")),
            name='Scanned Path'
        ))

        fig.update_layout(title=f"FAIL Report (Error Rate: {rate:.1f}%) | Max Error: {max_err:.1f}mm", scene=dict(aspectmode='data'))
        
        now_time = datetime.datetime.now()
        filename = f'FAIL_report_{now_time.strftime("%Y%m%d_%H%M%S")}.html'
        local_path = os.path.join(IMAGE_DIR, filename)
        fig.write_html(local_path)

        try:
            bucket = storage.bucket()
            blob = bucket.blob(f'inspection_reports/{filename}')
            blob.upload_from_filename(local_path)
            blob.make_public()
            db.reference("robot/defect").push({
                "triggeredAt": int(now_time.timestamp() * 1000), "date": now_time.strftime("%Y-%m-%d"),
                "time": now_time.strftime("%H:%M:%S"), "errorRate": f"{rate:.1f}%",
                "errorRateNum": float(rate), "storageUrl": blob.public_url
            })
        except Exception as e: self.get_logger().error(f'업로드 실패: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMonitorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()