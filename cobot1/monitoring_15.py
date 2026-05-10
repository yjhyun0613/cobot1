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
import threading 

import firebase_admin
from firebase_admin import credentials, storage, db

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
    
    TOOL_LENGTH_Z = 0.2586 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    
    pos_meters = (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]
    return pos_meters * 1000.0

class ErrorMonitorNode(Node):
    def __init__(self):
        super().__init__('error_monitor_node')
        self.is_recording = False
        
        # 💡 [핵심 추가] 궤적 기록을 잠시 멈추는 일시정지 스위치 변수
        self.record_enabled = False 
        
        self.trajectory_points = []
        self.ERROR_THRESHOLD = 5.0 
        
        try:
            # 인증 파일 경로 확인 필수!
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-7f5d874f48.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app',
                    'storageBucket': 'rokey-d-2-4c32a.firebasestorage.app' 
                })
            self.get_logger().info('✅ Firebase 연결 성공')
        except Exception as e:
            self.get_logger().error(f'Firebase 연결 실패: {e}')

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        
        # 💡 [핵심 추가] 제어 노드에서 보내는 record_enable 신호를 구독합니다.
        self.enable_sub = self.create_subscription(Bool, '/dsr01/record_enable', self.enable_callback, 10)

        os.makedirs(IMAGE_DIR, exist_ok=True)

    # 💡 [핵심 추가] 스위치를 끄고 켜는 콜백 함수
    def enable_callback(self, msg):
        self.record_enabled = msg.data
        if msg.data:
            self.get_logger().info('▶️ 궤적 수집 일시정지 해제 (기록 중)')
        else:
            self.get_logger().info('⏸️ 궤적 수집 일시정지 (허공 이동 무시)')

    def state_callback(self, msg):
        if msg.data == True and not self.is_recording:
            self.get_logger().info('🟢 전체 수집 프로세스 시작')
            self.is_recording = True
            self.record_enabled = False # 초기에는 끈 상태로 대기
            self.trajectory_points = []
            
        elif msg.data == False and self.is_recording:
            self.get_logger().info('🔴 전체 수집 종료 (분석 및 리포트 생성 중...)')
            self.is_recording = False
            self.record_enabled = False
            threading.Thread(target=self.analyze_and_report, daemon=True).start()

    def joint_callback(self, msg):
        # 💡 [핵심 수정] 프로세스가 켜져 있어도, record_enabled가 False면 절대 기록하지 않습니다!
        if not self.is_recording or not self.record_enabled: 
            return
            
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            self.trajectory_points.append(forward_kinematics(q))
        except KeyError: pass

    def analyze_and_report(self):
        if not self.trajectory_points: 
            self.get_logger().error('🚨 수집된 유효 데이터가 없습니다. record_enable 신호가 정상인지 확인하세요.')
            return

        pts_full = np.array(self.trajectory_points)
        pts = pts_full[::5] 
        
        try:
            baseline_pts = np.load('baseline.npy')[::5] 
        except FileNotFoundError:
            self.get_logger().error('🚨 baseline.npy 파일이 없습니다. 먼저 기준을 기록하세요.')
            return

        self.get_logger().info('🧮 오차 연산 및 리포트 생성 중...')
        tri = Delaunay(baseline_pts[:, :2])
        mesh = trimesh.Trimesh(vertices=baseline_pts, faces=tri.simplices)

        closest_points, dists, triangle_ids = trimesh.proximity.closest_point(mesh, pts)
        
        error_mask = dists > self.ERROR_THRESHOLD
        rate = (np.sum(error_mask) / len(dists)) * 100
        max_err = np.max(dists)
        
        fig = go.Figure()

        # 기준 돔
        fig.add_trace(go.Mesh3d(
            x=baseline_pts[:, 0], y=baseline_pts[:, 1], z=baseline_pts[:, 2],
            i=tri.simplices[:, 0], j=tri.simplices[:, 1], k=tri.simplices[:, 2],
            opacity=0.3, color='cyan', name='Baseline Surface'
        ))

        # 💡 이제 '손잡이' 없이 스캔 궤적만 예쁘게 출력됩니다!
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='markers+lines', line=dict(color='rgba(0,0,0,0.1)', width=1),
            marker=dict(size=3, color=dists, colorscale='Jet', cmin=0, cmax=self.ERROR_THRESHOLD, colorbar=dict(title="Error (mm)")),
            name='Clean Scanned Path'
        ))

        fig.update_layout(
            title=f"FAIL Report (Error Rate: {rate:.1f}%) | Max Error: {max_err:.1f}mm", 
            scene=dict(aspectmode='data')
        )
        
        now_time = datetime.datetime.now()
        filename = f'FAIL_report_{now_time.strftime("%Y%m%d_%H%M%S")}.html'
        local_path = os.path.join(IMAGE_DIR, filename)
        fig.write_html(local_path, include_plotlyjs='cdn')

        try:
            bucket = storage.bucket()
            blob = bucket.blob(f'inspection_reports/{filename}')
            blob.upload_from_filename(local_path)
            blob.make_public()
            
            db.reference("robot/defect").push({
                "triggeredAt": int(now_time.timestamp() * 1000), 
                "date": now_time.strftime("%Y-%m-%d"),
                "time": now_time.strftime("%H:%M:%S"), 
                "errorRate": f"{rate:.1f}%",
                "errorRateNum": float(rate), 
                "storageUrl": blob.public_url
            })
            self.get_logger().info('💾 리포트 업로드 및 DB 저장 완료!')
        except Exception as e:
            self.get_logger().error(f'❌ 업로드 실패: {e}')
        finally:
            self.get_logger().info('✅ [모니터링 완료] 다음 신호를 대기합니다.')

def main(args=None):
    rclpy.init(args=args)
    node = ErrorMonitorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()