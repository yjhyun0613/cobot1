import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import numpy as np
from scipy.spatial import cKDTree
import plotly.graph_objects as go
import os
import datetime

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
    # DSR-M0609 DH Parameters (m 단위)
    T01 = get_transform_matrix(0, 0, 0.1345, 0, 0, q[0])
    T12 = get_transform_matrix(0, 0.0062, 0, 0, -1.571, -1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[1])
    T23 = get_transform_matrix(0.411, 0, 0, 0, 0, 1.571) @ get_transform_matrix(0, 0, 0, 0, 0, q[2])
    T34 = get_transform_matrix(0, -0.368, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[3])
    T45 = get_transform_matrix(0, 0, 0, -1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[4])
    T56 = get_transform_matrix(0, -0.121, 0, 1.571, 0, 0) @ get_transform_matrix(0, 0, 0, 0, 0, q[5])
    
    # 💡 그리퍼(228.6mm) + 툴(35mm) = 263.6mm (0.2636m) 반영
    TOOL_LENGTH_Z = 0.2636 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    
    # 최종 TCP 위치 계산 후 mm 단위 변환
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3] * 1000.0

class TrajectoryMonitorNode(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.is_checking = False
        self.trajectory_points = []
        self.distances = []
        
        self.ERROR_THRESHOLD = 5.0  
        self.FAIL_RATE_LIMIT = 10.0 
        
        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)

        # 파이어베이스 초기화
        try:
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'storageBucket': 'rokey-d-2-4c32a.firebasestorage.app',
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app/'
                })
            self.get_logger().info('☁️ Firebase Storage 및 RTDB 연동 완료')
        except Exception as e:
            self.get_logger().error(f'Firebase 초기화 오류: {e}')

        try:
            self.baseline_points = np.load('/home/yoon/cobot_ws/src/cobot1/cobot1/baseline.npy')
            self.kdtree = cKDTree(self.baseline_points) 
            self.get_logger().info(f'✅ 기준 데이터 로드 완료')
        except FileNotFoundError:
            self.get_logger().error('❌ baseline.npy 파일이 없습니다.')
            raise SystemExit

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.result_pub = self.create_publisher(String, '/dsr01/inspection_result', 10)
        
        self.get_logger().info('📡 툴 끝단 기준 분석 모드 실행 중 (FAIL 발생 시 클라우드 업로드)')

    def state_callback(self, msg):
        if msg.data == True and not self.is_checking:
            self.get_logger().info('🟢 데이터 수집 시작')
            self.is_checking = True
            self.trajectory_points = []
            self.distances = []
        elif msg.data == False and self.is_checking:
            self.is_checking = False
            self.get_logger().info('🔴 분석 시작...')
            self.perform_final_inspection()

    def joint_callback(self, msg):
        if not self.is_checking: return
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            
            # 💡 필터링 제거 (모든 구간 데이터 수집)
            dist, _ = self.kdtree.query(current_pos)
            self.trajectory_points.append(current_pos)
            self.distances.append(dist)
        except Exception as e:
            self.get_logger().error(f'에러: {e}')

    def perform_final_inspection(self):
        if not self.distances: return
        
        total_pts = len(self.distances)
        dist_array = np.array(self.distances)
        fail_pts = np.sum(dist_array > self.ERROR_THRESHOLD)
        fail_rate = (fail_pts / total_pts) * 100
        max_err = np.max(dist_array)
        mean_err = np.mean(dist_array)
        
        result = "PASS" if fail_rate <= self.FAIL_RATE_LIMIT else "FAIL"
        
        res_msg = String()
        res_msg.data = result
        self.result_pub.publish(res_msg)

        if result == "FAIL":
            self.get_logger().warn(f'❌ 불량 감지({fail_rate:.1f}%)! 리포트 생성 및 DB 업로드를 진행합니다.')
            self.save_and_upload_report(result, fail_rate, max_err, mean_err)
        else:
            self.get_logger().info(f'✅ 합격({fail_rate:.1f}%)! 불필요한 기록 생성을 건너뜁니다.')

    def save_and_upload_report(self, result, rate, max_err, mean_err):
        pts = np.array(self.trajectory_points)
        dists = np.array(self.distances)
        fig = go.Figure()

        # 히트맵 메쉬 생성
        fig.add_trace(go.Mesh3d(
            x=pts[:, 0], 
            y=pts[:, 1], 
            z=pts[:, 2],
            intensity=dists,
            colorscale='Jet',
            cmin=0,
            cmax=self.ERROR_THRESHOLD,
            opacity=0.4,
            alphahull=-1,
            name='Error Heatmap Surface',
            colorbar=dict(title="오차 (mm)", thickness=20)
        ))

        # 기준 궤적 (점선)
        fig.add_trace(go.Scatter3d(
            x=self.baseline_points[:, 0], y=self.baseline_points[:, 1], z=self.baseline_points[:, 2],
            mode='lines', line=dict(color='rgba(150,150,150,0.8)', width=3, dash='dot'), name='Baseline'
        ))

        # 실제 궤적 라인
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='lines', line=dict(color='rgba(0,0,0,0.3)', width=1), name='Actual Path'
        ))

        fig.update_layout(
            title=f"FAIL Report (Error Rate: {rate:.1f}%) | Max Error: {max_err:.1f}mm",
            scene=dict(aspectmode='data'),
            margin=dict(l=0, r=0, b=0, t=80)
        )

        filename = f'FAIL_report_{datetime.datetime.now().strftime("%H%M%S")}.html'
        local_path = os.path.join(IMAGE_DIR, filename)
        fig.write_html(local_path)

        # Storage 업로드 및 RTDB 기록 연동
        try:
            bucket = storage.bucket()
            blob = bucket.blob(f'inspection_reports/{filename}')
            blob.upload_from_filename(local_path)
            blob.make_public()
            public_url = blob.public_url
            self.get_logger().info(f'🔗 리포트 주소: {public_url}')
            
            now = datetime.datetime.now()
            db.reference("robot/defect").push({
                "triggeredAt": int(now.timestamp() * 1000),
                "date": now.strftime("%Y-%m-%d"),
                "time": now.strftime("%H:%M:%S"),
                "errorRate": f"{rate:.1f}%",
                "errorRateNum": float(rate),
                "storageUrl": public_url
            })
            self.get_logger().info('💾 실시간 데이터베이스(RTDB)에 불량 이력 저장 완료!')
            
        except Exception as e:
            self.get_logger().error(f'❌ 업로드 또는 DB 저장 실패: {e}')

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