import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import numpy as np
from scipy.spatial import Delaunay  # 💡 면 생성을 위한 삼각분할 알고리즘
import trimesh                      # 💡 3D 면(Mesh) 처리 및 거리 계산 라이브러리
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
    
    # 💡 그리퍼(228.6mm) + 툴(30mm) = 263.6mm (0.2586m) 반영
    TOOL_LENGTH_Z = 0.2586 
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

        # 점(Point) 데이터를 불러와서 가상의 3D 면(Mesh)으로 변환
        try:
            self.baseline_points = np.load('/home/yoon/cobot_ws/src/cobot1/cobot1/baseline.npy')
            
            # X, Y 좌표를 이용해 점들을 삼각형으로 엮어줍니다 (Delaunay 삼각분할)
            tri = Delaunay(self.baseline_points[:, :2])
            
            # 만들어진 삼각형 연결 정보(Faces)와 기존 좌표(Vertices)를 이용해 Trimesh 객체 생성
            self.baseline_mesh = trimesh.Trimesh(vertices=self.baseline_points, faces=tri.simplices)
            
            self.get_logger().info(f'✅ 기준 데이터({len(self.baseline_points)}개)를 3D Mesh 표면으로 변환 완료')
        except FileNotFoundError:
            self.get_logger().error('❌ baseline.npy 파일이 없습니다.')
            raise SystemExit

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.result_pub = self.create_publisher(String, '/dsr01/inspection_result', 10)
        
        self.get_logger().info('📡 표면(Surface) 투영 방식 검사 모드 실행 중')

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
            
            # 점과 가장 가까운 "표면(Mesh Face)"까지의 최단 직교 거리 계산
            closest_points, distances, triangle_ids = self.baseline_mesh.nearest.on_surface([current_pos])
            dist = distances[0] # 첫 번째 점(current_pos)의 거리값
            
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

        # 기준 돔 형상을 점(Point)이 아닌 매끄러운 3D 껍질(Mesh)로 렌더링
        fig.add_trace(go.Mesh3d(
            x=self.baseline_mesh.vertices[:, 0], 
            y=self.baseline_mesh.vertices[:, 1], 
            z=self.baseline_mesh.vertices[:, 2],
            i=self.baseline_mesh.faces[:, 0], 
            j=self.baseline_mesh.faces[:, 1], 
            k=self.baseline_mesh.faces[:, 2], 
            color='lightgray',
            opacity=0.3, 
            name='Baseline Surface'
        ))

        # 로봇이 실제 이동한 궤적의 '불량 히트맵'을 점(Scatter3d)으로 덧그림
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], 
            y=pts[:, 1], 
            z=pts[:, 2],
            mode='markers+lines',
            line=dict(color='rgba(0,0,0,0.1)', width=1),
            marker=dict(
                size=3,
                color=dists,
                colorscale='Jet',
                cmin=0,
                cmax=self.ERROR_THRESHOLD,
                colorbar=dict(title="오차 (mm)", thickness=20)
            ),
            name='Scanned Path Error'
        ))

        fig.update_layout(
            title=f"FAIL Report (Error Rate: {rate:.1f}%) | Max Surface Error: {max_err:.1f}mm",
            scene=dict(aspectmode='data'),
            margin=dict(l=0, r=0, b=0, t=80)
        )

        # 💡 리포트 파일명에 날짜와 시간을 모두 포함하도록 수정 (예: FAIL_report_20260510_102200.html)
        now_time = datetime.datetime.now()
        filename = f'FAIL_report_{now_time.strftime("%Y%m%d_%H%M%S")}.html'
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
            
            db.reference("robot/defect").push({
                "triggeredAt": int(now_time.timestamp() * 1000),
                "date": now_time.strftime("%Y-%m-%d"),
                "time": now_time.strftime("%H:%M:%S"),
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