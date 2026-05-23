import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import numpy as np
import plotly.graph_objects as go
import os
import datetime
import threading
import time

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

class PredictiveMonitorNode(Node):
    def __init__(self):
        super().__init__('predictive_monitor_node')
        self.is_recording = False
        self.record_enabled = False 
        
        # 💡 [예측 알고리즘 핵심 변수]
        self.history_window = []     # 최근 5개의 점을 기억할 윈도우
        self.trajectory_points = []  # 전체 스캔 좌표
        self.errors = []             # 각 좌표의 예측 오차(mm)
        
        # 💡 사용자 설정 값 적용
        self.WINDOW_SIZE = 5         # 최근 5개 점으로 예측
        self.ERROR_THRESHOLD = 3.0   # 허용 오차 3mm
        self.FAIL_RATE_LIMIT = 5.0   # 전체의 5% 이상 오차가 나면 불량 판정
        
        os.makedirs(IMAGE_DIR, exist_ok=True)

        try:
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
        self.enable_sub = self.create_subscription(Bool, '/dsr01/record_enable', self.enable_callback, 10)
        
        self.result_pub = self.create_publisher(String, '/dsr01/inspection_result', 10)
        self.get_logger().info('📡 [예측 기반 검사 모드] 실행 중 (Baseline 의존성 제거됨)')

    def enable_callback(self, msg):
        self.record_enabled = msg.data
        # 💡 층이 바뀌거나 허공 이동 시 윈도우(기억)를 초기화합니다.
        if not msg.data:
            self.history_window = []

    def state_callback(self, msg):
        if msg.data == True and not self.is_recording:
            self.get_logger().info('🟢 전체 수집 시작')
            self.is_recording = True
            self.record_enabled = False 
            self.trajectory_points = []
            self.errors = []
            self.history_window = []
            
        elif msg.data == False and self.is_recording:
            self.get_logger().info('🔴 수집 종료 (오차 분석 및 리포트 생성 중...)')
            self.is_recording = False
            self.record_enabled = False
            threading.Thread(target=self.analyze_and_report, daemon=True).start()

    def joint_callback(self, msg):
        if not self.is_recording or not self.record_enabled: 
            return
            
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            
            # 💡 [예측 알고리즘 적용]
            if len(self.history_window) < self.WINDOW_SIZE:
                # 1~5번째 점: 예측 불가이므로 오차 0.0으로 통과 (Cold Start)
                self.history_window.append(current_pos)
                self.trajectory_points.append(current_pos)
                self.errors.append(0.0)
            else:
                # 6번째 점부터: 이전 5개 점으로 다음 위치 예측 (2차 다항식 곡선 피팅)
                t_vals = np.arange(self.WINDOW_SIZE)
                xs = [p[0] for p in self.history_window]
                ys = [p[1] for p in self.history_window]
                zs = [p[2] for p in self.history_window]
                
                # t=5 일 때의 x, y, z 예측 (t_vals의 마지막이 4이므로 다음은 5)
                pred_x = np.polyval(np.polyfit(t_vals, xs, 2), self.WINDOW_SIZE)
                pred_y = np.polyval(np.polyfit(t_vals, ys, 2), self.WINDOW_SIZE)
                pred_z = np.polyval(np.polyfit(t_vals, zs, 2), self.WINDOW_SIZE)
                pred_pos = np.array([pred_x, pred_y, pred_z])
                
                # 실제 도착한 좌표와 예상 좌표 간의 직선 거리(오차) 계산
                err_dist = np.linalg.norm(current_pos - pred_pos)
                
                # 윈도우 슬라이딩 (제일 오래된 것 버리고 현재 것 추가)
                self.history_window.pop(0)
                self.history_window.append(current_pos)
                
                self.trajectory_points.append(current_pos)
                self.errors.append(err_dist)
                
        except Exception as e: pass

    def analyze_and_report(self):
        if not self.trajectory_points: 
            self.get_logger().error('🚨 수집된 데이터가 없습니다!')
            return

        total_pts = len(self.errors)
        err_array = np.array(self.errors)
        
        max_err = np.max(err_array)
        fail_rate = (np.sum(err_array > self.ERROR_THRESHOLD) / total_pts) * 100
        result = "PASS" if fail_rate <= self.FAIL_RATE_LIMIT else "FAIL"
        
        res_msg = String()
        res_msg.data = result
        self.result_pub.publish(res_msg)

        status_text = "❌ FAIL" if result == "FAIL" else "✅ PASS"
        self.get_logger().info('--------------------------------------------------')
        self.get_logger().info(f'📊 [예측 기반] 분석 결과: {status_text}')
        self.get_logger().info(f'   - 불량률: {fail_rate:.1f}% (허용 한도: {self.FAIL_RATE_LIMIT}%)')
        self.get_logger().info(f'   - 최대 궤적 이탈 오차: {max_err:.1f}mm')
        self.get_logger().info('--------------------------------------------------')

        if result == "FAIL":
            self.save_and_upload_report(result, fail_rate, max_err)
        else:
            self.get_logger().info('✅ 스캔 경로가 매우 매끄럽습니다! 리포트 업로드를 생략합니다.')

    def save_and_upload_report(self, result, rate, max_err):
        self.get_logger().info('🌐 3D 리포트 생성 및 Firebase 업로드 중...')
        
        pts = np.array(self.trajectory_points)
        errs = np.array(self.errors)
        fig = go.Figure()

        # 💡 Baseline(기준 메쉬)이 없으므로, 오직 로봇이 그린 궤적의 '부드러움'만 시각화합니다.
        # 오차가 3mm를 넘어가는 튀는 구간은 빨간색으로 강렬하게 표시됩니다.
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='markers+lines', line=dict(color='rgba(0,0,0,0.2)', width=2),
            marker=dict(
                size=4, 
                color=errs, 
                colorscale='Jet', 
                cmin=0, 
                cmax=self.ERROR_THRESHOLD, 
                colorbar=dict(title="궤적 이탈 (mm)")
            ),
            name='Scanned Path & Jitter'
        ))

        fig.update_layout(
            title=f"Predictive Inspection FAIL Report | Error Rate: {rate:.1f}% | Max Jitter: {max_err:.1f}mm", 
            scene=dict(aspectmode='data')
        )
        
        now_time = datetime.datetime.now()
        filename = f'predict_FAIL_{now_time.strftime("%Y%m%d_%H%M%S")}.html'
        local_path = os.path.join(IMAGE_DIR, filename)
        fig.write_html(local_path, include_plotlyjs='cdn')

        try:
            bucket = storage.bucket()
            blob = bucket.blob(f'inspection_reports/{filename}')
            blob.upload_from_filename(local_path)
            blob.make_public()
            db.reference("robot/defect").push({
                "triggeredAt": int(now_time.timestamp() * 1000), "date": now_time.strftime("%Y-%m-%d"),
                "time": now_time.strftime("%H:%M:%S"), "errorRate": f"{rate:.1f}%",
                "errorRateNum": float(rate), "storageUrl": blob.public_url,
                "type": "predictive" # 예측 기반 검사임을 명시
            })
            self.get_logger().info(f'💾 리포트 업로드 완료! 주소: {blob.public_url}')
        except Exception as e: 
            self.get_logger().error(f'❌ 업로드 실패: {e}')
        finally:
            self.get_logger().info('✅ 다음 스캔을 대기합니다.')

def main(args=None):
    rclpy.init(args=args)
    node = PredictiveMonitorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()