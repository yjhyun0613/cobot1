import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import numpy as np
import plotly.graph_objects as go
import os
import datetime
import threading
import time  # 💡 [추가] 실시간 생중계 쿨타임(0.1초) 계산용

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
        
        self.history_window = []     
        self.trajectory_points = []  
        self.errors = []             
        
        self.WINDOW_SIZE = 5         
        self.ERROR_THRESHOLD = 3.0   
        self.FAIL_RATE_LIMIT = 5.0   
        
        self.last_fb_time = 0.0      # 💡 [추가] 파이어베이스 전송 쿨타임 변수
        
        os.makedirs(IMAGE_DIR, exist_ok=True)

        try:
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-7f5d874f48.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app',
                    'storageBucket': 'rokey-d-2-4c32a.firebasestorage.app' 
                })
            self.get_logger().info('✅ Firebase 연결 성공 (실시간 생중계 준비 완료)')
        except Exception as e:
            self.get_logger().error(f'Firebase 연결 실패: {e}')

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.enable_sub = self.create_subscription(Bool, '/dsr01/record_enable', self.enable_callback, 10)
        
        self.result_pub = self.create_publisher(String, '/dsr01/inspection_result', 10)
        self.get_logger().info('📡 [예측 기반 검사 + 웹 대시보드 중계 모드] 실행 중')

    def enable_callback(self, msg):
        self.record_enabled = msg.data
        if not msg.data:
            self.history_window = []

    def state_callback(self, msg):
        if msg.data == True and not self.is_recording:
            self.get_logger().info('🟢 전체 수집 시작 (웹 대시보드 초기화)')
            self.is_recording = True
            self.record_enabled = False 
            self.trajectory_points = []
            self.errors = []
            self.history_window = []
            
            # 💡 [추가] 웹 브라우저 대시보드 화면을 싹 지우고 새로 그릴 준비를 하라고 명령
            db.reference('robot/dsr01/status').set({'is_scanning': True, 'timestamp': time.time()})
            
        elif msg.data == False and self.is_recording:
            self.get_logger().info('🔴 수집 종료 (오차 분석 및 리포트 생성 중...)')
            self.is_recording = False
            self.record_enabled = False
            
            # 💡 [추가] 스캔이 끝났음을 웹에 알림
            db.reference('robot/dsr01/status').set({'is_scanning': False, 'timestamp': time.time()})
            
            threading.Thread(target=self.analyze_and_report, daemon=True).start()

    def joint_callback(self, msg):
        if not self.is_recording or not self.record_enabled: 
            return
            
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            
            err_dist = 0.0 # 웜업 구간(첫 5개 점)일 때 기본 오차값은 0.0으로 둡니다.

            # [예측 알고리즘 연산]
            if len(self.history_window) < self.WINDOW_SIZE:
                self.history_window.append(current_pos)
                self.trajectory_points.append(current_pos)
                self.errors.append(0.0)
            else:
                t_vals = np.arange(self.WINDOW_SIZE)
                xs = [p[0] for p in self.history_window]
                ys = [p[1] for p in self.history_window]
                zs = [p[2] for p in self.history_window]
                
                pred_x = np.polyval(np.polyfit(t_vals, xs, 2), self.WINDOW_SIZE)
                pred_y = np.polyval(np.polyfit(t_vals, ys, 2), self.WINDOW_SIZE)
                pred_z = np.polyval(np.polyfit(t_vals, zs, 2), self.WINDOW_SIZE)
                pred_pos = np.array([pred_x, pred_y, pred_z])
                
                err_dist = np.linalg.norm(current_pos - pred_pos)
                
                self.history_window.pop(0)
                self.history_window.append(current_pos)
                
                self.trajectory_points.append(current_pos)
                self.errors.append(err_dist)
            
            # 💡 [핵심 추가] 계산된 오차(err_dist)와 좌표를 0.1초마다 파이어베이스에 업로드!
            current_time = time.time()
            if current_time - self.last_fb_time > 0.02:
                db.reference('robot/dsr01/current_pose').set({
                    'x': float(current_pos[0]),
                    'y': float(current_pos[1]),
                    'z': float(current_pos[2]),
                    'error': float(err_dist)  # 웹 오실로스코프 그래프로 전달될 오차값!
                })
                self.last_fb_time = current_time
                
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
                "type": "predictive" 
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