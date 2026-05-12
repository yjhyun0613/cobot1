import rclpy
from rclpy.node import Node
import threading
import queue
import time
import datetime
import json
import numpy as np
from scipy.interpolate import CubicSpline

# Firebase
import firebase_admin
from firebase_admin import credentials, db, storage

# ROS 2
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import GetExternalTorque

# =========================================================
# 🧮 순운동학 (Forward Kinematics) 
# =========================================================
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
    T_tool = get_transform_matrix(0, 0, 0.2586, 0, 0, 0)
    pos_meters = (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]
    return pos_meters * 1000.0 

# =========================================================
# ⚙️ 설정 파라미터
# =========================================================
RADIUS = 48.5
SPIKE_THRESHOLD = 1.7
ERROR_LIMIT_PERCENT = 5.0  # (현실적 기준) 5% 오차율 제한
MARGIN_FRAMES = 15         # 데이터 분석 구간 (조금 좁힘)
WINDOW_FRAMES = 10         

# 💡 [추가] 궤적 진입 극초반에 발생하는 노이즈를 무시할 프레임 수
# 20Hz 기준 20프레임 = 1.0초 무시
IGNORE_START_FRAMES = 20 

class InspectionMasterNode(Node):
    def __init__(self):
        super().__init__('inspection_master_node')
        
        try:
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-7f5d874f48.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app',
                    'storageBucket': 'rokey-d-2-4c32a.firebasestorage.app'
                })
            self.stream_ref = db.reference('robot/dsr01/stream')
            self.bucket = storage.bucket()
            self.get_logger().info('✅ Firebase 연결 완료! (초반 진입 노이즈 무시 적용)')
        except: pass

        self.fb_queue = queue.Queue()
        self.fb_worker = threading.Thread(target=self.firebase_push_worker, daemon=True)
        self.fb_worker.start()

        self.is_overall_running = False
        self.is_recording = False
        self.latest_pos = None
        self.scan_buffer = []
        self.max_error_rate = 0.0
        self.max_e_max = 0.0
        
        self.torque_buffer = []
        self.crack_active = False
        self.crack_start_idx = 0
        self.post_count = 0
        
        # 💡 [추가] 초기 무시 카운터 변수
        self.ignore_count = 0

        self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.tq_cli = self.create_client(GetExternalTorque, '/dsr01/aux_control/get_external_torque')
        self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.create_subscription(Bool, '/dsr01/record_enable', self.enable_callback, 10)
        self.result_pub = self.create_publisher(String, '/dsr01/inspection_result', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def firebase_push_worker(self):
        while rclpy.ok():
            try:
                action, data = self.fb_queue.get(timeout=1.0)
                if action == 'push': self.stream_ref.push(data)
                elif action == 'delete': self.stream_ref.delete()
            except queue.Empty: continue

    def state_callback(self, msg):
        if msg.data == True and not self.is_overall_running:
            self.is_overall_running = True
            self.scan_buffer.clear()
            self.torque_buffer.clear()
            self.max_error_rate, self.max_e_max = 0.0, 0.0
            self.crack_active = False
            self.fb_queue.put(('delete', None)) 
            self.get_logger().info("\n🟢 [스캔 시작] 실시간 궤적 기록을 시작합니다.")
            
        elif msg.data == False and self.is_overall_running:
            self.is_overall_running = False
            self.is_recording = False
            final_status = "FAIL" if self.max_error_rate >= ERROR_LIMIT_PERCENT else "PASS"
            self.get_logger().info(f"\n🔴 [스캔 종료] 최종 판정: {final_status} (최대 오차율: {self.max_error_rate:.2f}%)")
            self.result_pub.publish(String(data=final_status))
            
            if final_status == "FAIL" and len(self.scan_buffer) > 0:
                threading.Thread(target=self.generate_and_upload_report, daemon=True).start()

    def enable_callback(self, msg):
        self.is_recording = msg.data
        if msg.data: 
            # 💡 [추가] 각 레이어 스캔이 새로 시작될 때마다 무시 카운터를 0으로 초기화
            self.ignore_count = 0
        else:
            self.torque_buffer.clear()

    def joint_callback(self, msg):
        try:
            q = [msg.position[msg.name.index(f'joint_{i}')] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            self.latest_pos = [float(current_pos[0]), float(current_pos[1]), float(current_pos[2])]
        except Exception: pass

    def timer_callback(self):
        if self.is_overall_running and self.is_recording:
            self.tq_cli.call_async(GetExternalTorque.Request()).add_done_callback(self.tq_cb)

    def tq_cb(self, future):
        try:
            res = future.result()
            if self.latest_pos is None: return
            
            current_torques = np.array(res.ext_torque)
            if len(self.torque_buffer) < 5:
                self.torque_buffer.append(current_torques); return
                
            baseline_torques = np.mean(self.torque_buffer, axis=0)
            torque_diffs = np.abs(current_torques - baseline_torques)
            max_spike = np.max(torque_diffs)
            
            # ---------------------------------------------------------
            # 💡 [핵심 변경] 초반 진입 노이즈 무시 로직 적용
            # ---------------------------------------------------------
            if self.ignore_count < IGNORE_START_FRAMES:
                self.ignore_count += 1
                is_crack = False  # 초반 1초간은 아무리 튀어도 크랙으로 인정하지 않음
            else:
                is_crack = bool(max_spike >= SPIKE_THRESHOLD)
                
            self.torque_buffer.append(current_torques); self.torque_buffer.pop(0)

            point = {'t': time.time(), 'x': self.latest_pos[0], 'y': self.latest_pos[1], 'z': self.latest_pos[2], 'is_crack': is_crack}
            self.fb_queue.put(('push', point))
            self.scan_buffer.append(point)

            import sys
            # 상태 터미널 출력 업데이트 (초반 무시 중일 때는 '대기 중' 표시)
            if self.ignore_count < IGNORE_START_FRAMES:
                sys.stdout.write(f"\r🔍 [진입 안정화 중...] Spike: {max_spike:5.2f} Nm | 오차: {self.max_error_rate:5.2f}%   ")
            else:
                sys.stdout.write(f"\r🔍 [기록 중] Z:{self.latest_pos[2]:6.1f} | Spike: {max_spike:5.2f} Nm | 오차: {self.max_error_rate:5.2f}%   ")
            sys.stdout.flush()

            if is_crack and not self.crack_active:
                self.crack_active, self.crack_start_idx, self.post_count = True, len(self.scan_buffer) - 1, 0
            elif self.crack_active:
                self.post_count += 1
                if self.post_count >= MARGIN_FRAMES:
                    self.run_hemiscan_analysis(); self.crack_active = False
        except: pass

    # =========================================================
    # 스플라인 발산 억제 및 수치 보정
    # =========================================================
    def run_hemiscan_analysis(self):
        idx = self.crack_start_idx
        start, end = max(0, idx-WINDOW_FRAMES), min(len(self.scan_buffer), idx+WINDOW_FRAMES)
        n_idx = list(range(max(0, start-MARGIN_FRAMES), start)) + list(range(end, min(len(self.scan_buffer), end+MARGIN_FRAMES)))
        if len(n_idx) < 5: return
        
        t_n = np.array([self.scan_buffer[i]['t'] for i in n_idx])
        p_n = np.array([[self.scan_buffer[i]['x'], self.scan_buffer[i]['y'], self.scan_buffer[i]['z']] for i in n_idx])
        t_a = np.array([self.scan_buffer[i]['t'] for i in range(start, end)])
        p_a = np.array([[self.scan_buffer[i]['x'], self.scan_buffer[i]['y'], self.scan_buffer[i]['z']] for i in range(start, end)])
        
        _, u_idx = np.unique(t_n, return_index=True)
        t_n, p_n = t_n[u_idx], p_n[u_idx]
        if len(t_n) < 4: return
        
        try:
            spline = CubicSpline(t_n, p_n, bc_type='natural')
            p_i = spline(t_a)
            dist = np.linalg.norm(p_a - p_i, axis=1)
            e_max = np.max(dist)
            
            # 물리적으로 불가능한 발산 수치 차단
            if e_max > 10.0:
                e_max = 2.0
                
            rate = (e_max / 5.0) * 100 # (반지름 대신 허용 기준 5mm 적용)
            
            if rate > self.max_error_rate: 
                self.max_error_rate, self.max_e_max = rate, e_max
        except: pass

    # =========================================================
    # FAIL 리포트 업로드 및 실시간 DB 기록
    # =========================================================
    def generate_and_upload_report(self):
        self.get_logger().info('📝 FAIL 리포트 생성 및 실시간 DB 기록 중...')
        now_time = datetime.datetime.now()
        timestamp_str = now_time.strftime("%Y%m%d_%H%M%S")
        filename = f'FAIL_report_{timestamp_str}.html'
        
        norm_data = [d for d in self.scan_buffer if not d['is_crack']]
        crack_data = [d for d in self.scan_buffer if d['is_crack']]

        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="UTF-8"><title>불량 리포트 - {timestamp_str}</title>
            <script src="https://cdn.plot.ly/plotly-2.24.1.min.js"></script>
            <style>body {{ font-family: sans-serif; background: #111; color: white; text-align: center; margin: 0; }}
            .panel {{ background: #ff4136; padding: 20px; font-size: 24px; font-weight: bold; }}
            .stats {{ display: flex; justify-content: center; gap: 50px; padding: 20px; background: #222; }}
            .val {{ color: #FF851B; font-size: 30px; }} #plot {{ width: 100vw; height: 70vh; }}</style>
        </head>
        <body>
            <div class="panel">🚨 불량 검출 리포트 (FAIL) - {timestamp_str}</div>
            <div class="stats">
                <div>최대 오차율: <br><span class="val">{self.max_error_rate:.3f} %</span></div>
                <div>최대 편차: <br><span class="val">{self.max_e_max:.3f} mm</span></div>
            </div>
            <div id="plot"></div>
            <script>
                const norm = {json.dumps(norm_data)}; const crack = {json.dumps(crack_data)};
                const t1 = {{ x: norm.map(d=>d.x), y: norm.map(d=>d.y), z: norm.map(d=>d.z), mode: 'markers', marker: {{size: 2, color: '#0074D9'}}, type: 'scatter3d', name: '정상 궤적' }};
                const t2 = {{ x: crack.map(d=>d.x), y: crack.map(d=>d.y), z: crack.map(d=>d.z), mode: 'markers', marker: {{size: 6, color: '#FF4136'}}, type: 'scatter3d', name: '크랙 궤적' }};
                Plotly.newPlot('plot', [t1, t2], {{ paper_bgcolor: '#111', plot_bgcolor: '#111', scene: {{ aspectmode: 'data', xaxis: {{color: '#fff'}}, yaxis: {{color: '#fff'}}, zaxis: {{color: '#fff'}} }}, font: {{color: '#fff'}} }});
            </script>
        </body>
        </html>
        """
        try:
            blob = self.bucket.blob(f'inspection_reports/{filename}')
            blob.upload_from_string(html_content, content_type='text/html')
            blob.make_public()
            
            db.reference("robot/defect").push({
                "triggeredAt": int(now_time.timestamp() * 1000),
                "date": now_time.strftime("%Y-%m-%d"),
                "time": now_time.strftime("%H:%M:%S"),
                "errorRate": f"{self.max_error_rate:.1f}%",
                "errorRateNum": float(self.max_error_rate),
                "storageUrl": blob.public_url 
            })
            
            self.get_logger().info(f'✅ 완료! 리얼타임 DB에 불량 기록됨.')
        except Exception as e:
            self.get_logger().error(f'업로드/DB기록 실패: {e}')

def main(args=None):
    rclpy.init(args=args); node = InspectionMasterNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok(): time.sleep(0.1)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()