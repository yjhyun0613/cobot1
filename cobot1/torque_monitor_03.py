import rclpy
from rclpy.node import Node
import threading
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
from dsr_msgs2.srv import GetExternalTorque, GetCurrentPosx

# =========================================================
# ⚙️ 설정 파라미터
# =========================================================
RADIUS = 48.5
CRACK_THRESHOLD = 1.0
ERROR_LIMIT_PERCENT = 0.5
MARGIN_FRAMES = 20
WINDOW_FRAMES = 15

class InspectionMasterNode(Node):
    def __init__(self):
        super().__init__('inspection_master_node')
        
        # 1. Firebase 초기화 (Storage 버킷 추가)
        try:
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-7f5d874f48.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app',
                    'storageBucket': 'rokey-d-2-4c32a.firebasestorage.app' # 💡 추가된 부분
                })
            self.stream_ref = db.reference('robot/dsr01/stream')
            self.bucket = storage.bucket()
            self.get_logger().info('✅ Firebase (DB & Storage) 연결 완료!')
        except Exception as e:
            self.get_logger().error(f'Firebase 연결 실패: {e}')

        # 2. 시스템 상태 변수
        self.is_overall_running = False
        self.is_recording = False
        self.latest_pos = None
        self.scan_buffer = []
        self.max_error_rate = 0.0
        self.max_e_max = 0.0
        self.max_rmse = 0.0
        
        self.crack_active = False
        self.crack_start_idx = 0
        self.post_count = 0

        # 3. ROS 2 통신 설정
        self.pos_cli = self.create_client(GetCurrentPosx, '/dsr01/system/get_current_posx')
        self.tq_cli = self.create_client(GetExternalTorque, '/dsr01/aux_control/get_external_torque')
        
        # 구독(Sub) & 발행(Pub)
        self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.create_subscription(Bool, '/dsr01/record_enable', self.enable_callback, 10)
        self.result_pub = self.create_publisher(String, '/dsr01/inspection_result', 10)

        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz

    # =========================================================
    # 🔄 스캔 전체 흐름 제어 (시작 / 종료)
    # =========================================================
    def state_callback(self, msg):
        # [시작]
        if msg.data == True and not self.is_overall_running:
            self.is_overall_running = True
            self.scan_buffer.clear()
            self.max_error_rate, self.max_e_max, self.max_rmse = 0.0, 0.0, 0.0
            self.crack_active = False
            self.stream_ref.delete() # 웹 뷰어 초기화
            self.get_logger().info("\n🟢 [스캔 시작] 실시간 궤적 기록을 시작합니다.")
            
        # [종료]
        elif msg.data == False and self.is_overall_running:
            self.is_overall_running = False
            self.is_recording = False
            
            # PASS / FAIL 판정
            final_status = "FAIL" if self.max_error_rate >= ERROR_LIMIT_PERCENT else "PASS"
            self.get_logger().info(f"\n🔴 [스캔 종료] 최종 판정: {final_status} (최대 오차율: {self.max_error_rate:.2f}%)")
            
            # ROS 2 토픽으로 결과 쏘기
            result_msg = String()
            result_msg.data = final_status
            self.result_pub.publish(result_msg)
            
            # FAIL인 경우에만 Storage에 HTML 리포트 저장
            if final_status == "FAIL" and len(self.scan_buffer) > 0:
                self.generate_and_upload_report()

    # ⏯️ 기록 일시정지 / 재개 제어
    def enable_callback(self, msg):
        self.is_recording = msg.data

    # =========================================================
    # 📡 실시간 데이터 수집 & 스트리밍
    # =========================================================
    def timer_callback(self):
        if self.is_overall_running and self.is_recording:
            self.pos_cli.call_async(GetCurrentPosx.Request(ref=0)).add_done_callback(self.pos_cb)
            self.tq_cli.call_async(GetExternalTorque.Request()).add_done_callback(self.tq_cb)

    def pos_cb(self, future):
        try:
            res = future.result()
            self.latest_pos = res.pos[0:3] if hasattr(res, 'pos') else res.task_pos_info[0].pos[0:3]
        except: pass

    def tq_cb(self, future):
        try:
            res = future.result()
            if self.latest_pos is None: return
            
            ext_tq = np.max(np.abs(res.ext_torque))
            is_crack = bool(ext_tq >= CRACK_THRESHOLD)
            
            point = {'t': time.time(), 'x': self.latest_pos[0], 'y': self.latest_pos[1], 'z': self.latest_pos[2], 'is_crack': is_crack}
            
            # 1. Firebase 실시간 뷰어로 좌표 쏘기
            self.stream_ref.push(point)
            # 2. 로컬 블랙박스 저장
            self.scan_buffer.append(point)

            sys.stdout.write(f"\r🔍 [기록 중] 외력: {ext_tq:5.2f} Nm | 현재 최고 오차율: {self.max_error_rate:5.2f}%   ")
            sys.stdout.flush()

            # 3. 크랙 감지 상태 머신
            if is_crack and not self.crack_active:
                self.crack_active = True
                self.crack_start_idx = len(self.scan_buffer) - 1
                self.post_count = 0
            elif self.crack_active:
                self.post_count += 1
                if self.post_count >= MARGIN_FRAMES:
                    self.run_hemiscan_analysis()
                    self.crack_active = False
        except: pass

    # =========================================================
    # 🧮 Hemiscan 스플라인 분석 로직
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

        # 일시정지로 인해 시간이 겹치는 데이터 제거
        _, unique_idx = np.unique(t_n, return_index=True)
        t_n, p_n = t_n[unique_idx], p_n[unique_idx]
        if len(t_n) < 4: return

        try:
            spline = CubicSpline(t_n, p_n)
            p_i = spline(t_a)
            distances = np.linalg.norm(p_a - p_i, axis=1)
            e_max = np.max(distances)
            rmse = np.sqrt(np.mean(distances**2))
            error_rate = (e_max / RADIUS) * 100

            if error_rate > self.max_error_rate:
                self.max_error_rate = error_rate
                self.max_e_max = e_max
                self.max_rmse = rmse
        except: pass

    # =========================================================
    # 📄 FAIL 리포트 생성 및 Firebase Storage 업로드
    # =========================================================
    def generate_and_upload_report(self):
        self.get_logger().info('📝 FAIL 리포트를 생성하고 업로드합니다...')
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'FAIL_report_{timestamp}.html'
        
        # 자바스크립트용 데이터로 변환
        norm_data = [d for d in self.scan_buffer if not d['is_crack']]
        crack_data = [d for d in self.scan_buffer if d['is_crack']]

        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="UTF-8">
            <title>불량 리포트 - {timestamp}</title>
            <script src="https://cdn.plot.ly/plotly-2.24.1.min.js"></script>
            <style>
                body {{ font-family: sans-serif; background: #111; color: white; text-align: center; margin: 0; }}
                .panel {{ background: #ff4136; padding: 20px; font-size: 24px; font-weight: bold; }}
                .stats {{ display: flex; justify-content: center; gap: 50px; padding: 20px; background: #222; }}
                .val {{ color: #FF851B; font-size: 30px; }}
                #plot {{ width: 100vw; height: 70vh; }}
            </style>
        </head>
        <body>
            <div class="panel">🚨 불량 검출 리포트 (FAIL) - {timestamp}</div>
            <div class="stats">
                <div>최대 오차율: <br><span class="val">{self.max_error_rate:.3f} %</span></div>
                <div>최대 편차: <br><span class="val">{self.max_e_max:.3f} mm</span></div>
                <div>RMSE: <br><span class="val">{self.max_rmse:.3f} mm</span></div>
            </div>
            <div id="plot"></div>
            <script>
                const norm = {json.dumps(norm_data)};
                const crack = {json.dumps(crack_data)};
                
                const t1 = {{ x: norm.map(d=>d.x), y: norm.map(d=>d.y), z: norm.map(d=>d.z), mode: 'markers', marker: {{size: 2, color: '#0074D9'}}, type: 'scatter3d', name: '정상 궤적' }};
                const t2 = {{ x: crack.map(d=>d.x), y: crack.map(d=>d.y), z: crack.map(d=>d.z), mode: 'markers', marker: {{size: 6, color: '#FF4136'}}, type: 'scatter3d', name: '크랙 궤적' }};
                
                Plotly.newPlot('plot', [t1, t2], {{ paper_bgcolor: '#111', plot_bgcolor: '#111', scene: {{ aspectmode: 'data', xaxis: {{color: '#fff'}}, yaxis: {{color: '#fff'}}, zaxis: {{color: '#fff'}} }}, font: {{color: '#fff'}} }});
            </script>
        </body>
        </html>
        """
        
        try:
            # Storage 지정된 폴더에 업로드
            blob = self.bucket.blob(f'inspection_reports/{filename}')
            blob.upload_from_string(html_content, content_type='text/html')
            blob.make_public() # 누구나 링크로 볼 수 있게 허용
            self.get_logger().info(f'✅ 업로드 완료! 리포트 링크:\n{blob.public_url}')
        except Exception as e:
            self.get_logger().error(f'업로드 실패: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = InspectionMasterNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        while rclpy.ok(): time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()