import rclpy
from rclpy.node import Node
import threading
import time
import numpy as np
from scipy.interpolate import CubicSpline
import firebase_admin
from firebase_admin import credentials, db

# ROS 2 메시지 및 서비스
from std_msgs.msg import Bool
from dsr_msgs2.srv import GetExternalTorque, GetCurrentPosx

# =========================================================
# ⚙️ 설정 파라미터
# =========================================================
RADIUS = 48.5           # 돔 반지름 (mm)
CRACK_THRESHOLD = 1.5   # 외력 임계값 (Nm)
ERROR_LIMIT_PERCENT = 1.0 # PASS/FAIL 기준 (%)
MARGIN_FRAMES = 20      # 분석을 위한 후속 데이터 개수
WINDOW_FRAMES = 15      # 분석할 크랙 구간 크기

class InspectionBackend(Node):
    def __init__(self):
        super().__init__('inspection_backend_node')
        
        # 1. Firebase 초기화
        try:
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-7f5d874f48.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.stream_ref = db.reference('robot/dsr01/stream')
            self.result_ref = db.reference('robot/dsr01/results')
            self.get_logger().info('✅ Firebase 연결 성공!')
        except Exception as e:
            self.get_logger().error(f'Firebase 연결 실패: {e}')

        # 2. 상태 변수
        self.latest_pos = None
        self.scan_buffer = []
        self.max_error_rate = 0.0
        self.is_recording = False
        
        # 크랙 분석 상태 머신
        self.crack_active = False
        self.crack_start_idx = 0
        self.post_count = 0

        # 3. ROS 서비스 설정
        self.pos_cli = self.create_client(GetCurrentPosx, '/dsr01/system/get_current_posx')
        self.tq_cli = self.create_client(GetExternalTorque, '/dsr01/aux_control/get_external_torque')
        self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.create_subscription(Bool, '/dsr01/record_enable', self.enable_callback, 10)

        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz

    def state_callback(self, msg):
        if msg.data: # 💡 새로운 스캔 시작 시 Firebase 및 버퍼 초기화
            self.scan_buffer.clear()
            self.max_error_rate = 0.0
            self.stream_ref.delete()
            self.result_ref.set({'max_error_rate': 0, 'status': 'WAITING', 'max_e_max': 0})
            print("\n🔄 새로운 검사 시작: 대시보드 초기화 완료")

    def enable_callback(self, msg):
        self.is_recording = msg.data

    def timer_callback(self):
        if not self.is_recording: return
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
            curr_t = time.time()

            # 데이터 포인트 생성
            point = {'t': curr_t, 'x': self.latest_pos[0], 'y': self.latest_pos[1], 'z': self.latest_pos[2], 'is_crack': is_crack}
            
            # 1. Firebase로 좌표 즉시 전송 (HTML에서 그림 그리기용)
            self.stream_ref.push(point)
            
            # 2. 로컬 버퍼 저장 (분석용)
            self.scan_buffer.append(point)

            # 3. 크랙 분석 로직 (Hemiscan)
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

    def run_hemiscan_analysis(self):
        # hemiscan.py 로직을 이용한 오차 계산
        idx = self.crack_start_idx
        start, end = max(0, idx-WINDOW_FRAMES), min(len(self.scan_buffer), idx+WINDOW_FRAMES)
        
        # 정상 구간 추출
        n_idx = list(range(max(0, start-MARGIN_FRAMES), start)) + list(range(end, min(len(self.scan_buffer), end+MARGIN_FRAMES)))
        if len(n_idx) < 5: return

        t_n = np.array([self.scan_buffer[i]['t'] for i in n_idx])
        p_n = np.array([[self.scan_buffer[i]['x'], self.scan_buffer[i]['y'], self.scan_buffer[i]['z']] for i in n_idx])
        t_a = np.array([self.scan_buffer[i]['t'] for i in range(start, end)])
        p_a = np.array([[self.scan_buffer[i]['x'], self.scan_buffer[i]['y'], self.scan_buffer[i]['z']] for i in range(start, end)])

        try:
            spline = CubicSpline(t_n, p_n)
            p_i = spline(t_a)
            e_max = np.max(np.linalg.norm(p_a - p_i, axis=1))
            error_rate = (e_max / RADIUS) * 100

            if error_rate > self.max_error_rate:
                self.max_error_rate = error_rate
                status = "FAIL" if error_rate >= ERROR_LIMIT_PERCENT else "PASS"
                self.result_ref.update({'max_error_rate': round(error_rate, 3), 'max_e_max': round(e_max, 3), 'status': status})
                print(f"💥 크랙 분석 완료: 오차율 {error_rate:.2f}% -> {status}")
        except: pass

def main():
    rclpy.init()
    node = InspectionBackend()
    try: rclpy.spin(node)
    except: rclpy.shutdown()

if __name__ == '__main__':
    main()