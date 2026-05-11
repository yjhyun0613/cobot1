import rclpy
from rclpy.node import Node
import sys
import time
from collections import deque
import numpy as np
from scipy.interpolate import CubicSpline

# 두산 ROS 2 서비스 임포트
from dsr_msgs2.srv import GetJointTorque, GetCurrentPosx

# =========================================================
# ⚙️ 사용자 설정 파라미터
# =========================================================
CRACK_THRESHOLD = 15.0  # [수정 필요] 이 토크(Nm)를 넘으면 크랙으로 간주합니다!
HZ = 50                 # 50Hz (0.02초)
BUFFER_SECONDS = 3.0    # 3초 분량의 블랙박스 데이터 유지 (약 150개 데이터)

WINDOW_SIZE = 15        # 충격 기준 앞뒤로 잘라낼 에러 구간 데이터 개수
MARGIN = 20             # 이상적 곡선을 그리기 위해 가져올 정상 구간 데이터 개수
# =========================================================

class RealtimeHemiscan(Node):
    def __init__(self):
        super().__init__('realtime_hemiscan_node')
        
        self.get_logger().info('📡 로봇 서비스(토크, 위치) 연결 중...')
        self.tq_cli = self.create_client(GetJointTorque, '/dsr01/aux_control/get_joint_torque')
        self.pos_cli = self.create_client(GetCurrentPosx, '/dsr01/system/get_current_posx')
        
        while not self.tq_cli.wait_for_service(timeout_sec=1.0) or not self.pos_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('대기 중...')
            
        self.get_logger().info('✅ 실시간 블랙박스 및 크랙 감지 시스템 가동 시작!')

        # 요청서 껍데기
        self.tq_req = GetJointTorque.Request()
        self.pos_req = GetCurrentPosx.Request()
        self.pos_req.ref = 0 # 0: Base 좌표계 기준
        
        # 최신 데이터를 임시 저장할 변수
        self.latest_pos = None
        
        # 블랙박스(최대 3초치 데이터만 유지, 꽉 차면 과거 데이터부터 삭제됨)
        self.history = deque(maxlen=int(HZ * BUFFER_SECONDS))
        
        # 상태 머신 변수
        self.is_capturing = False
        self.post_crack_frames = 0
        self.target_margin_frames = WINDOW_SIZE + MARGIN # 충격 이후 수집해야 할 프레임 수
        
        # 50Hz 타이머 시작
        self.timer = self.create_timer(1.0 / HZ, self.timer_callback)

    def timer_callback(self):
        # 1. 위치 요청 (비동기)
        pos_future = self.pos_cli.call_async(self.pos_req)
        pos_future.add_done_callback(self.pos_callback)
        
        # 2. 토크 요청 (비동기)
        tq_future = self.tq_cli.call_async(self.tq_req)
        tq_future.add_done_callback(self.tq_callback)

    def pos_callback(self, future):
        try:
            res = future.result()
            if hasattr(res, 'pos'):
                self.latest_pos = res.pos[0:3] # [X, Y, Z]
            elif hasattr(res, 'task_pos_info'): # 구버전 호환
                self.latest_pos = res.task_pos_info[0].pos[0:3]
        except Exception: pass

    def tq_callback(self, future):
        try:
            res = future.result()
            torques = res.joint_torque if hasattr(res, 'joint_torque') else res.jxt
            
            # 위치 데이터가 아직 도착 안했으면 무시
            if self.latest_pos is None or len(torques) < 6: return
            
            curr_time = time.time()
            max_tq = np.max(np.abs(torques)) # 6축 중 가장 크게 걸린 힘 (절댓값)
            
            # 블랙박스에 [시간, X, Y, Z, Max토크] 저장
            data_point = (curr_time, self.latest_pos[0], self.latest_pos[1], self.latest_pos[2], max_tq)
            self.history.append(data_point)
            
            # 터미널 실시간 출력 (기본)
            if not self.is_capturing:
                sys.stdout.write(f"\r🔍 [모니터링 중] X:{self.latest_pos[0]:6.1f} | Y:{self.latest_pos[1]:6.1f} | Z:{self.latest_pos[2]:6.1f} | 현재 최대 토크: {max_tq:5.1f} Nm   ")
                sys.stdout.flush()

            # =================================================
            # 🚨 크랙 감지 (이벤트 트리거)
            # =================================================
            if max_tq >= CRACK_THRESHOLD and not self.is_capturing:
                self.is_capturing = True
                self.post_crack_frames = 0
                print(f"\n\n💥 [경고] 임계값 초과! (토크: {max_tq:.1f} Nm)")
                print("📸 크랙 이후의 마진 데이터를 캡처합니다...")

            # =================================================
            # 📸 캡처 완료 후 사후 분석 (상태 머신)
            # =================================================
            if self.is_capturing:
                self.post_crack_frames += 1
                
                # 마진 데이터가 다 모이면 분석 시작!
                if self.post_crack_frames >= self.target_margin_frames:
                    print(f"✅ 데이터 캡처 완료. 분석을 시작합니다...\n")
                    self.analyze_crack_data()
                    
                    # 분석이 끝나면 다음 크랙을 찾기 위해 초기화
                    self.is_capturing = False
                    self.history.clear() # 이전 기록을 지워 연속 감지 방지
                    
        except Exception: pass

    def analyze_crack_data(self):
        # 덱(deque)에 있는 데이터를 넘파이 배열로 변환
        data = list(self.history)
        if len(data) < (WINDOW_SIZE * 2 + MARGIN * 2):
            print("⚠️ 버퍼에 데이터가 충분하지 않아 분석을 건너뜁니다.")
            return

        raw_time = np.array([row[0] for row in data])
        raw_time = raw_time - raw_time[0] # 시간을 0초부터 시작하게 정규화
        raw_positions = np.array([row[1:4] for row in data])
        raw_torques = np.array([row[4] for row in data])

        # hemiscan.py 의 분석 로직 실행
        spike_idx = np.argmax(raw_torques)
        err_start = max(0, spike_idx - WINDOW_SIZE)
        err_end = min(len(data), spike_idx + WINDOW_SIZE)
        
        # 안전 장치: 인덱스가 범위를 벗어나지 않게 처리
        if err_start - MARGIN < 0 or err_end + MARGIN >= len(data):
            print("⚠️ 크랙이 버퍼의 너무 끝부분에 있습니다. 분석 불가.")
            return

        t_actual = raw_time[err_start:err_end]
        actual_path = raw_positions[err_start:err_end]
        
        normal_idx = list(range(err_start - MARGIN, err_start)) + list(range(err_end, err_end + MARGIN))
        t_normal = raw_time[normal_idx]
        p_normal = raw_positions[normal_idx]
        
        # 3차 스플라인 곡선 추정 (시간 순서가 뒤섞이면 에러나므로 정렬 확인)
        spline_model = CubicSpline(t_normal, p_normal)
        ideal_path = spline_model(t_actual)
        
        # 오차(RMSE, Max) 계산
        distances = np.linalg.norm(actual_path - ideal_path, axis=1)
        e_max = np.max(distances)
        rmse = np.sqrt(np.mean(distances**2))
        
        print("="*50)
        print("📊 [Hemiscan 크랙 분석 결과]")
        print(f" - 크랙 지점 시간: {raw_time[spike_idx]:.2f} sec")
        print(f" - 최대 위치 편차 (Max Deviation): {e_max:.4f} mm")
        print(f" - 평균 제곱근 오차 (RMSE)     : {rmse:.4f} mm")
        print("="*50 + "\n")
        
        # (원한다면 여기서 e_max나 rmse가 특정 기준을 넘을 때 불량(NG) 판정을 내릴 수 있습니다.)

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeHemiscan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 시스템을 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()