import rclpy
from rclpy.node import Node
import sys
import os
import datetime
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from dsr_msgs2.srv import GetJointTorque
import time

class TorquePlotterNode(Node):
    def __init__(self):
        super().__init__('torque_plotter_node')
        
        self.is_overall_running = False # 전체 프로세스 상태 (checking_state)
        self.is_recording = False       # 실제 원 그리는 중인지 상태 (record_enable)
        
        # 데이터 수집용 리스트
        self.time_data = []
        self.torque_data = {1: [], 2: [], 3: [], 4: [], 5: [], 6: []}
        self.start_time = 0.0
        
        # 저장 경로 설정
        self.save_dir = os.path.join(os.getcwd(), 'torque_graphs')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # 1. 상태 구독 (spin 파일과 연동)
        self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.create_subscription(Bool, '/dsr01/record_enable', self.enable_callback, 10)

        # 2. 토크 서비스 클라이언트
        self.cli = self.create_client(GetJointTorque, '/dsr01/aux_control/get_joint_torque')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('로봇 토크 서비스 연결 대기 중...')
            
        self.get_logger().info('✅ 토크 모니터링 & 그래프 기록 준비 완료! (spin 코드를 실행해주세요)')
        
        self.req = GetJointTorque.Request()
        
        # 50Hz 간격으로 데이터 수집 타이머
        self.timer = self.create_timer(0.02, self.timer_callback)

    def state_callback(self, msg):
        # 전체 프로세스가 시작될 때 데이터 초기화
        if msg.data == True and not self.is_overall_running:
            self.get_logger().info('🟢 전체 스캔 프로세스 시작 감지! 데이터 수집 대기...')
            self.is_overall_running = True
            self.time_data = []
            self.torque_data = {1: [], 2: [], 3: [], 4: [], 5: [], 6: []}
            self.start_time = time.time()
            
        # 전체 프로세스가 끝날 때 그래프 저장
        elif msg.data == False and self.is_overall_running:
            self.get_logger().info('\n🔴 전체 스캔 프로세스 종료! 그래프 생성을 시작합니다...')
            self.is_overall_running = False
            self.is_recording = False
            self.save_and_plot()

    def enable_callback(self, msg):
        # 원을 그릴 때만 스위치 ON/OFF
        if msg.data:
            sys.stdout.write("\n▶️ 원 궤적 진입! 토크 기록을 시작합니다.\n")
        else:
            sys.stdout.write("\n⏸️ 궤적 종료! 토크 기록 일시정지.\n")
        self.is_recording = msg.data

    def timer_callback(self):
        # is_recording이 True(원을 그릴 때)에만 서비스 콜을 날림
        if self.is_recording:
            future = self.cli.call_async(self.req)
            future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            
            # 토크 데이터 추출
            if hasattr(response, 'joint_torque'):
                torques = response.joint_torque
            elif hasattr(response, 'jxt'): 
                torques = response.jxt
            else:
                return
                
            if len(torques) >= 6 and self.is_recording:
                # 시간 및 토크 배열 저장
                current_t = time.time() - self.start_time
                self.time_data.append(current_t)
                for i in range(6):
                    self.torque_data[i+1].append(torques[i])
                
                # 터미널에 진행 상황 애니메이션 출력
                sys.stdout.write(f"\r🦾 기록 중... 수집된 데이터: {len(self.time_data)}개")
                sys.stdout.flush()
                
        except Exception as e:
            self.get_logger().error(f"데이터 수집 에러: {e}")

    def save_and_plot(self):
        if len(self.time_data) == 0:
            self.get_logger().warning("저장할 토크 데이터가 없습니다.")
            return
            
        self.get_logger().info('📊 그래프 그리는 중...')
        
        plt.figure(figsize=(12, 6))
        
        # J1 ~ J6 꺾은선 그래프 추가
        colors = ['red', 'orange', 'green', 'blue', 'purple', 'brown']
        for i in range(1, 7):
            plt.plot(self.time_data, self.torque_data[i], label=f'Joint {i}', color=colors[i-1], linewidth=1.5)
            
        plt.title('Robot Joint Torque During Scanning', fontsize=16)
        plt.xlabel('Time (seconds)', fontsize=12)
        plt.ylabel('Torque (Nm)', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend(loc='upper right')
        
        # 파일 저장
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'torque_plot_{timestamp}.png'
        filepath = os.path.join(self.save_dir, filename)
        
        plt.tight_layout()
        plt.savefig(filepath, dpi=300)
        plt.close()
        
        self.get_logger().info(f'✅ 그래프 저장 완료: {filepath}')

def main(args=None):
    rclpy.init(args=args)
    node = TorquePlotterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 토크 모니터링 노드를 강제 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()