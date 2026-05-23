import rclpy
from rclpy.node import Node
import sys
import time
import threading
import queue
import json
import urllib.request

from dsr_msgs2.srv import GetExternalTorque

ROBOT_ID = "dsr01"

# Firebase REST API URL (Admin SDK 불필요 — JWT 서명 문제 우회)
FIREBASE_DB_URL = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app"
TORQUE_PATH = f"robot/{ROBOT_ID}/torques"

class TorqueWebMonitorNode(Node):
    def __init__(self):
        super().__init__('torque_web_monitor_node')
        
        self.get_logger().info(f'Firebase REST API 사용: {FIREBASE_DB_URL}')

        # Firebase push를 별도 스레드에서 처리
        self.fb_queue = queue.Queue()
        self.fb_worker = threading.Thread(target=self._firebase_worker, daemon=True)
        self.fb_worker.start()

        # External Torque 서비스 클라이언트
        self.cli = self.create_client(GetExternalTorque, '/dsr01/aux_control/get_external_torque')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('External Torque 서비스 연결 대기 중...')
            
        self.get_logger().info('✅ 토크 모니터 가동! Firebase REST API로 전송합니다.')
        
        self.req = GetExternalTorque.Request()
        self.push_count = 0
        self.fb_ok_count = 0
        self.fb_err_count = 0
        
        # 20Hz (0.05초) 간격으로 서비스 호출
        self.timer = self.create_timer(0.05, self.timer_callback)

    def _firebase_worker(self):
        """별도 스레드에서 Firebase REST API PUT 처리"""
        url = f"{FIREBASE_DB_URL}/{TORQUE_PATH}.json"
        while True:
            try:
                data = self.fb_queue.get(timeout=1.0)
                payload = json.dumps(data).encode('utf-8')
                req = urllib.request.Request(url, data=payload, method='PUT')
                req.add_header('Content-Type', 'application/json')
                with urllib.request.urlopen(req, timeout=2) as resp:
                    if resp.status == 200:
                        self.fb_ok_count += 1
                    else:
                        self.fb_err_count += 1
            except queue.Empty:
                continue
            except Exception as e:
                self.fb_err_count += 1
                # 에러를 자주 출력하지 않도록 100회마다 출력
                if self.fb_err_count % 100 == 1:
                    print(f'\n[Firebase REST 에러] {e}')

    def timer_callback(self):
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            
            if hasattr(response, 'ext_torque'):
                torques = response.ext_torque
                
                if len(torques) >= 6:
                    self.push_count += 1
                    
                    # Firebase에 push (큐에 넣기 — 비블로킹)
                    # 큐가 밀리면 최신 값만 유지
                    if self.fb_queue.qsize() < 5:
                        self.fb_queue.put({
                            'j1': round(float(torques[0]), 4),
                            'j2': round(float(torques[1]), 4),
                            'j3': round(float(torques[2]), 4),
                            'j4': round(float(torques[3]), 4),
                            'j5': round(float(torques[4]), 4),
                            'j6': round(float(torques[5]), 4),
                            'timestamp': int(time.time() * 1000)
                        })
                    
                    # 터미널 출력
                    sys.stdout.write(
                        f"\r[{self.push_count}] 🦾 "
                        f"J1:{torques[0]:7.2f} J2:{torques[1]:7.2f} J3:{torques[2]:7.2f} "
                        f"J4:{torques[3]:7.2f} J5:{torques[4]:7.2f} J6:{torques[5]:7.2f} "
                        f"| ✅{self.fb_ok_count} ❌{self.fb_err_count}   "
                    )
                    sys.stdout.flush()
        except Exception as e:
            print(f'\n[콜백 에러] {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TorqueWebMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 토크 모니터링을 종료합니다.")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
