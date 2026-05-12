import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time  # [추가] time 모듈 임포트

class ConveyorSensorBridge(Node):
    def __init__(self):
        super().__init__('conveyor_sensor_bridge')
        
        # [추가] 아두이노에 시작 명령을 중복으로 보내지 않기 위한 플래그
        self.arduino_started = False 

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("=== 컨베이어 센서 브릿지 가동 (시작 명령 '00' 대기 중) ===")
        except Exception as e:
            self.get_logger().error(f"시리얼 연결 실패: {e}")
            return

        self.command_sub = self.create_subscription(
            String, '/dsr01/progress_status', self.command_callback, 10)

        self.publisher_ = self.create_publisher(String, 'conveyor_stop_signal', 10)
        
        threading.Thread(target=self.receive_arduino, daemon=True).start()

    def command_callback(self, msg):
        # [수정] 명령이 '00'이고, 아직 아두이노를 가동하지 않았을 때만 딱 한 번 전송!
        if msg.data == "00" and not self.arduino_started:
            self.arduino_started = True
            self.get_logger().info(">>> [수신] 시작 명령(00) 수신! 컨베이어 모터를 가동합니다.")
            try:
                self.ser.write(b"00\n")  
            except Exception as e:
                self.get_logger().error(f"시리얼 전송 실패: {e}")

    def receive_arduino(self):
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if not line: continue
                    
                    if line == "STOP":
                        msg = String()
                        msg.data = "01"
                        self.publisher_.publish(msg)
                        self.get_logger().warn(">>> [이벤트] 물체 감지! (지휘관에게 01 발송)")
                    elif line == "GO":
                        self.get_logger().info(">>> [상태] 다시 주행 시작")
                except Exception as e:
                    pass
            else:
                # [핵심 수정] 수신할 데이터가 없으면 아주 잠깐(0.01초) 스레드를 재워 CPU 과부하를 막음
                time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()