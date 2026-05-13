import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class ConveyorSensorBridge(Node):
    def __init__(self):
        super().__init__('conveyor_sensor_bridge')
        try:
            self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)
            self.get_logger().info("=== 컨베이어 센서 브릿지 가동 (시작 명령 '00' 대기 중) ===")
        except Exception as e:
            self.get_logger().error(f"시리얼 연결 실패: {e}")
            return

        # [추가] 통합 토픽에서 '00'(시작 명령)을 듣기 위한 Subscriber
        self.command_sub = self.create_subscription(
            String, '/dsr01/progress_status', self.command_callback, 10)

        # 센서 가림 신호(01)를 지휘관에게 보내는 용도
        self.publisher_ = self.create_publisher(String, 'conveyor_stop_signal', 10)
        
        threading.Thread(target=self.receive_arduino, daemon=True).start()

    # [추가] 00 신호를 받으면 아두이노를 깨우는 함수
    def command_callback(self, msg):
        if msg.data == "00":
            self.get_logger().info(">>> [수신] 시작 명령(00) 수신! 컨베이어 모터를 가동합니다.")
            try:
                self.ser.write(b"00\n")  # 아두이노로 00 전송
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