import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TaskController(Node):
    def __init__(self):
        super().__init__('task_controller')
        self.get_logger().info("=== [작업 지휘관] 가동 완료. (시작 명령 '00' 대기 중) ===")

        # [기억 공간]
        self.item_ready = False  
        self.robot_busy = False  

        self.conveyor_sub = self.create_subscription(
            String, 'conveyor_stop_signal', self.conveyor_callback, 10)
        
        # 여기서 로봇 완료(04) 신호뿐만 아니라, 전체 시작(00) 신호도 같이 듣습니다.
        self.robot_sub = self.create_subscription(
            String, '/dsr01/progress_status', self.robot_callback, 10)
        
        self.command_pub = self.create_publisher(
            String, '/dsr01/progress_status', 10)

    def conveyor_callback(self, msg):
        if msg.data == "01":
            self.get_logger().info("[수신] 컨베이어: 물건 도착 (01)")
            self.item_ready = True 
            self.check_and_start_work()

    def robot_callback(self, msg):
        # [추가] 전체 공정 시작 신호 (00)
        if msg.data == "00":
            self.get_logger().info(">>> [시스템] 공정 시작 명령 (00) 수신! 지휘관 상태 초기화.")
            self.item_ready = False
            self.robot_busy = False
            
        # 기존 로봇 작업 완료 신호 (04)
        elif msg.data == "04":
            self.get_logger().info("[수신] 로봇: 공정 끝남 (04)")
            self.robot_busy = False 
            self.check_and_start_work()

    def check_and_start_work(self):
        if self.item_ready and not self.robot_busy:
            self.get_logger().warn(">>> 조건 충족! 로봇에게 작업 시작 명령(01) 하달!")
            
            start_msg = String()
            start_msg.data = "01"    
            self.command_pub.publish(start_msg)
            
            self.item_ready = False
            self.robot_busy = True

def main(args=None):
    rclpy.init(args=args)
    node = TaskController()
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