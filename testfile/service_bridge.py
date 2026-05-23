import rclpy
from rclpy.node import Node
import DR_init
import threading

# ROS 2 서비스 타입 (Doosan 전용)
from dsr_msgs2.srv import MoveJoint, MoveLine, MoveHome, ChangeOperationSpeed

# Firebase 관련
import firebase_admin
from firebase_admin import credentials, db

# 1. Firebase 초기화
cred = credentials.Certificate("serviceAccountKey.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app"
})

class RobotCommandBridge(Node):
    def __init__(self, dsr01):
        super().__init__('robot_command_bridge', namespace=dsr01)
        self.robot_id = dsr01
        
        # 서비스 클라이언트 생성
        self.srv_move_joint = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.srv_move_line = self.create_client(MoveLine, '/dsr01/motion/move_line')
        self.srv_move_home = self.create_client(MoveHome, '/dsr01/motion/move_home')
        self.srv_change_speed = self.create_client(ChangeOperationSpeed, '/dsr01/motion/change_operation_speed')

        # Firebase 명령 대기 경로 설정 및 리스너 등록
        self.cmd_ref = db.reference('commands/dsr01')
        self.cmd_ref.listen(self.on_command_received)

    def on_command_received(self, event):
        # event.data가 None이거나 처음 연결 시 발생하는 이벤트를 걸러냄
        if event.data is None or event.path == "/":
            return

        command = event.data
        print(f"새로운 명령 수신: {command}")

        action = command.get('action')
        
        if action == 'move_joint':
            self.call_move_joint(command.get('pos'), command.get('vel', 30.0), command.get('acc', 30.0))
        elif action == 'move_line':
            self.call_move_line(command.get('pos'), command.get('vel', 100.0), command.get('acc', 100.0))
        elif action == 'move_home':
            self.call_move_home()
        elif action == 'change_speed':
            self.call_change_speed(command.get('speed', 100))

    # --- 서비스 호출 함수들 ---
    def call_move_joint(self, pos, vel, acc):
        req = MoveJoint.Request()
        req.pos = [float(x) for x in pos]
        req.vel = float(vel)
        req.acc = float(acc)
        self.srv_move_joint.call_async(req)

    def call_move_line(self, pos, vel, acc):
        req = MoveLine.Request()
        req.pos = [float(x) for x in pos]
        req.vel = [float(vel)] * 2 # 선속도, 각속도
        req.acc = [float(acc)] * 2
        self.srv_move_line.call_async(req)

    def call_move_home(self):
        req = MoveHome.Request()
        self.srv_move_home.call_async(req)

    def call_change_speed(self, speed):
        req = ChangeOperationSpeed.Request()
        req.speed = int(speed)
        self.srv_change_speed.call_async(req)

def start_ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "m0609"
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    rclpy.init(args=args)
    bridge_node = RobotCommandBridge(ROBOT_ID)
    DR_init.__dsr__node = bridge_node 

    # 백그라운드에서 Firebase 명령 감시 실행
    thread = threading.Thread(target=start_ros_spin, args=(bridge_node,), daemon=True)
    thread.start()

    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    print("--- Web to Robot Service Bridge Active ---")

    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()