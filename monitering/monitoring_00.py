import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import datetime
import os

# 스크린샷 구조에 따른 image 폴더 경로 설정 (실행 위치가 cobot1/cobot1 일 때 기준)
IMAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '/home/yoon/cobot_ws/src/cobot1/image'))

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
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56)[:3, 3]

class TrajectoryMonitorNode(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.is_checking = False
        self.trajectory_points = []
        self.error_detected = False
        self.ERROR_THRESHOLD = 0.005 # 5mm      
        
        # image 폴더 확인 및 생성
        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)
            self.get_logger().info(f'📁 {IMAGE_DIR} 폴더를 생성했습니다.')

        try:
            self.baseline_points = np.load('/home/yoon/cobot_ws/src/cobot1/cobot1/baseline.npy')
            self.kdtree = cKDTree(self.baseline_points) 
            self.get_logger().info('✅ baseline.npy 로드 완료. 실시간 감시 준비 완료.')
        except FileNotFoundError:
            self.get_logger().error('❌ baseline.npy 파일이 없습니다. 수집 노드를 먼저 실행해주세요.')
            raise SystemExit

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)

    def stop_robot(self):
        """두산 로봇 강제 정지 서비스 호출"""
        self.get_logger().warn('🛑 로봇 강제 정지 명령 전송 중...')
        # stop_mode 1: Quick Stop (현재 모션 취소 후 감속 정지)
        os.system("ros2 service call /dsr01/system/stop dsr_msgs2/srv/Stop '{stop_mode: 1}'")
        self.get_logger().info('🛑 로봇 정지 명령 전송 완료.')

    def state_callback(self, msg):
        if msg.data == True and not self.is_checking:
            self.get_logger().info('🟢 감시 시작')
            self.is_checking = True
            self.error_detected = False
            self.trajectory_points = []
        elif msg.data == False and self.is_checking:
            self.get_logger().info('🔴 감시 종료 (에러 없음)')
            self.is_checking = False

    def joint_callback(self, msg):
        if not self.is_checking or self.error_detected:
            return

        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            self.trajectory_points.append(current_pos)
            
            min_distance, _ = self.kdtree.query(current_pos)
            
            if min_distance > self.ERROR_THRESHOLD:
                self.error_detected = True
                self.is_checking = False
                
                # 1. 로봇 즉시 정지
                self.stop_robot()
                
                self.get_logger().error(f'🚨 경로 이탈 감지! 오차: {min_distance*1000:.1f}mm')
                
                # 2. 이미지 저장
                self.save_error_image(current_pos)
                
        except Exception as e:
            pass

    def save_error_image(self, error_pos):
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.plot(self.baseline_points[:, 0], self.baseline_points[:, 1], self.baseline_points[:, 2], 
                label='Baseline', color='gray', linestyle='--', alpha=0.6)
        
        if len(self.trajectory_points) > 0:
            real_pts = np.array(self.trajectory_points)
            ax.plot(real_pts[:, 0], real_pts[:, 1], real_pts[:, 2], 
                    label='Real Trajectory', color='blue', linewidth=2.0)
            
        ax.scatter(*error_pos, color='red', s=200, label='ERROR DETECTED', zorder=10)
        
        # 현재 위치 기준 100mm 반경으로 축 고정 (어떤 크기의 도형이든 유연하게 대처)
        ax.set_xlim(error_pos[0] - 0.05, error_pos[0] + 0.05)
        ax.set_ylim(error_pos[1] - 0.05, error_pos[1] + 0.05)
        ax.set_zlim(error_pos[2] - 0.05, error_pos[2] + 0.05)
        ax.set_box_aspect((1, 1, 1))
        
        ax.set_title('Path Deviation Detection')
        ax.legend()
        
        # 요청하신 파일명 형식: YYYY.MM.DD HH:MM (운영체제 호환성을 위해 콜론을 언더바로 대체)
        current_time = datetime.datetime.now().strftime("%Y.%m.%d %H_%M")
        filename = f'{current_time}.png'
        filepath = os.path.join(IMAGE_DIR, filename)
        
        plt.savefig(filepath, dpi=300)
        plt.close(fig)
        self.get_logger().info(f'📸 에러 이미지 저장 완료: {filepath}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()