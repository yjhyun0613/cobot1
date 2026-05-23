import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import cKDTree
import datetime
import os

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
        
        # 1. 기준 데이터(baseline) 로드 및 KDTree 생성
        try:
            self.baseline_points = np.load('/home/yoon/doosan-robot2/src/cobot1/cobot1/baseline.npy')
            # cKDTree: 공간에서 가장 가까운 점을 광속으로 찾아주는 알고리즘
            self.kdtree = cKDTree(self.baseline_points) 
            self.get_logger().info('✅ baseline.npy 로드 완료. 실시간 감시 준비 완료.')
        except FileNotFoundError:
            self.get_logger().error('❌ baseline.npy 파일이 없습니다. 1단계 코드부터 실행해주세요.')
            raise SystemExit
            
        # 오차 허용 임계값 (10mm = 0.01m)
        self.ERROR_THRESHOLD = 0.005 

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)

    def state_callback(self, msg):
        if msg.data == True and not self.is_checking:
            self.get_logger().info('🟢 감시 시작: 정상 궤적과 비교를 시작합니다.')
            self.is_checking = True
            self.error_detected = False
            self.trajectory_points = []
            
        elif msg.data == False and self.is_checking:
            self.get_logger().info('🔴 사이클 종료 (에러 없음)')
            self.is_checking = False

    def joint_callback(self, msg):
        if not self.is_checking or self.error_detected:
            return

        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            self.trajectory_points.append(current_pos)
            
            # --- 핵심 로직: 현재 위치에서 기준 궤적까지의 최소 거리 계산 ---
            # kdtree.query는 (가장 가까운 점과의 거리, 그 점의 인덱스)를 반환합니다.
            min_distance, _ = self.kdtree.query(current_pos)
            
            if min_distance > self.ERROR_THRESHOLD:
                self.error_detected = True
                self.is_checking = False # 데이터 수집 즉시 중단
                self.get_logger().error(f'🚨 경로 이탈 감지! 오차: {min_distance*1000:.1f}mm (허용치: 10mm)')
                
                # [여기에 로봇 정지 명령을 추가할 수 있습니다]
                # 예: os.system("ros2 service call /dsr01/system/stop dsr_msgs2/srv/Stop 'stop_mode: 1'")
                
                self.save_error_image(current_pos)
                
        except Exception as e:
            pass

    def save_error_image(self, error_pos):
        self.get_logger().info('📸 에러 위치를 이미지로 저장합니다...')
        
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 1. 기준 궤적(회색 점선) 그리기
        ax.plot(self.baseline_points[:, 0], self.baseline_points[:, 1], self.baseline_points[:, 2], 
                label='Baseline (Ideal)', color='gray', linestyle='--', alpha=0.6)
        
        # 2. 로봇이 이동한 실제 궤적(파란색) 그리기
        if len(self.trajectory_points) > 0:
            real_pts = np.array(self.trajectory_points)
            ax.plot(real_pts[:, 0], real_pts[:, 1], real_pts[:, 2], 
                    label='Real Trajectory', color='blue', linewidth=2.0)
            
        # 3. 에러가 감지된 정확한 지점(빨간색 큰 점) 강조
        ax.scatter(*error_pos, color='red', s=200, label='ERROR DETECTED!', zorder=10)
        ax.text(error_pos[0], error_pos[1], error_pos[2] + 0.01, "Deviation > 5mm", color='red')
        
        # 축 고정 (50x50 사이즈에 맞춤)
        ax.set_xlim(0.43, 0.53)  # 430mm ~ 530mm
        ax.set_ylim(-0.05, 0.05) # -50mm ~ 50mm
        ax.set_zlim(0.35, 0.45)  # 350mm ~ 450mm
        ax.set_box_aspect((1, 1, 1))
        
        ax.set_xlabel('X Axis (m)')
        ax.set_ylabel('Y Axis (m)')
        ax.set_zlabel('Z Axis (m)')
        ax.set_title('Path Deviation Detection')
        ax.legend()
        
        current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'error_detected_{current_time}.png'
        plt.savefig(filename, dpi=300)
        plt.close(fig)
        self.get_logger().info(f'✅ 에러 이미지 저장 완료: {filename}')

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