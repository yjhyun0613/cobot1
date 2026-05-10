import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import datetime
import os

# 이미지 저장 경로 (사용자 설정 환경)
IMAGE_DIR = '/home/yoon/cobot_ws/src/cobot1/image'

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
    
    # ✅ [툴 없이 실행] Flange 중심 기준
    TOOL_LENGTH_Z = 0.0 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]

class TrajectoryMonitorNode(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.is_checking = False
        self.trajectory_points = []
        self.error_detected = False
        self.ERROR_THRESHOLD = 0.005 # 5mm 이탈 기준
        
        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)

        try:
            # 3중 원 데이터가 통합된 baseline.npy 로드
            self.baseline_points = np.load('/home/yoon/cobot_ws/src/cobot1/cobot1/baseline.npy')
            self.kdtree = cKDTree(self.baseline_points) 
            self.get_logger().info('✅ 통합 기준 데이터 로드 완료 (3중 원 검사 준비)')
        except FileNotFoundError:
            self.get_logger().error('❌ baseline.npy 파일이 없습니다.')
            raise SystemExit

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)

    def state_callback(self, msg):
        if msg.data == True and not self.is_checking:
            self.get_logger().info('🟢 검사 시작 (3중 원 경로 감시 중)')
            self.is_checking = True
            self.error_detected = False
            self.trajectory_points = []
        elif msg.data == False and self.is_checking:
            self.get_logger().info('🔴 검사 정상 종료')
            self.is_checking = False

    def joint_callback(self, msg):
        if not self.is_checking or self.error_detected: return
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            current_pos = forward_kinematics(q)
            self.trajectory_points.append(current_pos)
            
            min_distance, _ = self.kdtree.query(current_pos)
            if min_distance > self.ERROR_THRESHOLD:
                self.error_detected = True
                self.get_logger().error(f'🚨 이탈 감지! 오차: {min_distance*1000:.1f}mm')
                self.save_optimized_image(current_pos)
        except Exception: pass

    def save_optimized_image(self, error_pos):
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 1. 기준 경로 (3중 원 전체 표시)[cite: 3, 5]
        ax.plot(self.baseline_points[:, 0], self.baseline_points[:, 1], self.baseline_points[:, 2], 
                label='Baseline (3-Circles)', color='gray', alpha=0.3, linestyle='--')
        
        # 2. 현재 실제 경로
        if self.trajectory_points:
            pts = np.array(self.trajectory_points)
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], label='Real Trajectory', color='blue', linewidth=2)
            
        # 3. 에러 발생 지점 강조[cite: 3]
        ax.scatter(*error_pos, color='red', s=200, label='ERROR DETECTED', edgecolors='black', zorder=10)
        
        # 💡 [핵심 수정] Z축 과도한 확대 방지 및 1:1:1 비율 설정[cite: 3]
        all_pts = np.vstack([self.baseline_points, pts])
        max_range = np.array([np.ptp(all_pts[:, 0]), np.ptp(all_pts[:, 1]), np.ptp(all_pts[:, 2])]).max() / 2.0
        
        mid_x = (np.max(all_pts[:, 0]) + np.min(all_pts[:, 0])) * 0.5
        mid_y = (np.max(all_pts[:, 1]) + np.min(all_pts[:, 1])) * 0.5
        mid_z = (np.max(all_pts[:, 2]) + np.min(all_pts[:, 2])) * 0.5
        
        # 너무 미세하게 확대되지 않도록 최소 범위를 0.05m(50mm)로 보장[cite: 3]
        if max_range < 0.025: max_range = 0.025 

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        ax.set_title(f'Path Deviation Map (Error: {error_pos[2]*1000:.1f}mm)')
        ax.legend()
        
        filename = f'inspection_error_{datetime.datetime.now().strftime("%H%M%S")}.png'
        plt.savefig(os.path.join(IMAGE_DIR, filename), dpi=300)
        plt.close(fig)
        self.get_logger().info(f'📸 전체 궤적 이미지 저장 완료: {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMonitorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()