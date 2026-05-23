import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt
import os
import datetime

# 이미지 저장 경로
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
    
    TOOL_LENGTH_Z = 0.0 
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]

class BaselineRecorderNode(Node):
    def __init__(self):
        super().__init__('baseline_recorder')
        self.is_recording = False
        self.trajectory_points = []
        
        if not os.path.exists(IMAGE_DIR):
            os.makedirs(IMAGE_DIR)

        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.get_logger().info('📡 기준 데이터 수집 대기 중... (툴 미사용)')

    def state_callback(self, msg):
        if msg.data == True and not self.is_recording:
            self.get_logger().info('🟢 수집 시작')
            self.is_recording = True
            self.trajectory_points = []
        elif msg.data == False and self.is_recording:
            self.get_logger().info('🔴 수집 종료 및 저장 중...')
            self.is_recording = False
            self.save_baseline_with_image()

    def joint_callback(self, msg):
        if not self.is_recording: return
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            self.trajectory_points.append(forward_kinematics(q))
        except KeyError: pass

    def save_baseline_with_image(self):
        if not self.trajectory_points: return
        
        pts = np.array(self.trajectory_points)
        np.save('baseline.npy', pts)
        
        # 궤적 이미지 생성 및 저장[cite: 4]
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], label='Baseline Path', color='green', linewidth=2)
        ax.set_title('Recorded Baseline Trajectory')
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'baseline_{timestamp}.png'
        plt.savefig(os.path.join(IMAGE_DIR, filename))
        plt.close(fig)
        self.get_logger().info(f'✅ 데이터 및 이미지 저장 완료: {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = BaselineRecorderNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()