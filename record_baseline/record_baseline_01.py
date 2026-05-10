import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
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
    
    # 💡 [툴 길이 적용] 그리퍼 + 부품 A의 Z축 길이 (m 단위)
    # 예: 20cm = 0.200m 
    # TOOL_LENGTH_Z = 0.2286+0.0295 # 그리퍼 + 부품 A의 Z축 길이 (m 단위)  
    TOOL_LENGTH_Z = 0.0 # 그리퍼 + 부품 A의 Z축 길이 (m 단위)  
    T_tool = get_transform_matrix(0, 0, TOOL_LENGTH_Z, 0, 0, 0)
    
    # 마지막에 T_tool을 곱하여 실제 끝단 좌표 반환
    return (T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T_tool)[:3, 3]

class BaselineRecorderNode(Node):
    def __init__(self):
        super().__init__('baseline_recorder')
        self.is_recording = False
        self.trajectory_points = []
        
        self.joint_sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(Bool, '/dsr01/checking_state', self.state_callback, 10)
        self.get_logger().info('📡 기준 데이터 수집 대기 중... (checking_state 신호를 기다립니다)')

    def state_callback(self, msg):
        if msg.data == True and not self.is_recording:
            self.get_logger().info('🟢 수집 시작: 로봇의 이동 궤적을 기록합니다.')
            self.is_recording = True
            self.trajectory_points = []
            
        elif msg.data == False and self.is_recording:
            self.get_logger().info('🔴 수집 종료: 데이터를 저장합니다.')
            self.is_recording = False
            self.save_baseline()

    def joint_callback(self, msg):
        if not self.is_recording:
            return
        try:
            joint_data = dict(zip(msg.name, msg.position))
            q = [joint_data[f'joint_{i}'] for i in range(1, 7)]
            pos = forward_kinematics(q)
            self.trajectory_points.append(pos)
        except KeyError as e:
            pass

    def save_baseline(self):
        if len(self.trajectory_points) > 0:
            np.save('baseline.npy', np.array(self.trajectory_points))
            self.get_logger().info(f'✅ baseline.npy 저장 완료 (총 {len(self.trajectory_points)}개의 포인트)')
        else:
            self.get_logger().warning('⚠️ 저장할 데이터가 없습니다.')

def main(args=None):
    rclpy.init(args=args)
    node = BaselineRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()