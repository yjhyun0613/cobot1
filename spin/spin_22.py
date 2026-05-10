import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from dsr_msgs2.srv import DrlStart
import DR_init
import time
import math
import numpy as np

# =================================================================
# [1] 환경 설정 (Configuration)
# =================================================================
ROBOT_ID    = "dsr01" 
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"

REAL_RADIUS  = 45.0            # 실제 돔 반경
PATH_RADIUS  = 45.0            # 로봇이 따라갈 반경
TOTAL_LAYERS = 3               # 스캔 층수
CENTER_EXP   = np.array([424.15, 75.45, 25.50]) # 이론상 중심점

SAFE_Z_HEIGHT = 200.0          # 안전 대기 높이
TOOL_LENGTH   = 30             # 툴 길이 (30mm)

# =================================================================
# [2] 기하학 연산 함수
# =================================================================
def normalize(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-6 else v

def rot_to_zyz(R):
    beta = math.acos(max(min(R[2, 2], 1.0), -1.0))
    if abs(beta) < 1e-6:
        alpha, gamma = 0.0, math.atan2(R[1, 0], R[0, 0])
    else:
        alpha, gamma = math.atan2(R[1, 2], R[0, 2]), math.atan2(R[2, 1], -R[2, 0])
    return [math.degrees(alpha), math.degrees(beta), math.degrees(gamma)]

def wrap_angle(angle):
    while angle > 180.0: angle -= 360.0
    while angle < -180.0: angle += 360.0
    return angle

def make_continuous(base, target):
    diff = (target - base) % 360.0
    if diff > 180.0: diff -= 360.0
    return base + diff

def get_pure_blended_orientation(target_pos, center, tool_length=0.0):
    pure_normal = -normalize(target_pos - center)
    down_z_axis = np.array([0.0, 0.0, -1.0])
    z_axis_final = normalize((down_z_axis * 2.0) + (pure_normal * 1.0))
    up = np.array([0.0, 1.0, 0.0]) if abs(z_axis_final[2]) > 0.95 else np.array([0.0, 0.0, 1.0])
    x_axis = normalize(np.cross(up, z_axis_final))
    y_axis = normalize(np.cross(z_axis_final, x_axis))
    rx, ry, rz_raw = rot_to_zyz(np.column_stack((x_axis, y_axis, z_axis_final)))
    offset_pos = target_pos - (z_axis_final * tool_length)
    return wrap_angle(rx), wrap_angle(ry), rz_raw, offset_pos

# =================================================================
# [3] 완성형 DRL 스크립트 생성기 (Relative Path Anchoring)
# =================================================================
def generate_best_scan_drl(center, real_radius, path_radius, total_layers, tool_length):
    cx, cy, cz = center
    approach_z = cz + real_radius + 10.0
    approach_xyz = np.array([cx, cy, approach_z])
    a_rx, a_ry, _, a_pos = get_pure_blended_orientation(approach_xyz, center, tool_length)
    a_rz = -180.0
    top_exp_pos = a_pos - np.array([0, 0, 10.0]) # 이론상 꼭대기 좌표

    drl_code = 'set_tool("Tool Weight")\nset_tcp("GripperDA_v1")\nset_singularity_handling(1)\n\n'
    drl_code += 'p_ready = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)\nmovej(p_ready, v=100.0, a=100.0)\n\n'
    drl_code += f'p_top = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, 200.0, {a_rx:.2f}, {a_ry:.2f}, {a_rz:.2f})\n'
    drl_code += f'p_approach = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, {a_pos[2]:.2f}, {a_rx:.2f}, {a_ry:.2f}, {a_rz:.2f})\n'
    drl_code += 'movel(p_top, v=100.0, a=100.0)\nmovel(p_approach, v=50.0, a=100.0)\nsleep(1.0)\n\n'
    drl_code += 'base_f = get_tool_force(1)\nzero_tz = base_f[2]\n'
    drl_code += 'stiff = [3000, 3000, 100, 300, 300, 300]\ntask_compliance_ctrl(stiff, 1)\n\n'
    drl_code += 'set_desired_force([0,0,-15,0,0,0], [0,0,1,0,0,0], 1)\n'
    drl_code += 'while True:\n    if abs(get_tool_force(1)[2] - zero_tz) > 6.0: break\n    sleep(0.001)\n\n'
    drl_code += 'pos_touch, sol = get_current_posx()\nrelease_force()\n'
    drl_code += 'set_desired_force([0,0,-10,0,0,0], [0,0,1,0,0,0], 1)\n\n'

    for i in range(1, total_layers + 1):
        angle_rad = math.radians(90.0 * (1.0 - float(i)/total_layers))
        curr_z, curr_r = cz + path_radius * math.sin(angle_rad), path_radius * math.cos(angle_rad)
        s_xyz, t_xyz = np.array([cx - curr_r, cy, curr_z]), np.array([cx + curr_r, cy, curr_z])
        v_xyz = np.array([cx, cy + curr_r, curr_z]) if i%2==1 else np.array([cx, cy - curr_r, curr_z])
        
        rx_s, ry_s, rz_s_raw, p_s = get_pure_blended_orientation(s_xyz, center, tool_length)
        rx_v, ry_v, rz_v_raw, p_v = get_pure_blended_orientation(v_xyz, center, tool_length)
        rx_t, ry_t, rz_t_raw, p_t = get_pure_blended_orientation(t_xyz, center, tool_length)
        
        rz_s = -180.0 if i%2==1 else 180.0
        rz_v, rz_t = make_continuous(rz_s, rz_v_raw), make_continuous(make_continuous(rz_s, rz_v_raw), rz_t_raw)

        # 💡 터치 지점(pos_touch) 기준으로 상대 거리 보정 적용
        drl_code += f"p_s_{i} = posx(pos_touch[0]+({p_s[0]-top_exp_pos[0]:.2f}), pos_touch[1]+({p_s[1]-top_exp_pos[1]:.2f}), pos_touch[2]+({p_s[2]-top_exp_pos[2]:.2f}), {rx_s:.2f}, {ry_s:.2f}, {rz_s:.2f})\n"
        drl_code += f"p_v_{i} = posx(pos_touch[0]+({p_v[0]-top_exp_pos[0]:.2f}), pos_touch[1]+({p_v[1]-top_exp_pos[1]:.2f}), pos_touch[2]+({p_v[2]-top_exp_pos[2]:.2f}), {rx_v:.2f}, {ry_v:.2f}, {rz_v:.2f})\n"
        drl_code += f"p_t_{i} = posx(pos_touch[0]+({p_t[0]-top_exp_pos[0]:.2f}), pos_touch[1]+({p_t[1]-top_exp_pos[1]:.2f}), pos_touch[2]+({p_t[2]-top_exp_pos[2]:.2f}), {rx_t:.2f}, {ry_t:.2f}, {rz_t:.2f})\n"
        drl_code += f"movel(p_s_{i}, v=20 if {i}==1 else 40, a=80, r=10)\nmovec(p_v_{i}, p_t_{i}, v=10, a=10, angle=360, ori=2)\n\n"

    drl_code += "release_force()\nrelease_compliance_ctrl()\nmovel(p_approach, v=50, a=100)\n"
    return drl_code

# =================================================================
# [4] 메인 제어 노드
# =================================================================
class ScanRunnerNode(Node):
    def __init__(self):
        super().__init__('final_scan_runner', namespace=ROBOT_ID)
        self.state_pub = self.create_publisher(Bool, f'/{ROBOT_ID}/checking_state', 10)
        self.prog_pub = self.create_publisher(String, f'/{ROBOT_ID}/progress_status', 10)
        self.trigger = False
        self.busy = False
        self.create_subscription(String, f'/{ROBOT_ID}/progress_status', self.cb, 10)
        self.cli = self.create_client(DrlStart, f'/{ROBOT_ID}/drl/drl_start')
        while not self.cli.wait_for_service(1.0): self.get_logger().info('대기중...')

    def cb(self, msg):
        if msg.data == "02" and not self.busy: self.trigger = True

    def run(self):
        self.busy = True
        self.trigger = False
        drl = generate_best_scan_drl(CENTER_EXP, REAL_RADIUS, PATH_RADIUS, TOTAL_LAYERS, TOOL_LENGTH)
        self.cli.call_async(DrlStart.Request(code=drl))
        
        # 바닥 안착 감지 루프
        from DSR_ROBOT2 import get_current_posx, get_robot_state
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if abs(get_current_posx()[0][0] - CENTER_EXP[0]) > 5.0:
                self.state_pub.publish(Bool(data=True))
                print("🛸 스캔 기록 시작")
                break
        
        # 완료 대기
        stop_time = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if get_robot_state() == 1:
                stop_time += 0.1
                if stop_time > 3.0: break
            else: stop_time = 0
            
        self.state_pub.publish(Bool(data=False))
        self.prog_pub.publish(String(data="03"))
        self.busy = False
        print("🏁 작업 완료")

def main():
    DR_init.__dsr__id, DR_init.__dsr__model = ROBOT_ID, ROBOT_MODEL
    rclpy.init()
    node = ScanRunnerNode()
    DR_init.__dsr__node = node
    
    from DSR_ROBOT2 import set_robot_mode, set_tool, set_tcp
    set_robot_mode(0); set_tool(ROBOT_TOOL); set_tcp(ROBOT_TCP); set_robot_mode(1)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.trigger: node.run()
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()