import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from dsr_msgs2.srv import DrlStart
from dsr_msgs2.msg import RobotStop
import DR_init
import time
import math
import numpy as np

# =================================================================
# [1] 환경 설정
# =================================================================
ROBOT_ID    = "dsr01" 
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1" # 그리퍼 끝단 기준 TCP

# 💡 [핵심] 그리퍼 끝에서부터 추가로 잡고 있는 툴의 길이
EXTRA_TOOL_LENGTH = 30.0  
PRESS_OFFSET      = 4.0   # 돔 안으로 파고드는 깊이 (압력 생성)
HOVER_OFFSET      = -20.0 # 돔 밖으로 띄우는 안전 거리

CENTER_COORD = [423.15, 74.76, 106.06] 
LAYER_COORDS = [
    [411.66, 74.76, 103.48], [402.95, 74.76,  99.45], 
    [394.48, 74.76,  94.08], [384.26, 74.76,  81.16], 
    [380.89, 74.76,  72.00], [378.40, 74.76,  64.00], 
    [378.15, 74.76,  60.06], 
]

# =================================================================
# [2] 기하학 연산 (그리퍼 끝단 좌표 산출용)
# =================================================================
def rot_to_zyz(R):
    val = max(min(R[2, 2], 1.0), -1.0)
    beta = math.acos(val)
    if abs(val - 1.0) < 1e-6: 
        alpha, gamma = 0.0, math.atan2(R[1, 0], R[0, 0])
    elif abs(val + 1.0) < 1e-6: 
        alpha, gamma = 0.0, math.atan2(-R[1, 0], -R[0, 0])
    else: 
        alpha, gamma = math.atan2(R[1, 2], R[0, 2]), math.atan2(R[2, 1], -R[2, 0])
    return [math.degrees(alpha), math.degrees(beta), math.degrees(gamma)]

def make_continuous(base, target):
    diff = (target - base) % 360.0
    if diff > 180.0: diff -= 360.0
    return base + diff

def get_tool_vector_orientation(angle_xy_deg, tilt_deg):
    th_rad = math.radians(angle_xy_deg)
    tilt_rad = math.radians(tilt_deg)
    Z_t = np.array([-math.sin(tilt_rad)*math.cos(th_rad), -math.sin(tilt_rad)*math.sin(th_rad), -math.cos(tilt_rad)])
    Y_t = np.array([-math.sin(th_rad), math.cos(th_rad), 0.0])
    X_t = np.cross(Y_t, Z_t)
    X_t /= np.linalg.norm(X_t)
    Y_t = np.cross(Z_t, X_t)
    rx, ry, rz = rot_to_zyz(np.column_stack((X_t, Y_t, Z_t)))
    return rx, ry, rz, Z_t 

# =================================================================
# [3] DRL 생성기 (그리퍼 끝단 좌표로 자동 변환)
# =================================================================
def generate_scan_drl():
    TOTAL_LAYERS = len(LAYER_COORDS)
    drl = f'set_tool("{ROBOT_TOOL}")\nset_tcp("{ROBOT_TCP}")\n\n'
    drl += 'stiff = [3000, 3000, 300, 300, 300, 300]\ntask_compliance_ctrl(stiff, 1)\n\n'
    
    last_rx, last_rz = None, None
    for i in range(TOTAL_LAYERS):
        layer_num, pz_base = i + 1, LAYER_COORDS[i][2]
        R_actual = abs(CENTER_COORD[0] - LAYER_COORDS[i][0])
        tilt_deg = 80.0 * (float(layer_num) / TOTAL_LAYERS) # 바닥까지 가기 위해 틸트 상향
        
        angles = [-180.0, -90.0, 0.0, 90.0, 180.0] if layer_num % 2 == 1 else [180.0, 90.0, 0.0, -90.0, -180.0]
        pts_press, pts_safe = [], []

        for ang in angles:
            px_surface = CENTER_COORD[0] + R_actual * math.cos(math.radians(ang))
            py_surface = CENTER_COORD[1] + R_actual * math.sin(math.radians(ang))
            rx, ry, rz_raw, Z_t = get_tool_vector_orientation(ang, tilt_deg)
            
            # 💡 [핵심 계산] 
            # 돔 표면에서 툴 길이만큼 뒤로 빼고(-), 파고들기 오프셋만큼 더함(+)
            total_offset_press = PRESS_OFFSET - EXTRA_TOOL_LENGTH
            total_offset_safe  = HOVER_OFFSET - EXTRA_TOOL_LENGTH
            
            p_press = [px_surface + Z_t[0]*total_offset_press, py_surface + Z_t[1]*total_offset_press, pz_base + Z_t[2]*total_offset_press]
            p_safe  = [px_surface + Z_t[0]*total_offset_safe, py_surface + Z_t[1]*total_offset_safe, pz_base + Z_t[2]*total_offset_safe]
            
            if last_rx is None: rx_final, rz_final = rx, rz_raw
            else:
                rx_final = make_continuous(last_rx, rx)
                rz_final = make_continuous(last_rz, rz_raw)
            last_rx, last_rz = rx_final, rz_final
            
            pts_press.append(f"posx({p_press[0]:.2f}, {p_press[1]:.2f}, {p_press[2]:.2f}, {rx_final:.2f}, {ry:.2f}, {rz_final:.2f})")
            pts_safe.append(f"posx({p_safe[0]:.2f}, {p_safe[1]:.2f}, {p_safe[2]:.2f}, {rx_final:.2f}, {ry:.2f}, {rz_final:.2f})")

        drl += f"# Layer {layer_num}\n"
        drl += f"p_{i}_safe_start = {pts_safe[0]}\np_{i}_safe_end = {pts_safe[4]}\n"
        for j in range(5): drl += f"p_{i}_{j} = {pts_press[j]}\n"
        drl += f"movel(p_{i}_safe_start, v=50, a=100)\nmovel(p_{i}_0, v=20, a=40)\n"
        drl += f"movec(p_{i}_1, p_{i}_2, v=20, a=20, angle=180, r=5, ori=2)\n"
        drl += f"movec(p_{i}_3, p_{i}_4, v=20, a=20, angle=180, r=0, ori=2)\n"
        drl += f"movel(p_{i}_safe_end, v=40, a=80)\n\n"
        
    drl += "release_compliance_ctrl()\n"
    return drl, last_rz

# =================================================================
# [4] 메인 노드
# =================================================================
class ScanRunnerNode(Node):
    def __init__(self):
        super().__init__('final_scan_runner', namespace=ROBOT_ID)
        self.state_pub = self.create_publisher(Bool, f'/{ROBOT_ID}/checking_state', 10)
        self.prog_pub = self.create_publisher(String, f'/{ROBOT_ID}/progress_status', 10)
        self.cli = self.create_client(DrlStart, f'/{ROBOT_ID}/drl/drl_start')
        self.trigger, self.busy = False, False
        self.create_subscription(String, f'/{ROBOT_ID}/progress_status', self.cb, 10)

    def cb(self, msg):
        if msg.data == "02" and not self.busy: self.trigger = True

    def run(self):
        self.busy, self.trigger = True, False
        from DSR_ROBOT2 import movej, movel, get_robot_state, task_compliance_ctrl
        scan_drl, last_rz = generate_scan_drl()
        p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        movej(p_ready, vel=100, acc=100)
        task_compliance_ctrl([3000, 3000, 300, 300, 300, 300], 1)
        
        print("🟢 데이터 기록 시작 및 스캔 수행")
        self.state_pub.publish(Bool(data=True)) 
        self.cli.call_async(DrlStart.Request(code=scan_drl))
        
        time.sleep(1.0)
        stop_count = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if get_robot_state() == 1:
                stop_count += 0.1
                if stop_count > 3.0: break 
            else: stop_count = 0
            
        self.state_pub.publish(Bool(data=False)) 
        movej(p_ready, vel=100, acc=100)
        self.prog_pub.publish(String(data="03"))
        self.busy = False
        print("🏁 완료")

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