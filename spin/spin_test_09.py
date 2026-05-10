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
# [1] 환경 설정 및 유저 실측 좌표 데이터
# =================================================================
ROBOT_ID    = "dsr01" 
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"

MAX_TILT_ANGLE = 40.0  

# 💡 [핵심 파라미터]
PRESS_OFFSET = 2.0   # 돔 안으로 파고드는 깊이 (누름 힘 발생)
HOVER_OFFSET = -20.0 # 돔 밖으로 띄우는 안전 거리 (음수 = 밖으로)

CENTER_COORD = [423.15, 74.76, 106.06] # 돔 맨 위 중심
LAYER_COORDS = [
    [411.66, 74.76, 103.48], # 1층 
    [402.95, 74.76,  99.45], # 2층 
    [394.48, 74.76,  94.08], # 3층 
    [386.26, 74.76,  81.16], # 4층 
    [383.89, 74.76,  73.00], # 5층 
    [374.40, 74.76,  67.00], # 6층 
    [374.15, 74.76,  58.06], # 7층 
]

# =================================================================
# [2] 기하학 연산 
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

def get_forced_tilt_orientation(angle_xy_deg, tilt_deg):
    th_rad = math.radians(angle_xy_deg)
    tilt_rad = math.radians(tilt_deg)
    
    # 툴이 돔 안쪽을 찌르는 벡터
    Z_t = np.array([
        -math.sin(tilt_rad) * math.cos(th_rad),
        -math.sin(tilt_rad) * math.sin(th_rad),
        -math.cos(tilt_rad)
    ])
    
    Y_t = np.array([-math.sin(th_rad), math.cos(th_rad), 0.0])
    
    X_t = np.cross(Y_t, Z_t)
    X_t = X_t / np.linalg.norm(X_t)
    Y_t = np.cross(Z_t, X_t)
        
    R_mat = np.column_stack((X_t, Y_t, Z_t))
    rx, ry, rz = rot_to_zyz(R_mat)
    
    return rx, ry, rz, Z_t 

# =================================================================
# [3] Step 2: DRL 생성기 (Lift & Reposition 로직 추가)
# =================================================================
def generate_scan_drl():
    TOTAL_LAYERS = len(LAYER_COORDS)
    
    drl = f'set_tool("{ROBOT_TOOL}")\nset_tcp("{ROBOT_TCP}")\nset_singularity_handling(1)\n\n'
    drl += 'stiff = [3000, 3000, 300, 300, 300, 300]\n'
    drl += 'task_compliance_ctrl(stiff, 1)\n\n'
    
    last_rx, last_rz = None, None
    
    for i in range(TOTAL_LAYERS):
        layer_num = i + 1
        curr_z = LAYER_COORDS[i][2]
        R_actual = abs(CENTER_COORD[0] - LAYER_COORDS[i][0])
        tilt_deg = MAX_TILT_ANGLE * (float(layer_num) / TOTAL_LAYERS)
        
        if layer_num % 2 == 1:
            angles = [-180.0, -90.0, 0.0, 90.0, 180.0] 
        else:
            angles = [180.0, 90.0, 0.0, -90.0, -180.0] 
            
        pts_press = []
        pts_safe = []
        
        for j, ang in enumerate(angles):
            px = CENTER_COORD[0] + R_actual * math.cos(math.radians(ang))
            py = CENTER_COORD[1] + R_actual * math.sin(math.radians(ang))
            pz = curr_z
            
            rx_raw, ry, rz_raw, Z_t = get_forced_tilt_orientation(ang, tilt_deg)
            
            # 💡 1. 허공에 띄운 안전 좌표 (Hover)
            px_safe = px + Z_t[0] * HOVER_OFFSET
            py_safe = py + Z_t[1] * HOVER_OFFSET
            pz_safe = pz + Z_t[2] * HOVER_OFFSET
            
            # 💡 2. 돔 안으로 파고든 밀착 좌표 (Press)
            px_press = px + Z_t[0] * PRESS_OFFSET
            py_press = py + Z_t[1] * PRESS_OFFSET
            pz_press = pz + Z_t[2] * PRESS_OFFSET
            
            if last_rx is None:
                rx, rz = rx_raw, rz_raw
            else:
                rx = make_continuous(last_rx, rx_raw)
                rz = make_continuous(last_rz, rz_raw)
            last_rx, last_rz = rx, rz
            
            pts_press.append(f"posx({px_press:.2f}, {py_press:.2f}, {pz_press:.2f}, {rx:.2f}, {ry:.2f}, {rz:.2f})")
            pts_safe.append(f"posx({px_safe:.2f}, {py_safe:.2f}, {pz_safe:.2f}, {rx:.2f}, {ry:.2f}, {rz:.2f})")

        drl += f"# Layer {layer_num} - Lift & Scan\n"
        
        # 시작점(0번)과 끝점(4번)의 Safe 좌표 변수 할당
        drl += f"p_{i}_safe_start = {pts_safe[0]}\n"
        drl += f"p_{i}_safe_end = {pts_safe[4]}\n"
        
        for j in range(5):
            drl += f"p_{i}_{j} = {pts_press[j]}\n"
        
        # 💡 [새로운 주행 시퀀스]
        # 1. 띄워진 안전 위치로 빠르게 이동 (허공 이동)
        drl += f"movel(p_{i}_safe_start, v=50.0, a=100.0, r=0.0)\n"
        
        # 2. 돔 표면으로 파고들며 스무스하게 찌르기 (안착)
        drl += f"movel(p_{i}_0, v=20.0, a=40.0, r=0.0)\n"
        
        # 3. 돔 표면 스캔 (💡 인덱스 꼬임 버그 수정: 0 -> 1,2 -> 3,4 순서로 완벽한 반원 그림)
        drl += f"movec(p_{i}_3, p_{i}_2, v=20.0, a=20.0, angle=180.0, r=5.0, ori=2)\n"
        drl += f"movec(p_{i}_1, p_{i}_4, v=20.0, a=20.0, angle=180.0, r=0.0, ori=2)\n"
        
        # 4. 스캔 끝나면 즉시 표면에서 수직으로 띄우기 (압력 해제)
        drl += f"movel(p_{i}_safe_end, v=40.0, a=80.0, r=0.0)\n\n"
        
    drl += "release_compliance_ctrl()\n"
    return drl, last_rz

# =================================================================
# [4] 메인 제어 노드
# =================================================================
class ScanRunnerNode(Node):
    def __init__(self):
        super().__init__('final_scan_runner', namespace=ROBOT_ID)
        self.state_pub = self.create_publisher(Bool, f'/{ROBOT_ID}/checking_state', 10)
        self.prog_pub = self.create_publisher(String, f'/{ROBOT_ID}/progress_status', 10)
        self.stop_pub = self.create_publisher(RobotStop, f'/{ROBOT_ID}/stop', 10)
        
        self.trigger = False
        self.busy = False
        self.create_subscription(String, f'/{ROBOT_ID}/progress_status', self.cb, 10)
        self.cli = self.create_client(DrlStart, f'/{ROBOT_ID}/drl/drl_start')
        while not self.cli.wait_for_service(1.0): self.get_logger().info('제어기 서비스 대기중...')

    def cb(self, msg):
        if msg.data == "02" and not self.busy: self.trigger = True

    def wait_for_robot_stop(self, delay=2.0):
        from DSR_ROBOT2 import get_robot_state
        time.sleep(delay) 
        stop_time = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            state = get_robot_state()
            if state == 1: 
                stop_time += 0.1
                if stop_time > 1.0: break 
            else: 
                stop_time = 0

    def run(self):
        self.busy = True
        self.trigger = False
        
        from DSR_ROBOT2 import movej, movel, get_robot_state, task_compliance_ctrl
        
        print(f"\n🚀 총 {len(LAYER_COORDS)}개의 층 스캔을 준비합니다 (Lift & Reposition 모드)...")
        scan_drl, last_rz = generate_scan_drl()
        
        p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        p_approach = [CENTER_COORD[0], CENTER_COORD[1], CENTER_COORD[2] + 100.0, 0.0, 180.0, -180.0]
        
        movej(p_ready, vel=100.0, acc=100.0)
        movel(p_approach, vel=50.0, acc=100.0)
        time.sleep(0.5) 
        
        # 파이썬 단계에서 복잡하게 이동할 필요 없이, 바로 스프링 켜고 DRL로 위임!
        # print("\n🛡️ 하강 충돌 방지를 위해 순응 제어를 켜고 DRL을 시작합니다.")
        # task_compliance_ctrl([3000, 3000, 300, 300, 300, 300], 1)
        
        print("🟢 궤적 시작! 데이터 기록을 켭니다.")
        self.state_pub.publish(Bool(data=True)) 
        
        self.cli.call_async(DrlStart.Request(code=scan_drl))
        
        time.sleep(1.0) 
        stop_count = 0
        required_stop_time = 3.0 
        check_interval = 0.5
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            state = get_robot_state()
            
            if state == 1: 
                stop_count += check_interval
                if stop_count >= required_stop_time:
                    break
            else:
                stop_count = 0 
            
            time.sleep(check_interval)
            
        self.state_pub.publish(Bool(data=False)) 
        print("🔴 궤적 종료 (3초 정지 확인 완료)! 데이터 기록을 중지했습니다.")
        
        print("⬆️ 안전 고도 상승 후 Ready 자세로 복귀합니다...")
        p_up = [CENTER_COORD[0], CENTER_COORD[1], CENTER_COORD[2] + 100.0, 0.0, 180.0, last_rz]
        movel(p_up, vel=50.0, acc=100.0)
        
        movej(p_ready, vel=100.0, acc=100.0)

        self.prog_pub.publish(String(data="03"))
        self.busy = False
        print("🏁 스캔 프로세스 전체 완료!")

def main():
    DR_init.__dsr__id, DR_init.__dsr__model = ROBOT_ID, ROBOT_MODEL
    rclpy.init()
    node = ScanRunnerNode()
    DR_init.__dsr__node = node
    
    from DSR_ROBOT2 import set_robot_mode, set_tool, set_tcp
    set_robot_mode(0); set_tool(ROBOT_TOOL); set_tcp(ROBOT_TCP); set_robot_mode(1)
    
    try:
        print("\n🔄 대기 모드: 신호(02) 대기 중...")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.trigger: node.run()
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()