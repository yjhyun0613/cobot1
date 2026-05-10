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

# 💡 [핵심] 미끄러짐 이탈을 막기 위해 틸트 각도를 바닥 경사에 맞게 80도로 상향!
MAX_TILT_ANGLE = 30.0  

# 💡 [핵심] 수동 순응 제어: 목표 궤적을 돔 표면 안쪽으로 2mm 파고들게 만듦
PRESS_OFFSET = 2.0 

CENTER_COORD = [423.15, 74.76, 106.06] # 돔 맨 위 중심
LAYER_COORDS = [
    [411.66, 74.76, 103.48], # 1층 시작 좌표
    [402.95, 74.76,  99.45], # 2층 시작 좌표
    [394.48, 74.76,  94.08], # 3층 시작 좌표
    [384.26, 74.76,  81.16], # 4층 시작 좌표
    [378.89, 74.76,  70.00], # 5층 시작 
    [374.40, 74.76,  62.00], # 6층 시작 
    [374.15, 74.76,  58.06], # 7층 시작 
]

# =================================================================
# [2] 기하학 연산 (안쪽 찌르기 + ori=2 전용 방사형 벡터)
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
    
    # Z_t는 툴이 찌르는(바라보는) 안쪽 방향 벡터
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
    
    return rx, ry, rz, Z_t # 파고들기 연산을 위해 Z_t 벡터 반환

# =================================================================
# [3] Step 2: DRL 생성기 
# =================================================================
def generate_scan_drl():
    TOTAL_LAYERS = len(LAYER_COORDS)
    
    drl = f'set_tool("{ROBOT_TOOL}")\nset_tcp("{ROBOT_TCP}")\nset_singularity_handling(1)\n\n'
    
    # 💡 X, Y 방향(궤적)은 단단하게(3000) 잡아 경로 이탈을 막고, Z(누르는 방향)만 300으로 스펀지 처리
    drl += 'stiff = [3000, 3000, 300, 300, 300, 300]\n'
    drl += 'task_compliance_ctrl(stiff, 1)\n\n'
    
    last_rx, last_rz = None, None
    first_point = None  
    
    for i in range(TOTAL_LAYERS):
        layer_num = i + 1
        curr_z = LAYER_COORDS[i][2]
        R_actual = abs(CENTER_COORD[0] - LAYER_COORDS[i][0]) # 실제 표면 반경
        tilt_deg = MAX_TILT_ANGLE * (float(layer_num) / TOTAL_LAYERS)
        
        if layer_num % 2 == 1:
            angles = [-180.0, -90.0, 0.0, 90.0, 180.0] 
        else:
            angles = [180.0, 90.0, 0.0, -90.0, -180.0] 
            
        pts = []
        for j, ang in enumerate(angles):
            # 1. 일단 돔의 '실제 표면' 좌표를 구함
            px = CENTER_COORD[0] + R_actual * math.cos(math.radians(ang))
            py = CENTER_COORD[1] + R_actual * math.sin(math.radians(ang))
            pz = curr_z
            
            # 2. 자세(Ori) 계산 및 툴 방향 벡터(Z_t) 가져오기
            rx_raw, ry, rz_raw, Z_t = get_forced_tilt_orientation(ang, tilt_deg)
            
            # 3. 💡 [핵심] 툴이 바라보는 방향(Z_t)으로 4mm(PRESS_OFFSET) 강제 전진! (파고들기)
            px += Z_t[0] * PRESS_OFFSET
            py += Z_t[1] * PRESS_OFFSET
            pz += Z_t[2] * PRESS_OFFSET
            
            if last_rx is None:
                rx, rz = rx_raw, rz_raw
            else:
                rx = make_continuous(last_rx, rx_raw)
                rz = make_continuous(last_rz, rz_raw)
            last_rx, last_rz = rx, rz
            
            if i == 0 and j == 0:
                first_point = [px, py, pz, rx, ry, rz]
                
            pts.append(f"posx({px:.2f}, {py:.2f}, {pz:.2f}, {rx:.2f}, {ry:.2f}, {rz:.2f})")

        drl += f"# Layer {layer_num} - Target_R: {R_actual-PRESS_OFFSET:.2f}, Tilt: {tilt_deg:.2f}deg\n"
        
        for j in range(5):
            drl += f"p_{i}_{j} = {pts[j]}\n"
        
        drl += f"movel(p_{i}_0, v=40.0, a=80.0, r=0.0)\n"
        drl += f"movec(p_{i}_3, p_{i}_2, v=20.0, a=20.0, angle=180.0, r=10.0, ori=2)\n"
        drl += f"movec(p_{i}_1, p_{i}_4, v=20.0, a=20.0, angle=180.0, r=0.0, ori=2)\n\n"
        
    drl += "release_compliance_ctrl()\n"
    return drl, first_point, last_rz

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
        
        # 💡 task_compliance_ctrl 추가
        from DSR_ROBOT2 import movej, movel, get_robot_state, task_compliance_ctrl
        
        print(f"\n🚀 총 {len(LAYER_COORDS)}개의 층 스캔을 준비합니다...")
        scan_drl, first_point, last_rz = generate_scan_drl()
        
        p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        p_approach = [CENTER_COORD[0], CENTER_COORD[1], CENTER_COORD[2] + 100.0, 0.0, 180.0, -180.0]
        
        movej(p_ready, vel=100.0, acc=100.0)
        movel(p_approach, vel=50.0, acc=100.0)
        time.sleep(0.5) 
        
        # 💡 파이썬으로 1층 파고들기 지점으로 가기 전에 충돌 흡수용 스프링 활성화
        # print("\n🛡️ 하강 충돌 방지를 위해 파이썬에서 미리 순응 제어를 켭니다...")
        # task_compliance_ctrl([3000, 3000, 300, 300, 300, 300], 1)
        
        print("📍 1층 스캔 시작점(안쪽 4mm 파고든 지점)으로 진입합니다...")
        movel(first_point, vel=20.0, acc=40.0) # 속도를 늦춰서 스무스하게 안착
        time.sleep(0.5) 
        
        print("🟢 궤적 시작점 도착! 데이터 기록을 시작합니다.")
        self.state_pub.publish(Bool(data=True)) 
        
        print("🛸 원형 스캐닝 진행 중...")
        self.cli.call_async(DrlStart.Request(code=scan_drl))
        
        # =========================================================
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
        # =========================================================
            
        self.state_pub.publish(Bool(data=False)) 
        print("🔴 궤적 종료 (3초 정지 확인 완료)! 데이터 기록을 중지했습니다.")
        
        print("⬆️ 안전 고도 상승 후 Ready 자세로 복귀합니다...")
        p_up = [CENTER_COORD[0], CENTER_COORD[1], CENTER_COORD[2] + 100.0, 0.0, 180.0, last_rz]
        movel(p_up, vel=50.0, acc=100.0)
        
        movej(p_ready, vel=100.0, acc=100.0)

        self.prog_pub.publish(String(data="03"))
        self.busy = False
        print("🏁 스캔 및 리포트 저장 프로세스 전체 완료!")

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