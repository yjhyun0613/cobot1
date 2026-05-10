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

MAX_TILT_ANGLE = 25.0  

CENTER_COORD = [425.15, 74.76, 107.06] # 돔 맨 위 중심
LAYER_COORDS = [
    [411.66, 74.76, 103.48], # 1층 시작 좌표
    [402.95, 74.76,  99.45], # 2층 시작 좌표
    [394.48, 74.76,  94.08], # 3층 시작 좌표
    [384.26, 74.76,  81.16], # 4층 시작 좌표
]

# =================================================================
# [2] 기하학 연산 (안쪽 찌르기 + ori=2 전용 방사형 벡터 + 꼬임 방지)
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
    return rx, ry, rz

# =================================================================
# [3] Step 2: DRL 생성기 (반원 2분할 + 파이썬 반환 변수 추가)
# =================================================================
def generate_scan_drl():
    TOTAL_LAYERS = len(LAYER_COORDS)
    
    drl = f'set_tool("{ROBOT_TOOL}")\nset_tcp("{ROBOT_TCP}")\nset_singularity_handling(1)\n\n'
    drl += 'stiff = [300, 300, 500, 300, 300, 300]\n'
    drl += 'task_compliance_ctrl(stiff, 1)\n\n'
    
    last_rx, last_rz = None, None
    first_point = None  # 💡 1층 시작점 파이썬 전달용
    
    for i in range(TOTAL_LAYERS):
        layer_num = i + 1
        curr_z = LAYER_COORDS[i][2]
        R = abs(CENTER_COORD[0] - LAYER_COORDS[i][0])
        tilt_deg = MAX_TILT_ANGLE * (float(layer_num) / TOTAL_LAYERS)
        
        if layer_num % 2 == 1:
            angles = [-180.0, -90.0, 0.0, 90.0, 180.0] 
        else:
            angles = [180.0, 90.0, 0.0, -90.0, -180.0] 
            
        pts = []
        for j, ang in enumerate(angles):
            px = CENTER_COORD[0] + (R-1) * math.cos(math.radians(ang))
            py = CENTER_COORD[1] + (R-1) * math.sin(math.radians(ang))
            
            rx_raw, ry, rz_raw = get_forced_tilt_orientation(ang, tilt_deg)
            
            if last_rx is None:
                rx, rz = rx_raw, rz_raw
            else:
                rx = make_continuous(last_rx, rx_raw)
                rz = make_continuous(last_rz, rz_raw)
            last_rx, last_rz = rx, rz
            
            # 💡 기록 코드를 위해 1층의 첫 번째 지점을 파이썬에 저장
            if i == 0 and j == 0:
                first_point = [px, py, curr_z, rx, ry, rz]
                
            pts.append(f"posx({px:.2f}, {py:.2f}, {curr_z:.2f}, {rx:.2f}, {ry:.2f}, {rz:.2f})")

        drl += f"# Layer {layer_num} - R: {R:.2f}, Tilt: {tilt_deg:.2f}deg\n"
        
        for j in range(5):
            drl += f"p_{i}_{j} = {pts[j]}\n"
        
        drl += f"movel(p_{i}_0, v=40.0, a=80.0, r=0.0)\n"
        
        # 180도 반원 2개 분할 진행
        drl += f"movec(p_{i}_3, p_{i}_2, v=20.0, a=20.0, angle=180.0, r=10.0, ori=2)\n"
        drl += f"movec(p_{i}_1, p_{i}_4, v=20.0, a=20.0, angle=180.0, r=0.0, ori=2)\n\n"
        
    drl += "release_compliance_ctrl()\n"
    
    # 💡 [핵심] 기록 종료 시점과 맞추기 위해, 안전 고도로 상승하는 DRL 코드는 삭제!
    # 대신 파이썬에서 상승시킬 수 있도록 마지막 각도를 반환합니다.
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
        from DSR_ROBOT2 import movej, movel
        
        print(f"\n🚀 총 {len(LAYER_COORDS)}개의 층 스캔을 준비합니다...")
        scan_drl, first_point, last_rz = generate_scan_drl()
        
        # 1. 안전 고도 대기
        p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        p_approach = [CENTER_COORD[0], CENTER_COORD[1], CENTER_COORD[2] + 100.0, 0.0, 180.0, -180.0]
        movej(p_ready, vel=100.0, acc=100.0)
        movel(p_approach, vel=50.0, acc=100.0)
        time.sleep(0.5) 
        
        # =========================================================
        # 💡 [타이밍 동기화 로직 시작]
        # =========================================================
        # 2. 파이썬 제어로 1층 시작점까지 미리 이동합니다. (이때는 기록 안됨)
        print("\n📍 1층 스캔 시작점으로 진입합니다...")
        movel(first_point, vel=40.0, acc=80.0)
        time.sleep(0.5) # 로봇이 완벽하게 멈출 때까지 대기
        
        # 3. 목표 지점 도착 완료! 기록 노드에 ON(True) 신호를 보냅니다.
        print("🟢 궤적 시작점 도착! 데이터 기록을 시작합니다.")
        self.state_pub.publish(Bool(data=True)) 
        
        # 4. 즉시 스캐닝 DRL을 실행하여 원 궤적 돌입!
        print("🛸 원형 스캐닝 진행 중...")
        self.cli.call_async(DrlStart.Request(code=scan_drl))
        
        # 5. 스캔(마지막 층)이 끝날 때까지 대기합니다.
        self.wait_for_robot_stop(delay=3.0)
            
        # 6. 스캔이 끝나자마자 즉각 OFF(False) 신호를 보내 쓰레기 데이터를 방지합니다.
        self.state_pub.publish(Bool(data=False)) 
        print("🔴 궤적 종료! 데이터 기록을 중지했습니다.")
        
        # 7. 기록이 안전하게 끝난 후, 비로소 파이썬 제어로 로봇 팔을 들어 올립니다.
        print("⬆️ 안전 고도로 로봇을 복귀시킵니다...")
        p_up = [CENTER_COORD[0], CENTER_COORD[1], CENTER_COORD[2] + 100.0, 0.0, 180.0, last_rz]
        movel(p_up, vel=50.0, acc=100.0)
        # =========================================================

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