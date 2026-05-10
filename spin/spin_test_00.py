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
# [1] 환경 설정 및 💡 유저 실측 좌표 데이터
# =================================================================
ROBOT_ID    = "dsr01" 
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1" # ⚠️ 티칭 펜던트에 Z=30mm가 반영된 TCP여야 합니다!

MAX_TILT_ANGLE = 30.0  # 최종 층에서 꺾일 최대 각도

# 💡 실측하신 데이터를 여기에 계속 추가하시면 코드가 알아서 층수를 늘립니다.
CENTER_COORD = [425.15, 74.76, 106.06] # 돔 맨 위 중심
LAYER_COORDS = [
    [411.66, 74.76, 103.48], # 1층 시작 좌표
    [402.95, 74.76,  99.45], # 2층 시작 좌표
    [394.48, 74.76,  94.08], # 3층 시작 좌표
    [384.26, 74.76,  81.16], # 4층 시작 좌표
    # [370.00, 74.76,  70.00]  <-- 이런 식으로 계속 추가 가능!
]

# =================================================================
# [2] 기하학 연산 함수 (균등 기울기 + 손목 꼬임 방지)
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
    
    # 목표 기울기만큼 정확하게 바깥으로 꺾인 툴 방향(Z축) 계산
    Z_t = np.array([
        -math.sin(tilt_rad) * math.cos(th_rad),
        -math.sin(tilt_rad) * math.sin(th_rad),
        -math.cos(tilt_rad)
    ])
    
    # 손목 꼬임을 막기 위해 툴 몸체(Y축)를 항상 글로벌 기준에 맞춤
    global_y = np.array([0.0, 1.0, 0.0])
    X_t = np.cross(global_y, Z_t)
    
    if np.linalg.norm(X_t) < 1e-6:
        global_x = np.array([1.0, 0.0, 0.0])
        Y_t = np.cross(Z_t, global_x)
        Y_t = Y_t / np.linalg.norm(Y_t)
        X_t = np.cross(Y_t, Z_t)
    else:
        X_t = X_t / np.linalg.norm(X_t)
        Y_t = np.cross(Z_t, X_t)
        
    R_mat = np.column_stack((X_t, Y_t, Z_t))
    rx, ry, rz = rot_to_zyz(R_mat)
    return rx, ry, rz

# =================================================================
# [3] Step 2: DRL 생성기 (실측 좌표 기반 36분할 블렌딩)
# =================================================================
def generate_scan_drl():
    TOTAL_LAYERS = len(LAYER_COORDS) # 💡 리스트 길이에 따라 자동으로 층수 결정!
    
    drl = f'set_tool("{ROBOT_TOOL}")\nset_tcp("{ROBOT_TCP}")\nset_singularity_handling(1)\n\n'
    last_rx, last_rz = None, None
    
    for i in range(TOTAL_LAYERS):
        layer_num = i + 1
        curr_z = LAYER_COORDS[i][2]
        
        # 💡 각 층의 X 좌표를 중심 X와 비교하여 실제 회전 반경 계산
        R = abs(CENTER_COORD[0] - LAYER_COORDS[i][0])
        
        # 💡 균등한 기울기 분배 로직
        tilt_deg = MAX_TILT_ANGLE * (float(layer_num) / TOTAL_LAYERS)
        
        # 실측 좌표가 모두 중심보다 X가 작으므로 시작 각도는 180도 (왼쪽)
        start_angle = 180.0
        
        # 💡 지그재그 회전 방향 결정
        if layer_num % 2 == 1:
            direction = "CW"
            step_angle = -10.0 # 시계 방향은 각도가 줄어듦
        else:
            direction = "CCW"
            step_angle = 10.0  # 반시계 방향은 각도가 늘어남
            
        drl += f"# Layer {layer_num}/{TOTAL_LAYERS} - R: {R:.2f}, Z: {curr_z:.2f}, Tilt: {tilt_deg:.2f}deg ({direction})\n"
        
        # 원을 36개의 조각(10도 간격)으로 잘게 쪼갬
        steps = 36
        for k in range(steps + 1):
            curr_angle = start_angle + k * step_angle
            
            # 수학적으로 계산된 완벽한 3D 원형 궤적 좌표
            px = CENTER_COORD[0] + R * math.cos(math.radians(curr_angle))
            py = CENTER_COORD[1] + R * math.sin(math.radians(curr_angle))
            pz = curr_z
            
            # 특이점 없는 완벽한 자세 계산
            rx_raw, ry, rz_raw = get_forced_tilt_orientation(curr_angle, tilt_deg)
            
            if last_rx is None:
                rx, rz = rx_raw, rz_raw
            else:
                rx = make_continuous(last_rx, rx_raw)
                rz = make_continuous(last_rz, rz_raw)
            last_rx, last_rz = rx, rz
            
            drl += f"p_t = posx({px:.2f}, {py:.2f}, {pz:.2f}, {rx:.2f}, {ry:.2f}, {rz:.2f})\n"
            
            # 💡 원주 구속을 위한 movel 블렌딩 기법
            if k == 0:
                drl += f"movel(p_t, v=40.0, a=80.0, r=0.0)\n" # 층 시작점 도착
            elif k == steps:
                drl += f"movel(p_t, v=20.0, a=20.0, r=0.0)\n" # 층 한 바퀴 끝
            else:
                drl += f"movel(p_t, v=20.0, a=20.0, r=5.0)\n" # 둥글게 이어 붙이기
                
        drl += "\n"
    
    # 스캔 종료 후 안전 높이로 복귀
    drl += f"p_up = posx({CENTER_COORD[0]:.2f}, {CENTER_COORD[1]:.2f}, {CENTER_COORD[2]+100.0:.2f}, 0.0, 180.0, 0.0)\n"
    drl += "movel(p_up, v=50.0, a=100.0)\n"
    
    return drl

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
        
        # 오작동을 유발하던 측정(힘 제어) 단계 완전 삭제 및 즉시 스캔 시작
        print(f"\n🚀 총 {len(LAYER_COORDS)}개의 층 스캔을 준비합니다...")
        
        # 안전한 허공으로 먼저 이동
        p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        p_approach = [CENTER_COORD[0], CENTER_COORD[1], CENTER_COORD[2] + 100.0, 0.0, 180.0, 0.0]

        movej(p_ready, vel=100.0, acc=100.0)
        movel(p_approach, vel=50.0, acc=100.0)
        time.sleep(1.0) 
        
        print("\n🚀 실측 좌표 기반 스캐닝 DRL 구동 시작...")
        self.state_pub.publish(Bool(data=True)) 
        
        scan_drl = generate_scan_drl()
        self.cli.call_async(DrlStart.Request(code=scan_drl))
        
        self.wait_for_robot_stop(delay=3.0)
            
        self.state_pub.publish(Bool(data=False)) 
        self.prog_pub.publish(String(data="03"))
        self.busy = False
        print("🏁 스캔 및 리포트 저장 완료!")

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