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
# [1] 환경 설정 (Configuration)
# =================================================================
ROBOT_ID    = "dsr01" 
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"

TOTAL_LAYERS = 3               
CENTER_EXP   = np.array([424.15, 75.45, 25.50]) # X, Y 중심과 바닥 Z 높이
FLOOR_Z      = 25.50           

TOOL_LENGTH  = 30 # 장착된 툴 길이 (30mm)

# =================================================================
# [2] 기하학 연산 함수 (글로벌 투영법 + 2:1 비율 하이브리드 자세)
# =================================================================
def rot_to_zyz(R):
    val = max(min(R[2, 2], 1.0), -1.0)
    beta = math.acos(val)
    
    if abs(val - 1.0) < 1e-6: 
        alpha = 0.0
        gamma = math.atan2(R[1, 0], R[0, 0])
    elif abs(val + 1.0) < 1e-6: 
        alpha = 0.0
        gamma = math.atan2(-R[1, 0], -R[0, 0])
    else: 
        alpha = math.atan2(R[1, 2], R[0, 2])
        gamma = math.atan2(R[2, 1], -R[2, 0])
        
    return [math.degrees(alpha), math.degrees(beta), math.degrees(gamma)]

def make_continuous(base, target):
    diff = (target - base) % 360.0
    if diff > 180.0: diff -= 360.0
    return base + diff

def get_dome_orientation(target_pos, center, tool_length=0.0):
    normal = target_pos - center
    n_norm = np.linalg.norm(normal)
    if n_norm < 1e-6:
        normal = np.array([0.0, 0.0, 1.0])
    else:
        normal = normal / n_norm
        
    # 1. 툴의 Z축 결정 (2:1 비율로 적당히 기울여 돔 바라보기)
    Z_t_pure = -normal 
    Z_t_down = np.array([0.0, 0.0, -1.0]) 
    Z_t_blended = Z_t_pure * 1.0 + Z_t_down * 2.0
    Z_t = Z_t_blended / np.linalg.norm(Z_t_blended)
    
    # 💡 [핵심 해결 로직] 글로벌 벡터 투영법!
    # 툴의 Y축이 원의 진행방향을 따라 돌지 않고, 항상 글로벌 Y축(한 방향)을 바라보게 합니다.
    # 이렇게 하면 6축 손목 관절이 절대 꼬이지 않아 방향 전환(CW<->CCW)이 완벽하게 이루어집니다.
    global_y = np.array([0.0, 1.0, 0.0])
    
    # X축은 Z축과 글로벌 Y축에 수직인 방향
    X_t = np.cross(global_y, Z_t)
    
    if np.linalg.norm(X_t) < 1e-6:
        # 혹시라도 Z축이 글로벌 Y축과 일치하는 예외 상황 방어
        global_x = np.array([1.0, 0.0, 0.0])
        Y_t = np.cross(Z_t, global_x)
        Y_t = Y_t / np.linalg.norm(Y_t)
        X_t = np.cross(Y_t, Z_t)
    else:
        X_t = X_t / np.linalg.norm(X_t)
        Y_t = np.cross(Z_t, X_t)
        
    R = np.column_stack((X_t, Y_t, Z_t))
    rx, ry, rz = rot_to_zyz(R)
    
    offset_pos = target_pos - Z_t * tool_length
    return rx, ry, rz, offset_pos

# =================================================================
# [3] Step 2: 스캐닝 전용 DRL 생성기
# =================================================================
def generate_scan_drl(center, radius, total_layers, tool_length):
    cx, cy, cz = center
    true_center = np.array([cx, cy, FLOOR_Z]) 
    
    drl_code = 'set_tool("Tool Weight")\nset_tcp("GripperDA_v1")\nset_singularity_handling(1)\n\n'
    
    drl_code += 'stiff = [3000, 3000, 50, 300, 300, 300]\ntask_compliance_ctrl(stiff, 1)\n'
    drl_code += 'set_desired_force([0,0,-10,0,0,0], [0,0,1,0,0,0], 0)\nsleep(0.5)\n\n'

    last_rx = None
    last_rz = None

    for i in range(1, total_layers + 1):
        angle_rad = math.radians(90.0 * (1.0 - float(i)/total_layers))
        curr_z = FLOOR_Z + radius * math.sin(angle_rad)
        curr_r = radius * math.cos(angle_rad)
        
        # 항상 동일한 시작점(왼쪽)에서 출발하여 한 바퀴를 그립니다.
        p_left  = np.array([cx - curr_r, cy, curr_z])
        p_top   = np.array([cx, cy + curr_r, curr_z])
        p_right = np.array([cx + curr_r, cy, curr_z])
        p_bot   = np.array([cx, cy - curr_r, curr_z])

        if i % 2 == 1:
            pts = [p_left, p_top, p_right, p_bot, p_left]
            direction = "CW"
        else:
            pts = [p_left, p_bot, p_right, p_top, p_left]
            direction = "CCW"

        ro_pts = []
        for pt in pts:
            rx_raw, ry, rz_raw, pos = get_dome_orientation(pt, true_center, tool_length)
            
            # 오일러 각도의 미세한 튐 현상만 잡아줍니다. (손목은 이미 안 꼬임)
            if last_rx is None:
                rx, rz = rx_raw, rz_raw
            else:
                rx = make_continuous(last_rx, rx_raw)
                rz = make_continuous(last_rz, rz_raw)
                
            last_rx, last_rz = rx, rz
            ro_pts.append((rx, ry, rz, pos))

        vel = 20.0 if i == 1 else 40.0
        drl_code += f"# Layer {i} - 1 Full Circle ({direction})\n"

        for idx in range(5):
            rx, ry, rz, pos = ro_pts[idx]
            drl_code += f"p_{i}_{idx} = posx({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {rx:.2f}, {ry:.2f}, {rz:.2f})\n"

        drl_code += f"movel(p_{i}_0, v={vel}, a=80.0)\n"
        # 좌표 이동 순서(위쪽 경유 vs 아래쪽 경유)에 따라 로봇은 물리적으로 반대로 돌 수밖에 없습니다!
        drl_code += f"movec(p_{i}_1, p_{i}_2, v=15.0, a=20.0)\n"
        drl_code += f"movec(p_{i}_3, p_{i}_4, v=15.0, a=20.0)\n\n"

    approach_z = FLOOR_Z + radius + 50.0
    drl_code += "release_force()\nrelease_compliance_ctrl()\n"
    
    a_rx, a_ry, a_rz, a_pos = get_dome_orientation(np.array([cx, cy, approach_z]), true_center, tool_length)
    a_rx = make_continuous(last_rx, a_rx)
    a_rz = make_continuous(last_rz, a_rz)
    
    drl_code += f"p_approach = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, {a_pos[2]:.2f}, {a_rx:.2f}, {a_ry:.2f}, {a_rz:.2f})\n"
    drl_code += "movel(p_approach, v=50.0, a=100.0)\n"
    
    return drl_code

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
        from DSR_ROBOT2 import (
            movej, movel, task_compliance_ctrl, set_desired_force, release_force,
            get_current_posx, release_compliance_ctrl
        )
        
        print("\n🚀 [Phase 1] 돔 최고점 자동 탐색 (Base 좌표계 힘 제어 하강)...")
        
        cx, cy, cz = CENTER_EXP
        approach_z = FLOOR_Z + 150.0
        
        a_rx, a_ry, a_rz, a_pos = get_dome_orientation(np.array([cx, cy, approach_z]), CENTER_EXP, TOOL_LENGTH)
        
        p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        p_approach = [a_pos[0], a_pos[1], a_pos[2], a_rx, a_ry, a_rz]

        movej(p_ready, vel=100.0, acc=100.0)
        movel(p_approach, vel=50.0, acc=100.0)
        time.sleep(1.0) 
        
        task_compliance_ctrl([3000, 3000, 100, 300, 300, 300], 0)
        time.sleep(0.5)
        
        print("⏬ 스르륵 하강 시작...")
        set_desired_force([0, 0, -15, 0, 0, 0], [0, 0, 1, 0, 0, 0], 0)
        time.sleep(1.0) 
        
        start_z = get_current_posx()[0][2]
        prev_z = start_z
        stop_count = 0 
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.1)
            
            curr_z = get_current_posx()[0][2]
            
            if abs(start_z - curr_z) < 3.0:
                prev_z = curr_z
                continue 
                
            if abs(prev_z - curr_z) < 0.2:
                stop_count += 1
            else:
                stop_count = 0 
                
            if stop_count >= 3:
                release_force() 
                stop_msg = RobotStop()
                stop_msg.stop_mode = 2 
                self.stop_pub.publish(stop_msg)
                print(f"🛑 돔 표면 감지 확정! (최종 하강 높이: {curr_z:.2f}mm)")
                break
                
            prev_z = curr_z
            
        release_compliance_ctrl()
        time.sleep(1.0) 
        
        pos_touch = get_current_posx()[0]
        flange_z = pos_touch[2] 
        true_top_z = flange_z - TOOL_LENGTH 
        measured_radius = true_top_z - FLOOR_Z
        
        print("-" * 45)
        print(f"🎯 돔 크기 측정 완료!")
        print(f"   - 로봇 플랜지 멈춘 높이 : {flange_z:.2f} mm")
        print(f"   - 툴 길이 반영 돔 표면 : {true_top_z:.2f} mm")
        print(f"   👉 자동 계산된 반경(R)  : {measured_radius:.2f} mm")
        print("-" * 45)

        print("\n🚀 [Phase 2] 스캐닝 DRL 구동 시작 (손목 고정 완벽 방향전환)...")
        self.state_pub.publish(Bool(data=True)) 
        
        scan_drl = generate_scan_drl(CENTER_EXP, measured_radius, TOTAL_LAYERS, TOOL_LENGTH)
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