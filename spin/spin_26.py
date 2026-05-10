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
# [3] Step 1: 💡 돔 최고점 탐색 전용 DRL 생성 (Z값 감지 로직 내장)
# =================================================================
def generate_measure_drl(center, tool_length):
    cx, cy, cz = center
    approach_z = FLOOR_Z + 150.0
    a_rx, a_ry, _, a_pos = get_pure_blended_orientation(np.array([cx, cy, approach_z]), center, tool_length)
    
    drl_code = 'set_tool("Tool Weight")\nset_tcp("GripperDA_v1")\nset_singularity_handling(1)\n\n'
    drl_code += 'p_ready = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)\nmovej(p_ready, v=100.0, a=100.0)\n\n'
    
    # 1. 중심 위 150mm 안전 높이로 이동
    drl_code += f'p_approach = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, {a_pos[2]:.2f}, {a_rx:.2f}, {a_ry:.2f}, -180.0)\n'
    drl_code += 'movel(p_approach, v=50.0, a=100.0)\nsleep(1.0)\n\n'
    
    # 2. Z축을 스프링처럼 부드럽게 세팅
    drl_code += 'task_compliance_ctrl([3000, 3000, 100, 300, 300, 300])\n'
    
    # 3. 바닥 높이를 목표로 15mm/s로 천천히 비동기 하강 (amovel)
    drl_code += f'p_target = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, {FLOOR_Z:.2f}, {a_rx:.2f}, {a_ry:.2f}, -180.0)\n'
    drl_code += 'amovel(p_target, v=15.0, a=30.0)\n'
    drl_code += 'sleep(0.5)\n\n' # 초기 출발 대기
    
    # 4. 💡 제어기 내부에서 매 0.1초마다 Z값 변화 감지 (사용자님 로직!)
    drl_code += 'while True:\n'
    drl_code += '    pos1, sol1 = get_current_posx()\n'
    drl_code += '    sleep(0.1)\n'
    drl_code += '    pos2, sol2 = get_current_posx()\n'
    drl_code += '    diff = abs(pos1[2] - pos2[2])\n'
    drl_code += '    if diff < 0.2:\n'        # 0.1초 동안 0.2mm도 못 갔으면 막힌 것!
    drl_code += '        stop(2)\n'           # 즉시 정지 (Quick Stop)
    drl_code += '        break\n\n'
    
    drl_code += 'release_compliance_ctrl()\n'
    return drl_code

# =================================================================
# [4] Step 2: 스캐닝 전용 DRL 생성기
# =================================================================
def generate_scan_drl(center, radius, total_layers, tool_length):
    cx, cy, cz = center
    true_center = np.array([cx, cy, FLOOR_Z]) 
    
    drl_code = 'set_tool("Tool Weight")\nset_tcp("GripperDA_v1")\nset_singularity_handling(1)\n\n'
    
    # 스캔 텐션 켜기 (DRL 내부에서는 힘 제어가 완벽하게 동작합니다)
    drl_code += 'stiff = [3000, 3000, 50, 300, 300, 300]\ntask_compliance_ctrl(stiff, 1)\n'
    drl_code += 'set_desired_force([0,0,-10,0,0,0], [0,0,1,0,0,0], 1)\nsleep(0.5)\n\n'

    last_rz = -180.0 

    for i in range(1, total_layers + 1):
        angle_rad = math.radians(90.0 * (1.0 - float(i)/total_layers))
        curr_z = FLOOR_Z + radius * math.sin(angle_rad)
        curr_r = radius * math.cos(angle_rad)
        
        if i % 2 == 1:
            s_xyz = np.array([cx - curr_r, cy, curr_z])
            v_xyz = np.array([cx, cy + curr_r, curr_z])
            t_xyz = np.array([cx + curr_r, cy, curr_z])
            direction = "CW"
        else:
            s_xyz = np.array([cx + curr_r, cy, curr_z]) 
            v_xyz = np.array([cx, cy - curr_r, curr_z]) 
            t_xyz = np.array([cx - curr_r, cy, curr_z]) 
            direction = "CCW"
        
        rx_s, ry_s, rz_s_raw, p_s = get_pure_blended_orientation(s_xyz, true_center, tool_length)
        rx_v, ry_v, rz_v_raw, p_v = get_pure_blended_orientation(v_xyz, true_center, tool_length)
        rx_t, ry_t, rz_t_raw, p_t = get_pure_blended_orientation(t_xyz, true_center, tool_length)
        
        rz_s = make_continuous(last_rz, rz_s_raw)
        rz_v = make_continuous(rz_s, rz_v_raw)
        rz_t = make_continuous(rz_v, rz_t_raw)
        last_rz = rz_t 

        drl_code += f"# Layer {i} - {direction}\n"
        drl_code += f"p_s_{i} = posx({p_s[0]:.2f}, {p_s[1]:.2f}, {p_s[2]:.2f}, {rx_s:.2f}, {ry_s:.2f}, {rz_s:.2f})\n"
        drl_code += f"p_v_{i} = posx({p_v[0]:.2f}, {p_v[1]:.2f}, {p_v[2]:.2f}, {rx_v:.2f}, {ry_v:.2f}, {rz_v:.2f})\n"
        drl_code += f"p_t_{i} = posx({p_t[0]:.2f}, {p_t[1]:.2f}, {p_t[2]:.2f}, {rx_t:.2f}, {ry_t:.2f}, {rz_t:.2f})\n"
        
        drl_code += f"movel(p_s_{i}, v=20 if {i}==1 else 40, a=80, r=10)\n"
        drl_code += f"movec(p_v_{i}, p_t_{i}, v=10, a=10, angle=360, ori=2)\n\n"

    approach_z = FLOOR_Z + radius + 50.0
    a_rx, a_ry, _, a_pos = get_pure_blended_orientation(np.array([cx, cy, approach_z]), true_center, tool_length)
    drl_code += "release_force()\nrelease_compliance_ctrl()\n"
    drl_code += f"p_approach = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, {a_pos[2]:.2f}, {a_rx:.2f}, {a_ry:.2f}, -180.0)\n"
    drl_code += "movel(p_approach, v=50, a=100)\n"
    
    return drl_code

# =================================================================
# [5] 메인 제어 노드
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
        while not self.cli.wait_for_service(1.0): self.get_logger().info('제어기 서비스 대기중...')

    def cb(self, msg):
        if msg.data == "02" and not self.busy: self.trigger = True

    def wait_for_robot_stop(self, delay=1.5):
        from DSR_ROBOT2 import get_robot_state
        time.sleep(delay) 
        stop_time = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            state = get_robot_state()
            if state == 1:
                stop_time += 0.1
                if stop_time > 2.0: break 
            else: 
                stop_time = 0

    def run(self):
        self.busy = True
        self.trigger = False
        from DSR_ROBOT2 import get_current_posx
        
        # 1️⃣ [Phase 1] 돔 최고점 탐색 (DRL 내부 Z감지 로직 실행)
        print("\n🚀 [Phase 1] 돔 최고점 자동 탐색 (Z값 감지 하강)...")
        measure_drl = generate_measure_drl(CENTER_EXP, TOOL_LENGTH)
        self.cli.call_async(DrlStart.Request(code=measure_drl))
        
        # 로봇이 Z축을 내리다가 막혀서 스스로 stop() 할 때까지 대기
        self.wait_for_robot_stop(delay=2.0)
        
        # 2️⃣ 반경 자동 계산
        pos_touch = get_current_posx()[0]
        flange_z = pos_touch[2] 
        true_top_z = flange_z - TOOL_LENGTH 
        measured_radius = true_top_z - FLOOR_Z
        
        print("-" * 45)
        print(f"🎯 돔 크기 자동 측정 완료!")
        print(f"   - 로봇 플랜지 멈춘 높이 : {flange_z:.2f} mm")
        print(f"   - 툴 길이 반영 돔 표면 : {true_top_z:.2f} mm")
        print(f"   👉 자동 계산된 반경(R)  : {measured_radius:.2f} mm")
        print("-" * 45)

        # 3️⃣ [Phase 2] 계산된 반경으로 스캐닝 실행
        print("\n🚀 [Phase 2] 스캐닝 DRL 생성 및 시작...")
        self.state_pub.publish(Bool(data=True)) 
        
        scan_drl = generate_scan_drl(CENTER_EXP, measured_radius, TOTAL_LAYERS, TOOL_LENGTH)
        self.cli.call_async(DrlStart.Request(code=scan_drl))
        
        self.wait_for_robot_stop(delay=2.0)
            
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