import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from dsr_msgs2.srv import DrlStart
from dsr_msgs2.msg import RobotStop
import DR_init
import time
import math
import numpy as np
import threading

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
# [3] Step 2: 스캐닝 전용 DRL 생성기
# =================================================================
def generate_scan_drl(center, radius, total_layers, tool_length):
    cx, cy, cz = center
    true_center = np.array([cx, cy, FLOOR_Z]) 
    
    drl_code = 'set_tool("Tool Weight")\nset_tcp("GripperDA_v1")\nset_singularity_handling(1)\n\n'
    
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

    def run(self):
        self.busy = True
        self.trigger = False
        
        from DSR_ROBOT2 import (
            movej, movel, amovel, task_compliance_ctrl, 
            get_current_posx, release_compliance_ctrl, get_robot_state
        )
        
        # 1️⃣ [Phase 1] 비동기 하강 및 물리적 Z 정지 감지
        print("\n🚀 [Phase 1] Z축 하강 및 정지 감지 시작...")
        
        cx, cy, cz = CENTER_EXP
        approach_z = FLOOR_Z + 150.0
        a_rx, a_ry, _, a_pos = get_pure_blended_orientation(np.array([cx, cy, approach_z]), CENTER_EXP, TOOL_LENGTH)
        
        p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        p_approach = [a_pos[0], a_pos[1], a_pos[2], a_rx, a_ry, -180.0]

        movej(p_ready, vel=100.0, acc=100.0)
        movel(p_approach, vel=50.0, acc=100.0)
        time.sleep(1.0) 
        
        task_compliance_ctrl([3000, 3000, 100, 300, 300, 300])
        
        p_target = [a_pos[0], a_pos[1], FLOOR_Z, a_rx, a_ry, -180.0]
        amovel(p_target, vel=15.0, acc=30.0) 
        
        time.sleep(0.5)
        
        # 💡 [핵심 변경] 안전장치 변수 초기화
        start_z = get_current_posx()[0][2] # 출발 높이 저장
        prev_z = start_z
        stop_count = 0                     # 연속 정지 카운트
        
        print("⏬ 스르륵 하강 중... (오작동 방지 로직 가동 중)")
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.1)
            
            curr_z = get_current_posx()[0][2]
            
            # 🛡️ 1단계 안전장치: 최소 출발 거리 (Deadzone 3mm)
            moved_dist = abs(start_z - curr_z)
            if moved_dist < 3.0:
                prev_z = curr_z
                continue # 3mm 미만 이동 시에는 감지 로직 패스
                
            # 🛡️ 2단계 안전장치: 연속 멈춤 카운트 (Debouncing)
            diff = abs(prev_z - curr_z)
            if diff < 0.2:
                stop_count += 1
            else:
                stop_count = 0 # 0.2mm 이상 움직이면 초기화
                
            # 3회 연속(약 0.3초간) 정지 상태라면 바닥 감지 확정
            if stop_count >= 3:
                stop_msg = RobotStop()
                stop_msg.stop_mode = 2 
                self.stop_pub.publish(stop_msg)
                
                print(f"🛑 바닥 감지 확정! (최종 하강 높이: {curr_z:.2f}mm)")
                break
                
            prev_z = curr_z
            
        release_compliance_ctrl()
        time.sleep(1.0) 
        
        # 2️⃣ 반경 자동 계산
        pos_touch = get_current_posx()[0]
        flange_z = pos_touch[2] 
        true_top_z = flange_z - TOOL_LENGTH 
        measured_radius = true_top_z - FLOOR_Z
        
        print("-" * 45)
        print(f"🎯 돔 크기 파이썬 자동 측정 완료!")
        print(f"   - 로봇 플랜지 멈춘 높이 : {flange_z:.2f} mm")
        print(f"   - 툴 길이 반영 돔 표면 : {true_top_z:.2f} mm")
        print(f"   👉 자동 계산된 반경(R)  : {measured_radius:.2f} mm")
        print("-" * 45)

        # 3️⃣ [Phase 2] DRL 기반 스캐닝 실행
        print("\n🚀 [Phase 2] DRL 제어기로 스캐닝 위임 및 시작...")
        self.state_pub.publish(Bool(data=True)) 
        
        scan_drl = generate_scan_drl(CENTER_EXP, measured_radius, TOTAL_LAYERS, TOOL_LENGTH)
        self.cli.call_async(DrlStart.Request(code=scan_drl))
        
        time.sleep(2.0)
        stop_time = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if get_robot_state() == 1:
                stop_time += 0.1
                if stop_time > 2.0: break 
            else: 
                stop_time = 0
            
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