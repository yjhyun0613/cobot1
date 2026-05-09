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
CENTER_EXP   = np.array([422.15, 75.45, 25.50]) # X, Y 중심과 바닥 Z 높이
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
# [3] Step 2: 스캐닝 전용 DRL 생성기 (1바퀴 완전 스캔 로직 탑재)
# =================================================================
def generate_scan_drl(center, radius, total_layers, tool_length):
    cx, cy, cz = center
    true_center = np.array([cx, cy, FLOOR_Z]) 
    
    drl_code = 'set_tool("Tool Weight")\nset_tcp("GripperDA_v1")\nset_singularity_handling(1)\n\n'
    
    # Base 좌표계(ref=0) 기준 바닥으로 -10N 힘 적용
    drl_code += 'stiff = [3000, 3000, 50, 300, 300, 300]\ntask_compliance_ctrl(stiff, 0)\n'
    drl_code += 'set_desired_force([0,0,-10,0,0,0], [0,0,1,0,0,0], 0)\nsleep(0.5)\n\n'

    last_rz = -180.0 

    for i in range(1, total_layers + 1):
        angle_rad = math.radians(90.0 * (1.0 - float(i)/total_layers))
        curr_z = FLOOR_Z + radius * math.sin(angle_rad)
        curr_r = radius * math.cos(angle_rad)
        
        # 💡 [핵심 변경] 모든 층은 '왼쪽(Left)'에서 시작해서 한 바퀴를 완벽히 돕니다.
        p_left  = np.array([cx - curr_r, cy, curr_z])
        p_top   = np.array([cx, cy + curr_r, curr_z])
        p_right = np.array([cx + curr_r, cy, curr_z])
        p_bot   = np.array([cx, cy - curr_r, curr_z])

        if i % 2 == 1:
            # CW: 왼쪽 -> 위쪽 -> 오른쪽 -> 아래쪽 -> 다시 왼쪽
            pts = [p_left, p_top, p_right, p_bot, p_left]
            direction = "CW"
        else:
            # CCW: 왼쪽 -> 아래쪽 -> 오른쪽 -> 위쪽 -> 다시 왼쪽 (선 꼬임 방지)
            pts = [p_left, p_bot, p_right, p_top, p_left]
            direction = "CCW"

        # 각 4분면의 좌표와 자세 계산
        ro_pts = []
        for pt in pts:
            rx, ry, rz_raw, pos = get_pure_blended_orientation(pt, true_center, tool_length)
            rz = make_continuous(last_rz, rz_raw)
            last_rz = rz
            ro_pts.append((rx, ry, rz, pos))

        # DRL 코드 작성
        vel = 20.0 if i == 1 else 40.0
        drl_code += f"# Layer {i} - 1 Full Circle ({direction})\n"

        for idx in range(5):
            rx, ry, rz, pos = ro_pts[idx]
            drl_code += f"p_{i}_{idx} = posx({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {rx:.2f}, {ry:.2f}, {rz:.2f})\n"

        # 1. 왼쪽 시작점으로 이동
        drl_code += f"movel(p_{i}_0, v={vel}, a=80.0)\n"
        # 2. 첫 번째 반 바퀴 (왼쪽 -> 경유 -> 오른쪽)
        drl_code += f"movec(p_{i}_1, p_{i}_2, v=15.0, a=20.0)\n"
        # 3. 두 번째 반 바퀴 (오른쪽 -> 경유 -> 왼쪽 원대복귀)
        drl_code += f"movec(p_{i}_3, p_{i}_4, v=15.0, a=20.0)\n\n"

    approach_z = FLOOR_Z + radius + 50.0
    a_rx, a_ry, _, a_pos = get_pure_blended_orientation(np.array([cx, cy, approach_z]), true_center, tool_length)
    drl_code += "release_force()\nrelease_compliance_ctrl()\n"
    drl_code += f"p_approach = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, {a_pos[2]:.2f}, {a_rx:.2f}, {a_ry:.2f}, -180.0)\n"
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
            if state == 1: # 1: STANBY 상태
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
        
        # 1️⃣ [Phase 1] 파이썬 기반 안전 감지 로직 (눈으로 확인 가능)
        print("\n🚀 [Phase 1] 돔 최고점 자동 탐색 (Base 좌표계 힘 제어 하강)...")
        
        cx, cy, cz = CENTER_EXP
        approach_z = FLOOR_Z + 150.0
        a_rx, a_ry, _, a_pos = get_pure_blended_orientation(np.array([cx, cy, approach_z]), CENTER_EXP, TOOL_LENGTH)
        
        p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        p_approach = [a_pos[0], a_pos[1], a_pos[2], a_rx, a_ry, -180.0]

        movej(p_ready, vel=100.0, acc=100.0)
        movel(p_approach, vel=50.0, acc=100.0)
        time.sleep(1.0) 
        
        task_compliance_ctrl([3000, 3000, 100, 300, 300, 300], 0)
        time.sleep(0.5)
        
        print("⏬ 스르륵 하강 시작...")
        set_desired_force([0, 0, -15, 0, 0, 0], [0, 0, 1, 0, 0, 0], 0)
        time.sleep(1.0) # 출발 가속 시간 확보
        
        start_z = get_current_posx()[0][2]
        prev_z = start_z
        stop_count = 0 
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.1)
            
            curr_z = get_current_posx()[0][2]
            
            # Deadzone 방어
            if abs(start_z - curr_z) < 3.0:
                prev_z = curr_z
                continue 
                
            # 정지 감지 (0.1초 동안 0.2mm 미만 이동)
            if abs(prev_z - curr_z) < 0.2:
                stop_count += 1
            else:
                stop_count = 0 
                
            # 3회 연속 감지 시 확정
            if stop_count >= 3:
                release_force() 
                
                # 강제 정지 토픽 발송
                stop_msg = RobotStop()
                stop_msg.stop_mode = 2 
                self.stop_pub.publish(stop_msg)
                print(f"🛑 돔 표면 감지 확정! (최종 하강 높이: {curr_z:.2f}mm)")
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
        print(f"🎯 돔 크기 측정 완료!")
        print(f"   - 로봇 플랜지 멈춘 높이 : {flange_z:.2f} mm")
        print(f"   - 툴 길이 반영 돔 표면 : {true_top_z:.2f} mm")
        print(f"   👉 자동 계산된 반경(R)  : {measured_radius:.2f} mm")
        print("-" * 45)

        # 3️⃣ [Phase 2] 1바퀴 완전 스캔 DRL 실행
        print("\n🚀 [Phase 2] 스캐닝 DRL 구동 시작...")
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