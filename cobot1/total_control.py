import sys
import time
import math
import numpy as np

# ============================================================
# DSR_ROBOT2 / DR_init 경로 설정
# ============================================================
DSR_SITE_PACKAGES_PATH = "/home/rokey/cobot_ws/install/dsr_common2/lib/python3.10/site-packages"
if DSR_SITE_PACKAGES_PATH in sys.path:
    sys.path.remove(DSR_SITE_PACKAGES_PATH)
sys.path.insert(0, DSR_SITE_PACKAGES_PATH)

import DR_init
sys.modules["DR_init"] = DR_init

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from dsr_msgs2.srv import MoveJoint, MoveLine, DrlStart

# ============================================================
# 환경 설정 및 상수
# ============================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

ON, OFF = 1, 0
PROGRESS_TOPIC = f"/{ROBOT_ID}/progress_status"
RESULT_TOPIC = f"/{ROBOT_ID}/inspection_result"

# 돔 스캔 관련 상수
RADIUS = 48.5               
TOTAL_LAYERS = 10
CENTER = np.array([422.5, 72.76, 54.0]) 
SAFE_Z_HEIGHT = 200.0
LIFT_HEIGHT = 10.0   
MAX_RADIUS_EXPANSION = 9.5  

# ============================================================
# 기하학 연산 보조 함수 (스캔용)
# ============================================================
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

def get_pure_blended_orientation(target_pos, center):
    pure_normal = -normalize(target_pos - center)
    down_z_axis = np.array([0.0, 0.0, -1.0])
    z_axis_final = normalize((down_z_axis * 2.5) + (pure_normal * 1.0))
    up = np.array([0.0, 1.0, 0.0]) if abs(z_axis_final[2]) > 0.95 else np.array([0.0, 0.0, 1.0])
    x_axis = normalize(np.cross(up, z_axis_final))
    y_axis = normalize(np.cross(z_axis_final, x_axis))
    rx, ry, rz_raw = rot_to_zyz(np.column_stack((x_axis, y_axis, z_axis_final)))
    return wrap_angle(rx), wrap_angle(ry), rz_raw

def generate_layer_waypoints(center, radius, total_layers):
    cx, cy, cz = center
    layers_data = []
    
    angle_rad_1 = math.radians(90.0 * (1.0 - 1.0/total_layers))
    curr_z_1 = cz + radius * math.sin(angle_rad_1)
    curr_r_1 = radius * math.cos(angle_rad_1)
    start_1_xyz = np.array([cx - curr_r_1, cy, curr_z_1])
    s1_rx, s1_ry, _ = get_pure_blended_orientation(start_1_xyz, center)
    p_top = [cx, cy, SAFE_Z_HEIGHT, s1_rx, s1_ry, -180.0]
    p_start_1 = [start_1_xyz[0], start_1_xyz[1], start_1_xyz[2], s1_rx, s1_ry, -180.0]

    for i in range(1, total_layers + 1):
        expansion_ratio = (i - 1) / (total_layers - 1) if total_layers > 1 else 0
        current_expansion = MAX_RADIUS_EXPANSION * expansion_ratio

        angle_rad = math.radians(90.0 * (1.0 - float(i)/total_layers))
        curr_z = cz + radius * math.sin(angle_rad)
        curr_r = radius * math.cos(angle_rad) + current_expansion 
        
        start_xyz = np.array([cx - curr_r, cy, curr_z])
        tgt_xyz   = np.array([cx + curr_r, cy, curr_z])
        
        if (i % 2) == 1:
            via_xyz = np.array([cx, cy + curr_r, curr_z])
            target_start_rz = -180.0
        else:
            via_xyz = np.array([cx, cy - curr_r, curr_z])
            target_start_rz = 180.0

        sr_x, sr_y, _ = get_pure_blended_orientation(start_xyz, center)
        vr_x, vr_y, vr_z_raw = get_pure_blended_orientation(via_xyz, center)
        tr_x, tr_y, tr_z_raw = get_pure_blended_orientation(tgt_xyz, center)

        sr_z = target_start_rz
        vr_z = make_continuous(sr_z, vr_z_raw)
        tr_z = make_continuous(vr_z, tr_z_raw)

        drl_code = "set_singularity_handling(1)\n"
        drl_code += "stiff = [500, 500, 3000, 300, 300, 300]\n"
        drl_code += "task_compliance_ctrl(stiff, 0)\n" 
        drl_code += f"p_via = posx({via_xyz[0]:.2f}, {via_xyz[1]:.2f}, {via_xyz[2]:.2f}, {vr_x:.2f}, {vr_y:.2f}, {vr_z:.2f})\n"
        drl_code += f"p_tgt = posx({tgt_xyz[0]:.2f}, {tgt_xyz[1]:.2f}, {tgt_xyz[2]:.2f}, {tr_x:.2f}, {tr_y:.2f}, {tr_z:.2f})\n"
        drl_code += f"movec(p_via, p_tgt, v=10, a=20, angle=360.0, ori=2)\n"
        drl_code += "release_compliance_ctrl()\n"

        p_lift_in = [start_xyz[0], start_xyz[1], start_xyz[2] + LIFT_HEIGHT, sr_x, sr_y, sr_z] if i > 1 else None
        p_start = [start_xyz[0], start_xyz[1], start_xyz[2], sr_x, sr_y, sr_z]
        p_lift_out = [start_xyz[0], start_xyz[1], start_xyz[2] + LIFT_HEIGHT, sr_x, sr_y, sr_z] if i < total_layers else None

        layers_data.append({'lift_in': p_lift_in, 'start': p_start, 'drl': drl_code, 'lift_out': p_lift_out})

    return p_top, p_start_1, layers_data

# ============================================================
# 메인 마스터 노드
# ============================================================
class MasterInspectionNode(Node):
    def __init__(self):
        super().__init__("master_inspection_node", namespace=ROBOT_ID)

        setattr(DR_init, "__dsr__id", ROBOT_ID)
        setattr(DR_init, "__dsr__model", ROBOT_MODEL)
        setattr(DR_init, "__dsr__node", self)

        from DSR_ROBOT2 import (
            set_digital_output, get_digital_input, set_tool, set_tcp, 
            set_robot_mode, get_robot_state, movej, movel,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
        )
        self.set_digital_output = set_digital_output
        self.get_digital_input = get_digital_input
        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.set_robot_mode = set_robot_mode
        self.get_robot_state = get_robot_state 
        self.dsr_movej = movej
        self.dsr_movel = movel
        self.ROBOT_MODE_MANUAL = ROBOT_MODE_MANUAL
        self.ROBOT_MODE_AUTONOMOUS = ROBOT_MODE_AUTONOMOUS

        self.initialize_robot()

        # 통신 설정 (Pub/Sub/Client)
        self.sub_progress = self.create_subscription(String, PROGRESS_TOPIC, self.progress_callback, 10)
        self.sub_result = self.create_subscription(String, RESULT_TOPIC, self.result_callback, 10)
        
        self.pub_progress = self.create_publisher(String, PROGRESS_TOPIC, 10)
        self.pub_check_state = self.create_publisher(Bool, f"/{ROBOT_ID}/checking_state", 10)
        self.pub_record_en = self.create_publisher(Bool, f"/{ROBOT_ID}/record_enable", 10)
        
        self.drl_client = self.create_client(DrlStart, f'/{ROBOT_ID}/drl/drl_start')
        while not self.drl_client.wait_for_service(1.0): self.get_logger().info('DRL 서비스 대기 중...')

        # 상태 제어 변수
        self.system_state = "WAITING_01"  
        self.signal_01_received = False
        self.inspection_result_received = None

        # ====================================================
        # 주요 좌표 (Joint & PosX)
        # ====================================================
        self.home_joint = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.conveyor_above_pick_joint = [-31.66, 43.77, 37.3, 0, 99.24, -31.41]
        self.inspection_above_joint = [16.43, 14.66, 93.17, 0.21, 72.25, 24.45]
        self.tool_safe_joint = [7.2, 28.08, 75.94, -6.06, 48.37, 9.12] 
        self.pass_pos_joint = [35.75, 60.97, 15.71, 8.41, 66.92, 66.99]
        self.fail_pos_joint = [61.53, 31.85, 69.11, 1.16, 78.58, -14.88]

        self.conveyor_above_pick_posx = [552.47, -332.12, 136.34, 104.72, -177.01, 105.27]
        self.conveyor_pick_posx = [552.47, -332.12, 86.34, 104.72, -177.01, 105.27]
        self.inspection_above_posx = [431.75, 138.3, 66.5, 121.82, 179.67, 129.54]
        self.inspection_place_posx = [431.75, 138.3, 6.5, 121.82, 179.67, 129.54]
        self.tool_safe_posx = [708.82, 72.51, 99.76, 178.28, -151.92, 177.01]
        self.tool_place_posx = [744.15, 72.82, 33.01, 178.69, -152.04, 177.4]       
        self.pressing_above_posx = [425.35, 74.68, 101.92, 50.91, -179.12, 49.78]
        self.pressing_posx = [425.35, 74.68, 51.92, 50.91, -179.12, 49.78]

        self.get_logger().info("✅ M0609 통합 마스터 노드 준비 완료! '01' 신호 대기 중...")

    def initialize_robot(self):
        try:
            self.set_robot_mode(self.ROBOT_MODE_MANUAL)
            time.sleep(0.5)
            self.set_tool(ROBOT_TOOL)
            self.set_tcp(ROBOT_TCP)
            self.set_robot_mode(self.ROBOT_MODE_AUTONOMOUS)
            time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f"로봇 초기화 실패: {e}")

    # ============================================================
    # 콜백 및 통신 함수
    # ============================================================
    def clean_signal(self, data):
        return data.replace("'", "").replace('"', "").strip().upper()

    def progress_callback(self, msg):
        sig = self.clean_signal(msg.data)
        if sig == "01" and self.system_state == "WAITING_01":
            self.signal_01_received = True

    def result_callback(self, msg):
        res = self.clean_signal(msg.data)
        if res in ["PASS", "FAIL"] and self.system_state == "WAITING_RESULT":
            self.inspection_result_received = res

    def publish_status(self, status_code):
        self.pub_progress.publish(String(data=status_code))
        self.get_logger().info(f"📢 외부 상태 전송: '{status_code}'")

    # ============================================================
    # 💡 [핵심] 통신 지연 방지 (숨쉬기) 스마트 제어 함수
    # ============================================================
    def wait_smart(self, required_time=0.5):
        time.sleep(0.5) 
        stop_count = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01) # 통신 숨쉬기
            if self.get_robot_state() == 1: # 1: 정지 상태
                stop_count += 0.1
                if stop_count >= required_time: break
            else:
                stop_count = 0
            time.sleep(0.1)
        return True

    def movej_smart(self, desc, joint, v=30, a=30):
        self.get_logger().info(f"🤖 이동: {desc}")
        self.dsr_movej(joint, vel=v, acc=a)
        return self.wait_smart()

    def movel_smart(self, desc, posx, v=50, a=50):
        self.get_logger().info(f"🤖 이동: {desc}")
        self.dsr_movel(posx, vel=v, acc=a)
        return self.wait_smart()

    def gripper_smart(self, action, desc):
        self.get_logger().info(f"🦾 {desc}")
        on_pin, off_pin, di_pin = (1, 2, 1) if action == "close" else (2, 1, 2)
        self.set_digital_output(on_pin, ON)
        self.set_digital_output(off_pin, OFF)
        
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.01) # 통신 숨쉬기
            if self.get_digital_input(di_pin):
                time.sleep(0.5) 
                return True
            time.sleep(0.1)
        return False

    # ============================================================
    # 🚀 공정 1: First Work (가져오기 & 압입 & 툴 장착)
    # ============================================================
    def run_first_sequence(self):
        self.get_logger().info("\n▶️ [STEP 1] 물건 픽업 및 툴 장착 시작")
        tasks = [
            ("movej", "컨베이어 위 접근", self.conveyor_above_pick_joint),
            ("gripper", "그리퍼 열기", "open"),
            ("movel", "컨베이어 좌표 이동", self.conveyor_pick_posx),
            ("gripper", "물건 집기", "close"),
            ("movel", "컨베이어 위 복귀", self.conveyor_above_pick_posx),
            ("movej", "검사 장소 위 이동", self.inspection_above_joint),
            ("movel", "검사 장소 하강", self.inspection_place_posx),
            ("gripper", "물건 놓기", "open"),
            ("movel", "검사 장소 위 복귀", self.inspection_above_posx),
            ("movel", "압입 상부 이동", self.pressing_above_posx),
            ("movel", "압입 꾹 누르기", self.pressing_posx),
            ("movel", "압입 상부 복귀", self.pressing_above_posx),
            ("movej", "툴 거치대 위 이동", self.tool_safe_joint),
            ("movel", "툴 장착 위치 이동", self.tool_place_posx),
            ("gripper", "툴 잡기", "close"),
            ("movel", "툴 들고 안전위치 복귀", self.tool_safe_posx),
            ("movej", "초기 위치 이동", self.home_joint)
        ]
        for t, desc, target in tasks:
            if t == "movej": self.movej_smart(desc, target)
            elif t == "movel": self.movel_smart(desc, target)
            elif t == "gripper": self.gripper_smart(target, desc)
        return True

    # ============================================================
    # 🚀 공정 2: Spin (3D 돔 스캐닝)
    # ============================================================
    def run_scan_sequence(self):
        self.get_logger().info("\n▶️ [STEP 2] 3D 돔 표면 검사 스캔 시작")
        p_top, p_start_1, layers_data = generate_layer_waypoints(CENTER, RADIUS, TOTAL_LAYERS)
        
        self.movej_smart("스캔 시작 고도 이동", [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], v=100, a=100)
        self.movel_smart("스캔 중앙 상단", p_top, v=50, a=100)
        self.movel_smart("스캔 1층 진입", p_start_1, v=40, a=80)
        
        # 파이어베이스 뷰어 전체 기록 시작 (새로운 검사)
        self.pub_check_state.publish(Bool(data=True))
        self.pub_record_en.publish(Bool(data=False))

        for i, layer in enumerate(layers_data):
            self.get_logger().info(f"🛸 Layer {i+1} 스캔 중...")
            if layer['lift_in']:
                self.movel_smart(f"Layer {i+1} 진입(상승)", layer['lift_in'], v=60, a=100)
                self.movel_smart(f"Layer {i+1} 진입(하강)", layer['start'], v=30, a=60)

            # 실시간 데이터 기록 켜기
            self.pub_record_en.publish(Bool(data=True))
            
            req = DrlStart.Request()
            req.robot_system = 0; req.code = layer['drl']
            self.drl_client.call_async(req)
            self.wait_smart(1.0) # 💡 DRL 실행 중에도 외부 신호를 받을 수 있도록 wait_smart 사용!

            # 실시간 데이터 기록 끄기
            self.pub_record_en.publish(Bool(data=False))

            if layer['lift_out']:
                self.movel_smart(f"Layer {i+1} 탈출", layer['lift_out'], v=60, a=100)

        # 검사 완전 종료 신호 전송
        self.pub_check_state.publish(Bool(data=False))
        self.movej_smart("스캔 완료 후 홈 복귀", [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], v=100, a=100)
        return True

    # ============================================================
    # 🚀 공정 3: Last Work (툴 반납 & 결과에 따른 분류)
    # ============================================================
    def run_last_sequence(self, result):
        self.get_logger().info(f"\n▶️ [STEP 3] 검사 결과({result})에 따른 분류 및 원복 시작")
        target_pos = self.pass_pos_joint if result == "PASS" else self.fail_pos_joint
        
        tasks = [
            ("movej", "도구 반납 접근", self.tool_safe_joint),
            ("movel", "도구 최종 좌표 이동", self.tool_place_posx),
            ("gripper", "도구 내려놓기", "open"),
            ("movel", "도구 안전 탈출", self.tool_safe_posx),
            ("movej", "초기 위치 복귀", self.home_joint),
            ("movej", "검사 장소 위 이동", self.inspection_above_joint),
            ("gripper", "그리퍼 열기", "open"),
            ("movel", "물체 위치 하강", self.inspection_place_posx),
            ("gripper", "물체 다시 잡기", "close"),
            ("movel", "검사 장소 위 상승", self.inspection_above_posx),
            ("movej", f"분배 이동 ({result})", target_pos),
            ("gripper", "물체 놓기", "open"),
            ("movej", "최종 홈 복귀", self.home_joint)
        ]
        for t, desc, target in tasks:
            if t == "movej": self.movej_smart(desc, target)
            elif t == "movel": self.movel_smart(desc, target)
            elif t == "gripper": self.gripper_smart(target, desc)
        return True

# ============================================================
# 메인 루프 (상태 머신)
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = MasterInspectionNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # 1. 01번 신호를 기다리는 상태
            if node.system_state == "WAITING_01" and node.signal_01_received:
                node.signal_01_received = False
                node.system_state = "RUNNING_FIRST"
                
                node.run_first_sequence()
                node.publish_status("02") # GUI에 진행 상황 알림
                node.system_state = "RUNNING_SCAN" # 즉시 다음 공정으로!

            # 2. 내부적으로 스캔을 자동 시작하는 상태
            if node.system_state == "RUNNING_SCAN":
                node.run_scan_sequence()
                node.publish_status("03")
                node.get_logger().info("🔄 파이어베이스의 PASS/FAIL 결과를 대기합니다...")
                node.system_state = "WAITING_RESULT"

            # 3. 분석 결과를 기다리는 상태
            if node.system_state == "WAITING_RESULT" and node.inspection_result_received:
                res = node.inspection_result_received
                node.inspection_result_received = None
                node.system_state = "RUNNING_LAST"
                
                node.run_last_sequence(res)
                node.publish_status("04")
                node.get_logger().info("🏁 모든 공정 완료! 다음 '01' 신호를 기다립니다.")
                node.system_state = "WAITING_01"

    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()