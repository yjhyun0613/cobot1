import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from dsr_msgs2.srv import DrlStart
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

RADIUS       = 50.0            
TOTAL_LAYERS = 5    
CENTER       = np.array([421.15, 73.76, 53.5]) 

SAFE_Z_HEIGHT = 200.0  
LIFT_HEIGHT   = 15.0   
MAX_RADIUS_EXPANSION = 8.0  # 💡 7.0에서 9.0으로 증가 (맨 아래층 쪼임 방지)

# =================================================================
# [2] 수학 및 기하학 보조 함수 
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

def get_pure_blended_orientation(target_pos, center):
    pure_normal = -normalize(target_pos - center)
    down_z_axis = np.array([0.0, 0.0, -1.0])
    z_axis_final = normalize((down_z_axis * 3.0) + (pure_normal * 1.0))

    up = np.array([0.0, 1.0, 0.0]) if abs(z_axis_final[2]) > 0.95 else np.array([0.0, 0.0, 1.0])
    x_axis = normalize(np.cross(up, z_axis_final))
    y_axis = normalize(np.cross(z_axis_final, x_axis))
    
    rx, ry, rz_raw = rot_to_zyz(np.column_stack((x_axis, y_axis, z_axis_final)))
    return wrap_angle(rx), wrap_angle(ry), rz_raw

# =================================================================
# [3] 층별 데이터 생성기 (DRL 쪼개기)
# =================================================================
def generate_layer_waypoints(center, radius, total_layers):
    cx, cy, cz = center
    layers_data = []
    
    # 1층 시작점 추출
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

        # 💡 [핵심] DRL에는 오직 돔 표면을 훑는 movec(스캔) 명령만 들어갑니다!
        drl_code = "set_singularity_handling(1)\n"
        drl_code += "stiff = [50, 3000, 300, 300, 300, 300]\n"
        drl_code += "task_compliance_ctrl(stiff)\n"
        drl_code += f"p_via = posx({via_xyz[0]:.2f}, {via_xyz[1]:.2f}, {via_xyz[2]:.2f}, {vr_x:.2f}, {vr_y:.2f}, {vr_z:.2f})\n"
        drl_code += f"p_tgt = posx({tgt_xyz[0]:.2f}, {tgt_xyz[1]:.2f}, {tgt_xyz[2]:.2f}, {tr_x:.2f}, {tr_y:.2f}, {tr_z:.2f})\n"
        drl_code += f"movec(p_via, p_tgt, v=50, a=100, angle=360.0, ori=2)\n"
        drl_code += "release_compliance_ctrl()\n"

        p_lift_in = [start_xyz[0], start_xyz[1], start_xyz[2] + LIFT_HEIGHT, sr_x, sr_y, sr_z] if i > 1 else None
        p_start = [start_xyz[0], start_xyz[1], start_xyz[2], sr_x, sr_y, sr_z]
        p_lift_out = [start_xyz[0], start_xyz[1], start_xyz[2] + LIFT_HEIGHT, sr_x, sr_y, sr_z] if i < total_layers else None

        layers_data.append({
            'lift_in': p_lift_in, 'start': p_start, 'drl': drl_code, 'lift_out': p_lift_out
        })

    return p_top, p_start_1, layers_data

# =================================================================
# [4] 제어 노드 (Pause/Resume 로직 포함)
# =================================================================
class ScanRunnerNode(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)
        self.state_pub = self.create_publisher(Bool, f'/{namespace}/checking_state', 10)
        
        # 💡 [핵심 추가] 궤적 기록을 통제할 새로운 퍼블리셔
        self.enable_pub = self.create_publisher(Bool, f'/{namespace}/record_enable', 10) 
        
        self.progress_pub = self.create_publisher(String, f'/{namespace}/progress_status', 10)
        self.trigger_scan = False
        self.is_scanning = False 
        self.create_subscription(String, f'/{namespace}/progress_status', self.progress_callback, 10)
        
        self.drl_client = self.create_client(DrlStart, f'/{namespace}/drl/drl_start')
        while not self.drl_client.wait_for_service(1.0): self.get_logger().info('DRL 대기 중...')

    def progress_callback(self, msg):
        if msg.data == "02" and not self.is_scanning and not self.trigger_scan:
            self.trigger_scan = True

    def publish_state(self, state: bool):
        self.state_pub.publish(Bool(data=state)) 
        
    def publish_record_enable(self, state: bool):
        self.enable_pub.publish(Bool(data=state)) 

    def wait_for_idle(self, required_time=1.0):
        from DSR_ROBOT2 import get_robot_state
        time.sleep(0.5) 
        stop_count = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if get_robot_state() == 1: 
                stop_count += 0.1
                if stop_count >= required_time: break
            else:
                stop_count = 0
            time.sleep(0.1)

    def run(self):
        self.busy, self.trigger_scan = True, False
        from DSR_ROBOT2 import movej, movel
        
        p_top, p_start_1, layers_data = generate_layer_waypoints(CENTER, RADIUS, TOTAL_LAYERS)
        
        movej([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=100.0, acc=100.0)
        movel(p_top, vel=50.0, acc=100.0)
        movel(p_start_1, vel=40.0, acc=80.0)
        
        # 💡 대기 시간 최소화 (0.5초 -> 0.1초)
        self.wait_for_idle(0.1)

        # 전체 분석 준비 시작 (이때는 스위치가 꺼져 있어서 기록 안 함)
        self.publish_state(True)
        self.publish_record_enable(False) 

        for i, layer in enumerate(layers_data):
            print(f"\n🛸 Layer {i+1} 스캔 시작...")
            
            # 1. 띄운 상태로 이동 및 표면 안착 (기록 안됨 ❌)
            if layer['lift_in']:
                movel(layer['lift_in'], vel=40.0, acc=80.0)
                self.wait_for_idle(0.1) # 💡 대기 시간 0.1초
                movel(layer['start'], vel=20.0, acc=40.0)
                self.wait_for_idle(0.1) # 💡 대기 시간 0.1초

            # 2. 🟢 표면 도착 완료! 스위치 ON 및 스캔 진행 (기록 됨 ⭕)
            print("   -> 🟢 데이터 기록 개시 (스위치 ON)")
            self.publish_record_enable(True)
            
            req = DrlStart.Request()
            req.robot_system = 0; req.code = layer['drl']
            self.drl_client.call_async(req)
            
            # 💡 스캔 완료 대기는 확실한 완료를 위해 3.0초 유지
            self.wait_for_idle(3.0) 

            # 3. 🔴 스캔 완료! 스위치 OFF (기록 정지 ❌)
            self.publish_record_enable(False)
            print("   -> 🔴 기록 일시정지 (스위치 OFF)")

            # 4. 다음 층을 위해 수직으로 띄우기 (기록 안됨 ❌)
            if layer['lift_out']:
                movel(layer['lift_out'], vel=40.0, acc=80.0)
                self.wait_for_idle(0.1) # 💡 대기 시간 0.1초

        self.publish_state(False)
        self.progress_pub.publish(String(data="03"))
        
        print("\n⬆️ 안전 고도 상승 후 Ready 자세 복귀...")
        movel(p_top, vel=50.0, acc=100.0)
        movej([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=100.0, acc=100.0)

        self.is_scanning = False 
        print("🏁 스캐닝 사이클 완벽 종료! 깔끔한 궤적 데이터가 저장되었습니다.")

def main(args=None):
    DR_init.__dsr__id, DR_init.__dsr__model = ROBOT_ID, ROBOT_MODEL
    rclpy.init(args=args)
    node = ScanRunnerNode('scan_runner_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
     
    try:
        from DSR_ROBOT2 import set_robot_mode, set_tool, set_tcp, ROBOT_MODE_AUTONOMOUS, ROBOT_MODE_MANUAL
        set_robot_mode(ROBOT_MODE_MANUAL); set_tool(ROBOT_TOOL); set_tcp(ROBOT_TCP); set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        
        print(f"\n🔄 신호 대기 중... (일시정지 동기화 모드 | 최대 확장 반경: +{MAX_RADIUS_EXPANSION}mm)")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.trigger_scan: node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()