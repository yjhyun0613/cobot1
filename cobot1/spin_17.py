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

# 💡 궤적 반경은 46.0(압축), 실제 돔 반경은 48.0으로 설정
REAL_RADIUS  = 48.0            
PATH_RADIUS  = 46.0            
TOTAL_LAYERS = 10
CENTER       = np.array([422.58, 66.46, 25.50]) # [X, Y, Z]

SAFE_Z_HEIGHT = 200.0  # 대기 위치의 안전 Z 높이
TOOL_LENGTH  = 30      # 장착된 툴의 길이

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

def get_pure_blended_orientation(target_pos, center, tool_length=0.0):
    pure_normal = -normalize(target_pos - center)
    down_z_axis = np.array([0.0, 0.0, -1.0])
    
    z_axis_final = normalize((down_z_axis * 3.0) + (pure_normal * 1.0))
    up = np.array([0.0, 1.0, 0.0]) if abs(z_axis_final[2]) > 0.95 else np.array([0.0, 0.0, 1.0])
    x_axis = normalize(np.cross(up, z_axis_final))
    y_axis = normalize(np.cross(z_axis_final, x_axis))
    
    rx, ry, rz_raw = rot_to_zyz(np.column_stack((x_axis, y_axis, z_axis_final)))
    offset_pos = target_pos - (z_axis_final * tool_length)

    return wrap_angle(rx), wrap_angle(ry), rz_raw, offset_pos

# =================================================================
# [3] 💡 융합된 DRL 경로 생성기 (Force Control + Dome Scan)
# =================================================================
def generate_force_dome_drl(center, real_radius, path_radius, total_layers, tool_length):
    cx, cy, cz = center

    # 1. 돔의 물리적 꼭대기보다 10mm 높은 곳을 Approach(접근) 지점으로 계산
    approach_z = cz + real_radius + 10.0
    approach_xyz = np.array([cx, cy, approach_z])
    a_rx, a_ry, _, a_pos = get_pure_blended_orientation(approach_xyz, center, tool_length)
    a_rz = -180.0

    drl_code = 'set_tool("Tool Weight")\n'
    drl_code += 'set_tcp("GripperDA_v1")\n'
    drl_code += 'set_singularity_handling(1)\n\n'
    
    # 접근 지점으로 바로 이동 (대기 Z=200을 거쳐서)
    drl_code += f'p_top = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, 200.0, {a_rx:.2f}, {a_ry:.2f}, {a_rz:.2f})\n'
    drl_code += f'p_approach = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, {a_pos[2]:.2f}, {a_rx:.2f}, {a_ry:.2f}, {a_rz:.2f})\n'
    drl_code += 'movel(p_top, v=100.0, a=100.0)\n'
    drl_code += 'movel(p_approach, v=50.0, a=100.0)\n'
    drl_code += 'sleep(1.0)\n\n'

    # 영점 측정 (툴 좌표계 기준)
    drl_code += '# 허공에서의 툴 기준 힘 측정 (영점)\n'
    drl_code += 'base_f = get_tool_force(1) # 1: Tool 좌표계\n'
    drl_code += 'zero_tz = base_f[2]\n\n'
    
    # 순응 제어 ON (💡 기준을 Tool(1)로 맞춰서 로봇이 기울어도 툴 Z축이 스프링이 됨)
    drl_code += 'stiff = [300, 300, 100, 300, 300, 300]\n'
    drl_code += 'task_compliance_ctrl(stiff, 1)\n\n'
    
    # 힘 제어 ON (-15N으로 하강 시작)
    drl_code += '# 툴 방향(Z축)으로 -15N 힘 주며 탐색 하강\n'
    drl_code += 'fd_down = [0.0, 0.0, -15.0, 0.0, 0.0, 0.0]\n'
    drl_code += 'fdir = [0, 0, 1, 0, 0, 0]\n'
    drl_code += 'set_desired_force(fd_down, fdir, 1)\n\n'
    
    # 표면 접촉 감지
    drl_code += 'while True:\n'
    drl_code += '    cur_f = get_tool_force(1)\n'
    drl_code += '    if abs(cur_f[2] - zero_tz) > 6.0:\n'
    drl_code += '        break\n'
    drl_code += '    sleep(0.001)\n\n'
    
    drl_code += '# 💡 힘을 끄지 않고 유지한 채로 압축된 궤적(46.0) 돔 스캔 진입!\n'

    # 층별 회전 로직 (path_radius = 46.0 기준)
    for i in range(1, total_layers + 1):
        angle_rad = math.radians(90.0 * (1.0 - float(i)/total_layers))
        curr_z = cz + path_radius * math.sin(angle_rad)
        curr_r = path_radius * math.cos(angle_rad)
        
        start_xyz = np.array([cx - curr_r, cy, curr_z])
        tgt_xyz   = np.array([cx + curr_r, cy, curr_z])
        
        if (i % 2) == 1:
            via_xyz = np.array([cx, cy + curr_r, curr_z])
            direction = "CW"
            target_start_rz = -180.0
        else:
            via_xyz = np.array([cx, cy - curr_r, curr_z])
            direction = "CCW"
            target_start_rz = 180.0

        sr_x, sr_y, _, s_pos = get_pure_blended_orientation(start_xyz, center, tool_length)
        vr_x, vr_y, vr_z_raw, v_pos = get_pure_blended_orientation(via_xyz, center, tool_length)
        tr_x, tr_y, tr_z_raw, t_pos = get_pure_blended_orientation(tgt_xyz, center, tool_length)

        sr_z = target_start_rz
        vr_z = make_continuous(sr_z, vr_z_raw)
        tr_z = make_continuous(vr_z, tr_z_raw)

        drl_code += f"# Layer {i} - {direction}\n"
        drl_code += f"p_start_{i} = posx({s_pos[0]:.2f}, {s_pos[1]:.2f}, {s_pos[2]:.2f}, {sr_x:.2f}, {sr_y:.2f}, {sr_z:.2f})\n"
        drl_code += f"p_via_{i}   = posx({v_pos[0]:.2f}, {v_pos[1]:.2f}, {v_pos[2]:.2f}, {vr_x:.2f}, {vr_y:.2f}, {vr_z:.2f})\n"
        drl_code += f"p_tgt_{i}   = posx({t_pos[0]:.2f}, {t_pos[1]:.2f}, {t_pos[2]:.2f}, {tr_x:.2f}, {tr_y:.2f}, {tr_z:.2f})\n"
        
        # 첫 진입은 부드럽게
        if i == 1:
            drl_code += f"movel(p_start_{i}, v=20, a=40, r=10.0)\n"
        else:
            drl_code += f"movel(p_start_{i}, v=40, a=80, r=10.0)\n"
            
        drl_code += f"movec(p_via_{i}, p_tgt_{i}, v=50, a=100, angle=360.0, ori=2)\n\n"

    # 제어 해제 및 안전 복귀
    drl_code += "release_force()\n"
    drl_code += "release_compliance_ctrl()\n"
    drl_code += "movel(p_approach, v=50.0, a=100.0)\n"
    
    return drl_code

# =================================================================
# [4] ROS2 통신 노드 (Node Communication)
# =================================================================
class ScanRunnerNode(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)
        self.state_pub = self.create_publisher(Bool, f'/{namespace}/checking_state', 10)
        self.progress_pub = self.create_publisher(String, f'/{namespace}/progress_status', 10)
        
        self.trigger_scan = False
        self.is_scanning = False 
        
        self.create_subscription(String, f'/{namespace}/progress_status', self.progress_callback, 10)
        
        srv_name = f'/{namespace}/drl/drl_start'
        self.drl_client = self.create_client(DrlStart, srv_name)
        while not self.drl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{srv_name} 서비스 대기 중...')

    def progress_callback(self, msg):
        if msg.data == "02" and not self.is_scanning and not self.trigger_scan:
            self.trigger_scan = True

    def publish_state(self, state: bool):
        self.state_pub.publish(Bool(data=state))
        self.get_logger().info("🟢 기록 시작 (표면 스캔 진입)" if state else "🔴 기록 종료 (스캔 완료)")

    def publish_completion(self):
        self.progress_pub.publish(String(data="03"))
        self.get_logger().info('🏁 작업 완료 신호(03) 송신')

    def send_drl_code(self, drl_script):
        req = DrlStart.Request()
        req.robot_system = 0        
        req.code = drl_script       
        self.drl_client.call_async(req)

# =================================================================
# [5] 메인 제어 흐름
# =================================================================
def main(args=None):
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    rclpy.init(args=args)
    node = ScanRunnerNode('scan_runner_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
     
    try:
        from DSR_ROBOT2 import set_robot_mode, set_tool, set_tcp, get_robot_state, get_current_posx
        from DSR_ROBOT2 import ROBOT_MODE_AUTONOMOUS, ROBOT_MODE_MANUAL

        print("\n⚙️ 로봇 초기화 중...")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(1.0) 

        print("\n🔄 대기 모드: 신호(02) 대기 중...")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if node.trigger_scan:
                node.is_scanning = True
                node.trigger_scan = False
                
                # 💡 합쳐진 DRL 코드 생성 호출
                drl_code = generate_force_dome_drl(CENTER, REAL_RADIUS, PATH_RADIUS, TOTAL_LAYERS, TOOL_LENGTH)
                
                print("\n🚀 제어기로 DRL 전송 중 (접근 및 탐색 시작)...")
                node.send_drl_code(drl_code)
                time.sleep(1.0)
                
                print("⏳ 돔 꼭대기를 찾는 중입니다...")
                
                # 💡 [핵심] 바닥에 닿은 후 첫 궤적 진입을 감지하여 기록 켜기!
                # 로봇이 꼭대기(X=422.58)에서 하강하다가, 첫 번째 층을 위해 X축으로 움직이는 순간을 감지합니다.
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.01)
                    current_x = get_current_posx()[0][0]
                    
                    # 중심 X좌표에서 5.0mm 이상 멀어지면 돔 훑기를 시작한 것으로 간주
                    if abs(current_x - CENTER[0]) > 5.0:
                        node.publish_state(True)
                        print("🛸 돔 표면 안착 완료! 스캐닝 기록을 시작합니다.")
                        break
                    time.sleep(0.05)
                
                # 스캔 완료 판정 (상태가 1(대기)로 돌아오면 완료)
                stop_count = 0
                required_stop_time = 3.0 
                check_interval = 0.5
                
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.01)
                    state = get_robot_state()
                    
                    if state == 1: 
                        stop_count += check_interval
                        if stop_count >= required_stop_time:
                            break
                    else:
                        stop_count = 0 
                    
                    time.sleep(check_interval)

                node.publish_state(False)
                node.publish_completion()
                print("\n🔄 1 사이클 완료. 다음 신호 대기 중...")
                node.is_scanning = False 
                time.sleep(1.0) 

    except Exception as e:
        print(f"\n🚨 에러 발생: {e}")
        try:
            node.publish_state(False)
        except:
            pass
        
    finally:
        print("✅ 프로그램을 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()