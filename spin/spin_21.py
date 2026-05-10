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

REAL_RADIUS  = 45.0            
PATH_RADIUS  = 45.0            
TOTAL_LAYERS = 3
CENTER       = np.array([424.15, 75.45, 25.50]) # [X, Y, Z] (참고용 중심점)

SAFE_Z_HEIGHT = 200.0  
TOOL_LENGTH  = 30      

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
    
    # 툴의 자세 결정
    z_axis_final = normalize((down_z_axis * 2.0) + (pure_normal * 1.0))
    up = np.array([0.0, 1.0, 0.0]) if abs(z_axis_final[2]) > 0.95 else np.array([0.0, 0.0, 1.0])
    x_axis = normalize(np.cross(up, z_axis_final))
    y_axis = normalize(np.cross(z_axis_final, x_axis))
    
    rx, ry, rz_raw = rot_to_zyz(np.column_stack((x_axis, y_axis, z_axis_final)))
    offset_pos = target_pos - (z_axis_final * tool_length)

    return wrap_angle(rx), wrap_angle(ry), rz_raw, offset_pos

# =================================================================
# [3] 💡 융합된 DRL 경로 생성기 (터치 좌표 절대 앵커링 적용)
# =================================================================
def generate_force_dome_drl(center, real_radius, path_radius, total_layers, tool_length):
    cx, cy, cz = center

    # 파이썬이 예상하는 돔 위의 접근 지점
    approach_z = cz + real_radius + 10.0
    approach_xyz = np.array([cx, cy, approach_z])
    a_rx, a_ry, _, a_pos = get_pure_blended_orientation(approach_xyz, center, tool_length)
    a_rz = -180.0

    # 💡 [핵심] 실제 돔 꼭대기(터치 예상 지점)의 파이썬 좌표 계산
    # 접근 지점(a_pos)에서 Z축으로 딱 10mm 아래가 수학적인 꼭대기입니다.
    top_exp_x = a_pos[0]
    top_exp_y = a_pos[1]
    top_exp_z = a_pos[2] - 10.0

    drl_code = 'set_tool("Tool Weight")\n'
    drl_code += 'set_tcp("GripperDA_v1")\n'
    drl_code += 'set_singularity_handling(1)\n\n'

    drl_code += '# 1. 초기 대기 위치로 이동\n'
    drl_code += 'p_ready = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)\n'
    drl_code += 'movej(p_ready, v=100.0, a=100.0)\n\n'
    
    drl_code += f'p_top = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, 200.0, {a_rx:.2f}, {a_ry:.2f}, {a_rz:.2f})\n'
    drl_code += f'p_approach = posx({a_pos[0]:.2f}, {a_pos[1]:.2f}, {a_pos[2]:.2f}, {a_rx:.2f}, {a_ry:.2f}, {a_rz:.2f})\n'
    drl_code += 'movel(p_top, v=100.0, a=100.0)\n'
    drl_code += 'movel(p_approach, v=50.0, a=100.0)\n'
    drl_code += 'sleep(1.0)\n\n'

    drl_code += 'base_f = get_tool_force(1)\n'
    drl_code += 'zero_tz = base_f[2]\n\n'
    
    drl_code += 'stiff = [3000, 3000, 100, 300, 300, 300]\n'
    drl_code += 'task_compliance_ctrl(stiff, 1)\n\n'
    
    # 바닥 탐색: -15N으로 하강
    drl_code += 'fd_down = [0.0, 0.0, -15.0, 0.0, 0.0, 0.0]\n'
    drl_code += 'fdir = [0, 0, 1, 0, 0, 0]\n'
    drl_code += 'set_desired_force(fd_down, fdir, 1)\n\n'
    
    drl_code += 'while True:\n'
    drl_code += '    cur_f = get_tool_force(1)\n'
    drl_code += '    if abs(cur_f[2] - zero_tz) > 6.0:\n'
    drl_code += '        break\n'
    drl_code += '    sleep(0.001)\n\n'

    # 💡 2. 터치 순간의 실제 위치(pos_touch) 읽기!
    drl_code += 'pos_touch, sol = get_current_posx()\n\n'

    # 💡 3. 표면 훑기용 힘 제어 시작 (툴 좌표계 기준 -10N 유지)
    drl_code += 'release_force()\n'
    drl_code += 'fd_scan = [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]\n'
    drl_code += 'fdir_z = [0, 0, 1, 0, 0, 0]\n'
    drl_code += 'set_desired_force(fd_scan, fdir_z, 1) \n\n'

    # 4. 상대 거리(Offset)를 통한 완벽한 밀착 궤적 생성
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

        # 파이썬 수학 좌표 계산
        sr_x, sr_y, _, s_pos = get_pure_blended_orientation(start_xyz, center, tool_length)
        vr_x, vr_y, vr_z_raw, v_pos = get_pure_blended_orientation(via_xyz, center, tool_length)
        tr_x, tr_y, tr_z_raw, t_pos = get_pure_blended_orientation(tgt_xyz, center, tool_length)

        sr_z = target_start_rz
        vr_z = make_continuous(sr_z, vr_z_raw)
        tr_z = make_continuous(vr_z, tr_z_raw)

        # 💡 [핵심] "예상 꼭대기"에서 각 점들이 얼마나 떨어져 있는지(상대 거리) 계산
        rel_sx = s_pos[0] - top_exp_x
        rel_sy = s_pos[1] - top_exp_y
        rel_sz = s_pos[2] - top_exp_z
        
        rel_vx = v_pos[0] - top_exp_x
        rel_vy = v_pos[1] - top_exp_y
        rel_vz = v_pos[2] - top_exp_z
        
        rel_tx = t_pos[0] - top_exp_x
        rel_ty = t_pos[1] - top_exp_y
        rel_tz = t_pos[2] - top_exp_z

        drl_code += f"# Layer {i} - {direction}\n"
        
        # 💡 "실제로 닿은 점(pos_touch)"에 상대 거리를 더해서 최종 좌표 결정
        drl_code += f"p_start_{i} = posx(pos_touch[0] + ({rel_sx:.2f}), pos_touch[1] + ({rel_sy:.2f}), pos_touch[2] + ({rel_sz:.2f}), {sr_x:.2f}, {sr_y:.2f}, {sr_z:.2f})\n"
        drl_code += f"p_via_{i}   = posx(pos_touch[0] + ({rel_vx:.2f}), pos_touch[1] + ({rel_vy:.2f}), pos_touch[2] + ({rel_vz:.2f}), {vr_x:.2f}, {vr_y:.2f}, {vr_z:.2f})\n"
        drl_code += f"p_tgt_{i}   = posx(pos_touch[0] + ({rel_tx:.2f}), pos_touch[1] + ({rel_ty:.2f}), pos_touch[2] + ({rel_tz:.2f}), {tr_x:.2f}, {tr_y:.2f}, {tr_z:.2f})\n"
        
        if i == 1:
            drl_code += f"movel(p_start_{i}, v=20, a=40, r=10.0)\n"
        else:
            drl_code += f"movel(p_start_{i}, v=40, a=80, r=10.0)\n"
            
        drl_code += f"movec(p_via_{i}, p_tgt_{i}, v=10, a=10, angle=360.0, ori=2)\n\n"

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
            self.get_logger().info('🔔 스캔 시작 신호(02) 수신 완료!')

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
# [5] 메인 제어 흐름 (단일 쓰레드 비동기)
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
                
                drl_code = generate_force_dome_drl(CENTER, REAL_RADIUS, PATH_RADIUS, TOTAL_LAYERS, TOOL_LENGTH)
                
                print("\n🚀 제어기로 DRL 전송 중 (터치 절대 좌표 앵커링 기술 적용됨)...")
                node.send_drl_code(drl_code)
                time.sleep(1.0)
                
                print("⏳ 돔 꼭대기를 찾는 중입니다...")
                
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.01)
                    current_x = get_current_posx()[0][0]
                    if abs(current_x - CENTER[0]) > 5.0:
                        node.publish_state(True)
                        print("🛸 돔 표면 안착 완료! 스캐닝 기록을 시작합니다.")
                        break
                    time.sleep(0.05)
                
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

    except KeyboardInterrupt:
        pass
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