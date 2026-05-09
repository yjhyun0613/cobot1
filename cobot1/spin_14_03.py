import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from dsr_msgs2.srv import DrlStart
from dsr_msgs2.msg import RobotStop
import DR_init
import time
import math

class ScanRunnerNode(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)
        self.state_pub = self.create_publisher(Bool, f'/{namespace}/checking_state', 10)
        self.progress_pub = self.create_publisher(String, f'/{namespace}/progress_status', 10)
        self.stop_pub = self.create_publisher(RobotStop, f'/{namespace}/stop', 10) 
        
        self.trigger_scan = False
        self.is_scanning = False 
        
        self.progress_sub = self.create_subscription(
            String,
            f'/{namespace}/progress_status',
            self.progress_callback,
            10
        )
        
        service_name = f'/{namespace}/drl/drl_start'
        self.drl_client = self.create_client(DrlStart, service_name)
        
        while not self.drl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} 서비스 대기 중...')
            
        self.get_logger().info('✅ 토픽 퍼블리셔 및 DRL 서비스 클라이언트 준비 완료')

    def progress_callback(self, msg):
        if msg.data == "02":
            if not self.is_scanning and not self.trigger_scan:
                self.get_logger().info('수신된 신호 [02]: 스캔 시퀀스를 시작합니다.')
                self.trigger_scan = True

    def publish_state(self, state: bool):
        msg = Bool()
        msg.data = state
        self.state_pub.publish(msg)
        status_text = "🟢 ON (데이터 기록 시작)" if state else "🔴 OFF (데이터 기록 종료)"
        self.get_logger().info(f'신호 전송: {status_text}')

    def publish_completion(self):
        msg = String()
        msg.data = "03"
        self.progress_pub.publish(msg)
        self.get_logger().info('신호 전송: 🏁 [03] 스캔 전체 완료')

    def send_drl_code(self, drl_script):
        self.get_logger().info('DRL 코드를 전송합니다...')
        req = DrlStart.Request()
        req.robot_system = 0        
        req.code = drl_script       
        self.drl_client.call_async(req)

def main(args=None):
    # 💡 설정 변수
    TOTAL_LAYERS = 5    
    CENTER_X = 419.15  
    CENTER_Y = 74.45    
    CENTER_Z = 25.50  # 💡 [수정] 오차를 없애기 위해 실제 바닥 높이로 롤백
    MAX_TILT_ANGLE = 30.0
    TOOL_LENGTH = 30.0      

    ROBOT_ID = "dsr01" 
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = "m0609"
    
    rclpy.init(args=args)
    node = ScanRunnerNode('scan_runner_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
     
    try:
        from DSR_ROBOT2 import (
            set_robot_mode, ROBOT_MODE_AUTONOMOUS, ROBOT_MODE_MANUAL,
            set_tool, set_tcp, movej, movel, get_robot_state,
            task_compliance_ctrl, set_desired_force, release_force,
            get_current_posx, release_compliance_ctrl
        )

        print("\n로봇 초기화 중...")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool("Tool Weight")
        set_tcp("GripperDA_v1")
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(1.0) 

        print("\n🔄 대기 모드: /dsr01/progress_status 토픽에서 '02' 신호를기다립니다...")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if node.trigger_scan:
                node.is_scanning = True
                node.trigger_scan = False
                
                # ==========================================================
                # [Phase 1] 돔 최고점 자동 탐색
                # ==========================================================
                print("\n🚀 [Phase 1] 돔 최고점 자동 탐색 (힘 제어 하강)...")
                p_ready = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
                rx_top, ry_top, rz_top = 0.0, 179.0, 270.0
                
                approach_z = CENTER_Z + 70.0 + TOOL_LENGTH
                p_top = [CENTER_X, CENTER_Y, approach_z, rx_top, ry_top, rz_top]

                movej(p_ready, vel=100.0, acc=100.0)
                movel(p_top, vel=50.0, acc=100.0) 
                time.sleep(1.0)

                task_compliance_ctrl([300, 300, 100, 300, 300, 300], 0)
                time.sleep(0.5)
                
                print("⏬ 스르륵 하강 시작...")
                set_desired_force([0, 0, -15, 0, 0, 0], [0, 0, 1, 0, 0, 0], 0)
                time.sleep(1.0) 
                
                start_z = get_current_posx()[0][2]
                prev_z = start_z
                stop_count = 0 
                
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.01)
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
                        node.stop_pub.publish(stop_msg)
                        print(f"🛑 돔 표면 감지 확정! (최종 하강 높이: {curr_z:.2f}mm)")
                        break
                        
                    prev_z = curr_z
                    
                release_compliance_ctrl()
                time.sleep(1.0) 
                
                pos_touch = get_current_posx()[0]
                flange_z = pos_touch[2] 
                # 💡 TCP가 그리퍼 기준이므로 TOOL_LENGTH를 빼야 진짜 돔 꼭대기 나옴 (정확함)
                true_top_z = flange_z - TOOL_LENGTH 
                measured_radius = true_top_z - CENTER_Z
                
                print("-" * 45)
                print(f"🎯 돔 크기 자동 측정 완료!")
                print(f"   - 로봇 플랜지 멈춘 높이 : {flange_z:.2f} mm")
                print(f"   - 툴 길이 반영 돔 표면 : {true_top_z:.2f} mm")
                print(f"   👉 계산된 실제 반경(R)  : {measured_radius:.2f} mm")
                print("-" * 45)

                R = measured_radius

                # ==========================================================
                # [긴급 안전 조치] 충돌 방지를 위한 수직 상승
                # ==========================================================
                print("\n⬆️ 돔 파고듦 방지: 안전 높이로 수직 상승합니다...")
                safe_z_up = CENTER_Z + R + 50.0  
                p_safe_up = [CENTER_X, CENTER_Y, safe_z_up, rx_top, ry_top, rz_top]
                movel(p_safe_up, vel=50.0, acc=100.0)
                time.sleep(1.0) 

                # ==========================================================
                # [Phase 2: 사전 이동] 💡 삼각함수 오프셋 계산 적용 (Python)
                # ==========================================================
                first_layer_tilt = MAX_TILT_ANGLE * (1.0 / TOTAL_LAYERS)
                angle_rad = math.radians(90.0 * (1.0 - 1.0/TOTAL_LAYERS))
                curr_z = CENTER_Z + R * math.sin(angle_rad)
                curr_r = R * math.cos(angle_rad)
                
                # 💡 [핵심 해결] 기울어진 각도만큼 그리퍼 좌표를 허공으로 빼주는 오프셋 적용!
                layer_tilt_rad_py = math.radians(first_layer_tilt)
                offset_z_py = TOOL_LENGTH * math.cos(layer_tilt_rad_py)  # 위로 당기기
                offset_r_py = TOOL_LENGTH * math.sin(layer_tilt_rad_py)  # 밖으로 당기기
                
                safe_z_py = curr_z + offset_z_py
                safe_r_py = curr_r + offset_r_py
                
                ry_1 = -179.0 + first_layer_tilt
                p_start_1 = [CENTER_X + safe_r_py, CENTER_Y, safe_z_py, 0.0, ry_1, 270.0]
                
                print(f"\n📍 1층 스캔 지점(오프셋 적용 완료)으로 대기 이동...")
                movel(p_start_1, vel=40.0, acc=80.0)
                time.sleep(0.5)

                node.publish_state(True)
                time.sleep(0.2) 
                
                # ==========================================================
                # [DRL 실행] 💡 삼각함수 오프셋 계산 적용 (DRL)
                # ==========================================================
                scan_drl_code = f"""
R = {R}+1    
TOTAL_LAYERS = {TOTAL_LAYERS}    
CENTER_X = {CENTER_X}   
CENTER_Y = {CENTER_Y}    
CENTER_Z = {CENTER_Z} 
MAX_ANGLE = {MAX_TILT_ANGLE}
TOOL_LENGTH = {TOOL_LENGTH}

set_singularity_handling(1)
stiff = [500, 500, 500, 100, 100, 100]
task_compliance_ctrl(stiff)

rx = 0.0

for i in range(1, TOTAL_LAYERS + 1):
    angle_rad = d2r(90.0 * (1.0 - float(i)/TOTAL_LAYERS))
    curr_z = CENTER_Z + R * sin(angle_rad)
    curr_r = R * cos(angle_rad)
    
    layer_tilt = MAX_ANGLE * (float(i)/TOTAL_LAYERS)
    ry = -180 + layer_tilt  
    
    # 💡 [핵심 해결] DRL 내부에서도 실시간 오프셋 계산!
    layer_tilt_rad = d2r(layer_tilt)
    offset_z = TOOL_LENGTH * cos(layer_tilt_rad) # 위로 당기기
    offset_r = TOOL_LENGTH * sin(layer_tilt_rad) # 밖으로 당기기
    
    safe_z = curr_z + offset_z
    safe_r = curr_r + offset_r
    
    if (i % 2) == 1:
        rz = 270.0
        p_via1_y = CENTER_Y - safe_r  
        p_via2_y = CENTER_Y + safe_r  
    else:
        rz = -90.0
        p_via1_y = CENTER_Y + safe_r  
        p_via2_y = CENTER_Y - safe_r  

    # 💡 순수 돔 표면(curr_z)이 아닌, 툴 길이만큼 물러난 안전 좌표(safe_z, safe_r) 적용
    p_start = posx(CENTER_X + safe_r, CENTER_Y, safe_z, rx, ry, rz)
    p_via1  = posx(CENTER_X, p_via1_y, safe_z, rx, ry, rz)
    p_mid   = posx(CENTER_X - safe_r, CENTER_Y, safe_z, rx, ry, rz)
    p_via2  = posx(CENTER_X, p_via2_y, safe_z, rx, ry, rz)
    p_end   = posx(CENTER_X + safe_r, CENTER_Y, safe_z, rx, ry, rz)

    movel(p_start, v=40, a=80, r=10.0)
    
    movec(p_via1, p_mid, v=20, a=20, angle=180.0, r=10.0, ori=2)
    movec(p_via2, p_end, v=20, a=20, angle=180.0, r=10.0, ori=2)

release_compliance_ctrl()
"""
                node.send_drl_code(scan_drl_code)
                
                print("\n🛸 스캐닝 진행 중...")
                time.sleep(2.0) 
                
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
                print("\n✅ 모든 프로세스 및 데이터 기록 종료.")
                
                print("\n🔄 다음 '02' 신호를 대기합니다...")
                node.is_scanning = False 

    except Exception as e:
        print(f"\n🚨 에러 발생: {e}")
        node.publish_state(False) 
        
    finally:
        print("✅ 노드를 안전하게 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()