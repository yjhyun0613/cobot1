import rclpy
import DR_init
import time
import os

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = ""  # 실제 볼 캐스터나 프로브를 달면 무게/길이 입력 필수!
ROBOT_TCP = ""

# 💡 정밀한 탐색을 위해 속도와 가속도를 1/2 수준으로 줄입니다.
VELOCITY = 50 
ACC = 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, get_robot_mode, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)

def trace_and_detect_hole(node):
    print("🔍 Starting Surface Tracing & Hole Detection...")
    from DSR_ROBOT2 import posx, movej, movel, amovel, check_motion, get_current_posx
    from DSR_ROBOT2 import task_compliance_ctrl, set_desired_force, release_compliance_ctrl

    JReady = [0, 0, 90, 0, 90, 0]
    
    # 바닥면(대략 Z=150)보다 살짝 높은 허공(Z=200)을 시작점으로 잡습니다.
    p_start_hover = posx([500, 0, 50, 150, 179, 150]) 
    p_target = posx([300, 0, 50, 150, 179, 150]) # X축 방향으로 200mm 전진 (목표)

    print("Moving to Hover Position...")
    movej(JReady, vel=100, acc=100)
    movel(p_start_hover, vel=VELOCITY, acc=ACC)
    
    # ==========================================
    # 1️⃣ 순응 제어 켜기 (말랑해지기)
    # ==========================================
    print("1️⃣ Compliance Control ON (Z축 말랑하게 변경)")
    # [X, Y, Z, Rx, Ry, Rz] 강성 세팅 
    # X, Y는 흔들리지 않게 3000으로 뻣뻣하게 잡고, Z축은 500으로 말랑하게 풉니다.
    stiffness = [3000, 3000, 500, 3000, 3000, 3000]
    task_compliance_ctrl(stiffness)
    
    # ==========================================
    # 2️⃣ 바닥 찾기 (Soft Approach)
    # ==========================================
    print("2️⃣ Approaching floor with 5N force...")
    # Base 좌표계 기준 Z축(dir=[0,0,1]) 방향으로 -5N(아래로) 힘을 가합니다.
    set_desired_force(fd=[0, 0, -5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
    
    # 로봇이 5N의 힘으로 스르륵 내려가서 바닥에 안착할 때까지 2초 정도 충분히 기다려줍니다.
    time.sleep(2.0) 
    
    # ==========================================
    # 3️⃣ 경사면 훑으며 횡단 시작
    # ==========================================
    print("3️⃣ Tracing surface towards Target...")
    # 비동기 명령: 옆으로 가라고 명령하지만, 순응 제어가 켜져 있어서 Z축은 알아서 경사를 탑니다.
    amovel(p_target, vel=VELOCITY, acc=ACC)
    
    # ==========================================
    # 4️⃣ 실시간 구멍 감지 루프 (100Hz) ⭐️
    # ==========================================
    print("4️⃣ 100Hz Hole Detection Monitoring is Active!")
    prev_pos = get_current_posx()[0]
    prev_z = prev_pos[2]
    
    while check_motion() == 1:
        current_pos = get_current_posx()[0]
        current_z = current_pos[2]
        
        # 0.01초(10ms) 동안 Z축이 얼마나 변했는지(변화량) 계산
        dz = current_z - prev_z 
        
        # [핵심 로직] 정상적인 경사면 하강이라면 0.01초에 0.5mm 이상 떨어질 수 없습니다.
        # 만약 순간적으로 1.5mm 이상 푹 꺼진다면, 바닥이 사라진 구멍(허당)입니다!
        if dz < -1.5: 
            print(f"🚨 [경고] 구멍 감지! 푹 꺼짐 발생 (고도 하락량: {dz:.2f}mm)")
            print("🛑 로봇을 즉시 정지합니다!")
            os.system("ros2 service call /dsr01/system/stop dsr_msgs2/srv/Stop '{stop_mode: 1}'") # ✅ 이 부분으로 교체!
            break
        
        prev_z = current_z
        time.sleep(0.01) # 100Hz 주기
        
    # ==========================================
    # 종료 처리
    # ==========================================
    print("✅ Tracing finished or stopped.")
    release_compliance_ctrl() # 힘 제어를 끄고 다시 원래의 뻣뻣한 로봇으로 복귀
    print("Compliance Control OFF.")
    
    # 팁이 걸려있을 수 있으니 수직으로 살짝 들어 올림
    print("Moving back to safety...")
    time.sleep(1)
    movel(p_start_hover, vel=VELOCITY, acc=ACC)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("detect_hole_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        trace_and_detect_hole(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 전 혹시 모를 안전을 위해 순응 제어 강제 해제
        from DSR_ROBOT2 import release_compliance_ctrl
        try: release_compliance_ctrl() 
        except: pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()