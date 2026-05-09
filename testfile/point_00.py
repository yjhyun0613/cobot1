import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = ""
ROBOT_TCP = ""

VELOCITY = 50
ACC = 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, get_robot_mode, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_MANUAL)
    time.sleep(0.5)
    set_tool(ROBOT_TOOL)
    time.sleep(0.5)
    set_tcp(ROBOT_TCP)
    time.sleep(0.5)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)

def touch_and_record(node):
    print("🧽 Starting Compliance Touch Down...")
    from DSR_ROBOT2 import posx, movej, movel, get_current_posx
    from DSR_ROBOT2 import task_compliance_ctrl, set_desired_force, release_compliance_ctrl

    JReady = [0, 0, 90, 0, 90, 0]
    
    # 목표 위치의 5cm(50mm) 상공 (RX, RY, RZ는 수직 하방 자세 유지)
    X_TARGET = 450
    Y_TARGET = 0
    HOVER_HEIGHT = 50 
    RX, RY, RZ = 0, 180, 0

    p_hover = posx([X_TARGET, Y_TARGET, HOVER_HEIGHT, RX, RY, RZ])

    print("Moving to Hover Position (Z=50mm)...")
    movej(JReady, vel=VELOCITY, acc=ACC)
    
    # 💡 [핵심 수정 부분] 공간 좌표(posx)로 이동할 때는 반드시 'movel'을 사용해야 합니다!
    movel(p_hover, vel=VELOCITY, acc=ACC)
    
    # ==========================================
    # 1️⃣ 순응 제어 켜기 (말랑해지기)
    # ==========================================
    print("1️⃣ Compliance Control ON (Z축 말랑하게)")
    # Z축만 말랑하게 (500), 나머지는 뻣뻣하게 (3000)
    stiffness = [3000, 3000, 500, 3000, 3000, 3000]
    task_compliance_ctrl(stiffness)
    
    # ==========================================
    # 2️⃣ 부드럽게 하강 시작 (Soft Touch)
    # ==========================================
    print("2️⃣ Approaching floor with -5N force...")
    set_desired_force(fd=[0, 0, -5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
    
    prev_z = get_current_posx()[0][2]
    time.sleep(0.5) 
    
    # 💡 [추가된 부분] 연속 멈춤 횟수를 세는 카운터
    stop_count = 0 
    
    while True:
        current_z = get_current_posx()[0][2]
        dz = abs(current_z - prev_z)
        
        if dz < 0.5: 
            # 움직임이 없으면 카운트 증가
            stop_count += 1
        else:
            # 조금이라도 쑥 내려가면 카운트 리셋 (다시 처음부터 감시)
            stop_count = 0 
            
        # 💡 [핵심] 3번 연속(0.3초 동안)으로 움직임이 없어야 진짜 바닥으로 인정!
        if stop_count >= 3: 
            print("✅ Securely touched the surface!")
            break
            
        prev_z = current_z
        time.sleep(0.1)

    # ==========================================
    # 3️⃣ 좌표 기록
    # ==========================================
    # 완전히 안착할 때까지 잠깐 대기
    time.sleep(0.5) 
    hit_pos = get_current_posx()[0]
    print(f"🎯 Recorded Point -> X: {hit_pos[0]:.1f}, Y: {hit_pos[1]:.1f}, Z: {hit_pos[2]:.1f}")
    
    # ==========================================
    # 4️⃣ 순응 제어 해제 및 복귀
    # ==========================================
    print("4️⃣ Compliance Control OFF & Returning...")
    release_compliance_ctrl() 
    
    # 다시 5cm 상공으로 안전하게 후퇴
    movel(p_hover, vel=VELOCITY, acc=ACC)
    movej(JReady, vel=VELOCITY, acc=ACC)
    print("Mission Complete!")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("compliance_touch_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        touch_and_record(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 비정상 종료 시에도 순응 제어가 풀리도록 안전장치
        from DSR_ROBOT2 import release_compliance_ctrl
        try: release_compliance_ctrl()
        except: pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()