import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"   
ROBOT_TCP = "GripperDA_v1"   

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)

def single_soft_touch():
    from DSR_ROBOT2 import movel, posx, get_current_posx, wait
    from DSR_ROBOT2 import task_compliance_ctrl, set_desired_force, release_compliance_ctrl

    # 1. 탐색 시작 위치 (X=450, Y=0, Z=100 상공)
    start_pos = posx([450.0, 0.0, 100.0, 0.0, 180.0, 0.0])
    print("🚀 탐색 위치로 이동 중...")
    movel(start_pos, vel=100, acc=100)

    # 2. 순응 제어 ON (Z축만 말랑하게)
    print("🧽 순응 제어 활성화...")
    task_compliance_ctrl([3000, 3000, 500, 3000, 3000, 3000])
    wait(0.5)

    # 3. Z축 하방으로 15N 힘 가하기 (내려가기 시작)
    print("⬇️ 하강 시작 (15N 힘 적용)...")
    set_desired_force(fd=[0, 0, -15.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
    
    # 4. 바닥 안착 감지 로직
    wait(0.5) 
    prev_z = get_current_posx()[0][2]
    stop_count = 0
    
    while True:
        curr_z = get_current_posx()[0][2]
        if abs(curr_z - prev_z) < 0.5:
            stop_count += 1
        else:
            stop_count = 0
        
        if stop_count >= 3: # 0.3초간 멈추면 바닥으로 인정
            break
        prev_z = curr_z
        wait(0.1)

    # 5. 결과 기록 및 복귀
    hit_pos = get_current_posx()[0]
    print(f"✅ 터치 완료! 측정된 바닥 높이: {hit_pos[2]:.1f}mm")
    
    release_compliance_ctrl() # 힘 제어 해제
    wait(0.2)
    movel(start_pos, vel=100, acc=100) # 다시 상공으로 복귀
    print("🏁 복귀 완료.")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("single_touch_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        single_soft_touch()
    finally:
        from DSR_ROBOT2 import release_compliance_ctrl
        try: release_compliance_ctrl()
        except: pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()