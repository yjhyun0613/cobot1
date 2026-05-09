import rclpy
import DR_init
import time
from std_msgs.msg import Bool

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VEL = [50.0, 50.0]  
ACC = [100.0, 100.0] 

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, set_ref_coord, DR_BASE, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_ref_coord(DR_BASE) 
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)

def draw_simultaneous_orbit(node):
    from DSR_ROBOT2 import posx, movej, movel, movec, DR_BASE
    print("🚀 [동시 진행 모드] 절대 좌표(mod=0)를 기반으로 정확한 원을 그립니다!")

    state_pub = node.create_publisher(Bool, f'/{ROBOT_ID}/checking_state', 10)
    msg = Bool()

    BASE_Z = 200.0
    RX, RY = 150.0, 179.0

    # 💡 정방향(CW) 공전 (RZ 값을 부드럽게 감소시켜 손목 회전 유도)
    p_left_cw   = [400.0, 0.0, BASE_Z, RX, RY, 0.0]      # 시작
    p_top_cw    = [450.0, 50.0, BASE_Z, RX, RY, -90.0]   # 경유1
    p_right_cw  = [500.0, 0.0, BASE_Z, RX, RY, -180.0]   # 도착1   
    p_bottom_cw = [450.0, -50.0, BASE_Z, RX, RY, -270.0] # 경유2
    p_end_cw    = [400.0, 0.0, BASE_Z, RX, RY, -360.0]   # 도착2

    # 💡 역방향(CCW) 꼬임 풀기 (RZ 값을 부드럽게 증가)
    p_left_ccw   = [400.0, 0.0, BASE_Z, RX, RY, 0.0]     
    p_bottom_ccw = [450.0, -50.0, BASE_Z, RX, RY, 90.0]  
    p_right_ccw  = [500.0, 0.0, BASE_Z, RX, RY, 180.0]   
    p_top_ccw    = [450.0, 50.0, BASE_Z, RX, RY, 270.0]  
    p_end_ccw    = [400.0, 0.0, BASE_Z, RX, RY, 360.0]   

    JReady = [0, 0, 90, 0, 90, 0]

    while rclpy.ok():
        print("\n--- 새로운 사이클 시작 ---")
        movej(JReady, vel=100, acc=100)
        
        # 시작점(절대 좌표)으로 정확히 이동
        movel(p_left_cw, vel=VEL, acc=ACC, ref=DR_BASE)
        time.sleep(0.5)

        # -----------------------------------------------------------
        # [Step 1] 동시 진행 (정방향 CW)
        # -----------------------------------------------------------
        print("⭕ 정방향 공전 (원 궤적 이동 + 툴 회전 동시 진행 중...)")
        msg.data = True
        state_pub.publish(msg)
        
        # 💡 문제의 mod=1 제거!! 로봇이 각도(RZ) 변화를 감지하고 스스로 툴을 돌립니다.
        movec(p_top_cw, p_right_cw, vel=VEL, acc=ACC, angle=360.0, ref=DR_BASE)
        # movec(p_bottom_cw, p_end_cw, vel=VEL, acc=ACC, angle=180.0, ref=DR_BASE)
        
        msg.data = False
        state_pub.publish(msg)

        # -----------------------------------------------------------
        # [Step 2] 꼬임 방지를 위해 상승 후 복귀
        # -----------------------------------------------------------
        print("⬆️ 케이블 풀기 준비 (상승)...")
        p_lift = list(p_end_cw)
        p_lift[2] += 10.0
        movel(p_lift, vel=VEL, acc=ACC, ref=DR_BASE)
        time.sleep(0.5)

        movel(p_left_ccw, vel=VEL, acc=ACC, ref=DR_BASE)

        # -----------------------------------------------------------
        # [Step 3] 동시 진행 (역방향 CCW) - 케이블 풀기
        # -----------------------------------------------------------
        print("🔄 역방향 공전 (반대로 돌며 케이블 푸는 중...)")
        msg.data = True
        state_pub.publish(msg)
        
        # 💡 여기도 mod=1 완전 제거
        movec(p_bottom_ccw, p_right_ccw, vel=VEL, acc=ACC, angle=360, ref=DR_BASE)
        # movec(p_top_ccw, p_end_ccw, vel=VEL, acc=ACC, angle=180.0, ref=DR_BASE)
        
        msg.data = False
        state_pub.publish(msg)

        print("✅ 한 사이클 완료. 3초 후 재시작...")
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("simultaneous_orbit_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        draw_simultaneous_orbit(node)
    except KeyboardInterrupt:
        print("\n🛑 안전하게 마무리합니다 (Ctrl+C).")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()