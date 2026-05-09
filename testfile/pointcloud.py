import rclpy
import DR_init
import time
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"   
ROBOT_TCP = "GripperDA_v1"   

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot_tcp():
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
    print("\n--- 로봇 Tool / TCP 초기화 시작 ---")
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)
    print(f"적용된 TCP: {get_tcp()}, Tool: {get_tool()}")
    print("--- 초기화 완료 ---\n")

def run_safe_pointcloud(node):
    from DSR_ROBOT2 import (
        movel, amovel, posx, get_current_posx, wait, DR_BASE,
        task_compliance_ctrl, set_desired_force, release_compliance_ctrl, 
        check_motion, get_tool_force
    )

    # =========================================================
    # 1. 파라미터 세팅
    # =========================================================
    X_START, X_END, X_STEP = 423, 506, 10   
    Y_START, Y_END, Y_STEP = 69, 147, 10    
    SAFE_Z = 106.0         
    TARGET_Z = 43.56       
    
    # 💡 [핵심 수정 1] 실제 로봇의 관성 노이즈를 이기기 위해 기준값을 30.0N으로 상향!
    # (만약 그래도 허공에서 멈추면 이 값을 35.0 이나 40.0으로 더 올려주세요)
    SOFT_TOUCH_FORCE = 30.0  
    
    POS_A, POS_B, POS_C = 174.81, 179.88, 174.43

    point_cloud = []

    print(f"탐색 시작점 상단의 안전 고도({SAFE_Z})로 정렬합니다.")
    pos_init = posx(X_START, Y_START, SAFE_Z, POS_A, POS_B, POS_C)
    movel(pos_init, vel=100, acc=100)
    wait(1.0) 

    # =========================================================
    # 2. 메인 스캔 루프
    # =========================================================
    for x in range(X_START, X_END + 1, X_STEP):
        for y in range(Y_START, Y_END + 1, Y_STEP):
            
            pos_safe = posx(x, y, SAFE_Z, POS_A, POS_B, POS_C)
            movel(pos_safe, vel=100, acc=100)
            
            # 순응 제어 및 영점 잡기 (기계적 딜레이 고려 2.0초)
            task_compliance_ctrl([3000, 3000, 300, 300, 300, 300])
            wait(2.0) 
            base_f = get_tool_force(DR_BASE)[2]
            
            # 하강 전 관절 텐션 완화를 위해 아주 짧게 대기
            wait(0.2) 
            
            # 💡 [핵심 수정 2] 출발 가속도를 대폭 낮춤 (acc=5 -> acc=1)
            # 아주 부드럽고 묵직하게 스르륵 내려가도록 유도합니다.
            amovel(posx(x, y, TARGET_Z - 5.0, POS_A, POS_B, POS_C), vel=5, acc=1) 
            
            # 💡 [핵심 수정 3] 출발 직후 관성 노이즈를 무시하는 시간을 길게 확보 (0.2 -> 0.5초)
            wait(0.5) 
            
            touched = False
            hit_count = 0
            
            while check_motion() == 1:
                cur_fz = get_tool_force(DR_BASE)[2]
                force_delta = abs(cur_fz - base_f)
                curr_p, _ = get_current_posx()
                curr_z = curr_p[2]
                
                # 💡 [디버깅] 허공에서 힘이 얼마나 튀는지 눈으로 직접 확인!
                # 이 숫자가 SOFT_TOUCH_FORCE(현재 30.0)보다 높게 튀면 가짜 터치가 일어납니다.
                print(f"현재 힘 변화량(Delta): {force_delta:.2f}N") 
                
                # Hard Stop: 지정된 한계 고도(TARGET_Z)를 넘어가면 무조건 강제 정지
                if curr_z <= TARGET_Z:
                    print("⚠️ 한계 고도 도달! 강제 정지합니다.")
                    movel(curr_p, vel=100, acc=100) 
                    touched = False
                    break

                # 정상적인 센서 터치 감지 로직
                if force_delta > SOFT_TOUCH_FORCE: 
                    hit_count += 1
                else:
                    hit_count = 0 
                
                # 12번 연속으로 닿아야 인정 (약 0.12초간 지속적인 힘)
                if hit_count >= 12:
                    movel(curr_p, vel=100, acc=100) 
                    touched = True
                    break
                
                wait(0.01)
            
            wait(0.5) # 정지 후 흔들림 방지 대기
            final_pos, _ = get_current_posx()
            px, py, pz = final_pos[0], final_pos[1], final_pos[2]
            
            release_compliance_ctrl()
            wait(0.5) # 제어 해제 후 관절이 다시 뻣뻣해질 시간 확보
            
            # 데이터 분류 기록
            if touched: 
                point_cloud.append([px, py, pz])
                print(f"✅ 물체 표면 -> X: {px:.1f}, Y: {py:.1f}, Z: {pz:.1f}\n")
            else:
                point_cloud.append([px, py, pz])
                print(f"🟦 바닥 도달(안전 정지) -> X: {px:.1f}, Y: {py:.1f}, Z: {pz:.1f}\n")

            # 다음 측정을 위해 안전 고도로 복귀
            movel(pos_safe, vel=100, acc=100)

    # =========================================================
    # 3. 렌더링 파트
    # =========================================================
    print("모든 스캔 완료! 3D 렌더링 시작...")
    if point_cloud:
        xs = [p[0] for p in point_cloud]
        ys = [p[1] for p in point_cloud]
        zs = [p[2] for p in point_cloud]

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')
        
        scatter = ax.scatter(xs, ys, zs, c=zs, cmap='viridis', marker='o', s=20)

        max_range = np.array([np.ptp(xs), np.ptp(ys), np.ptp(zs)]).max() / 2.0
        mid_x = (np.max(xs) + np.min(xs)) * 0.5
        mid_y = (np.max(ys) + np.min(ys)) * 0.5
        mid_z = (np.max(zs) + np.min(zs)) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        ax.set_title('Safe Point Cloud Scan Result')
        ax.set_xlabel('Robot X (mm)')
        ax.set_ylabel('Robot Y (mm)')
        ax.set_zlabel('Height Z (mm)')
        fig.colorbar(scatter, ax=ax, pad=0.1, label='Z Height (mm)')
        
        plt.savefig('pointcloud_result.png', dpi=300)
        print("📸 3D Map image saved as 'pointcloud_result.png'")
        plt.show() 
    else:
        print("탐지 데이터 없음.")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('safe_point_cloud', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot_tcp()
        run_safe_pointcloud(node)
    except KeyboardInterrupt:
        print("\n사용자가 강제 종료했습니다. 안전 조치를 실행합니다.")
    finally:
        # 비상 종료 시 무조건 순응 제어 해제
        print("--- 🛡️ 순응 제어 강제 해제 및 종료 ---")
        from DSR_ROBOT2 import release_compliance_ctrl
        try:
            release_compliance_ctrl()
        except Exception as e:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()