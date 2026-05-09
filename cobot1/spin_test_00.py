import rclpy
import time
import DR_init
from dsr_msgs2.msg import RobotStop # 💡 정지 신호를 쏘기 위한 메시지 임포트

# =================================================================
# [1] 환경 설정
# =================================================================
ROBOT_ID    = "dsr01" 
ROBOT_MODEL = "m0609"

def main(args=None):
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    rclpy.init(args=args)
    node = rclpy.create_node('touch_detector')
    DR_init.__dsr__node = node
    
    # 💡 로봇을 즉시 멈추기 위한 퍼블리셔 (Publisher) 생성
    stop_pub = node.create_publisher(RobotStop, f'/{ROBOT_ID}/stop', 10)
    
    try:
        # 에러가 나던 stop 임포트 제거
        from DSR_ROBOT2 import (
            set_robot_mode, movej, amovel, 
            task_compliance_ctrl, release_compliance_ctrl, get_current_posx
        )
        
        # 자동 모드 설정
        set_robot_mode(1)
        time.sleep(1.0)
        
        # 1. 홈 위치로 이동
        print("\n🚀 1. 홈 위치로 이동합니다...")
        movej([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=60.0, acc=60.0)
        time.sleep(1.0)
        
        start_pos, _ = get_current_posx()
        print(f"   현재 시작 Z 높이: {start_pos[2]:.2f} mm")
        
        # 2. 충돌 대비 순응 제어 (Z축만 부드러운 스프링 모드)
        print("\n🛡️ 2. 순응 제어(스프링 모드) 활성화...")
        task_compliance_ctrl([3000, 3000, 100, 300, 300, 300])
        time.sleep(0.5)
        
        # 3. 하강 목표지점 설정 (현재 위치에서 20cm 아래로)
        target_pos = list(start_pos)
        target_pos[2] -= 200.0  
        
        # 파이썬 비동기 이동 명령 
        print("\n⏬ 3. 하강 시작 (바닥 감지 중)...")
        amovel(target_pos, vel=15.0, acc=30.0)
        time.sleep(0.5) # 로봇이 출발할 때까지 잠깐 대기
        
        # 4. 정지 감지 루프
        prev_z = get_current_posx()[0][2]
        stop_count = 0
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.1) # 0.1초마다 검사
            
            curr_pos, _ = get_current_posx()
            curr_z = curr_pos[2]
            
            # 0.1초 동안 0.2mm 이하로 움직였다면 (어딘가에 막혀서 멈춤)
            diff = abs(prev_z - curr_z)
            if diff < 0.2:
                stop_count += 1
            else:
                stop_count = 0 # 계속 내려가고 있으면 초기화
                
            # 3번 연속 (약 0.3초) 멈춰있으면 바닥 감지 확정!
            if stop_count >= 3:
                # 💡 함수 대신 ROS2 통신으로 Quick Stop 명령 발송!
                stop_msg = RobotStop()
                stop_msg.stop_mode = 2 
                stop_pub.publish(stop_msg)
                
                print(f"🛑 바닥 감지! 로봇을 정지합니다.")
                break
                
            prev_z = curr_z
            
        # 로봇이 완전히 멈출 때까지 대기 후 스프링 모드 해제
        time.sleep(1.0)
        release_compliance_ctrl()
        
        # 5. 최종 좌표 출력
        final_pos, _ = get_current_posx()
        print("\n" + "=" * 40)
        print("📍 터치 지점 최종 좌표 데이터")
        print("=" * 40)
        print(f" X  : {final_pos[0]:.2f} mm")
        print(f" Y  : {final_pos[1]:.2f} mm")
        print(f" Z  : {final_pos[2]:.2f} mm (최종 높이)")
        print(f" Rx : {final_pos[3]:.2f} 도")
        print(f" Ry : {final_pos[4]:.2f} 도")
        print(f" Rz : {final_pos[5]:.2f} 도")
        print("=" * 40 + "\n")
        
    except Exception as e:
        print(f"\n🚨 에러 발생: {e}")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()