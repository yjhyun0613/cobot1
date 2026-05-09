#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = ""
ROBOT_TCP = ""

VELOCITY = [100.0, 30.0]  # [선속도 mm/s, 각속도 deg/s] - 각속도 제어가 중요하므로 리스트 형태로 변경
ACC = [100.0, 50.0]       # [선가속도 mm/s2, 각가속도 deg/s2]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, get_robot_mode, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_MANUAL)
    # set_tool(ROBOT_TOOL) # 툴이 없으면 주석 처리하거나 에러 무시
    # set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)

def draw_stacked_circles_with_radial(node):
    print("🌪️ 원통 주변 회전 시작 (Z: 150mm -> 300mm, 원주 구속 적용)...")
    from DSR_ROBOT2 import posx, movej, movel, movec, DR_BASE

    JReady = [0, 0, 90, 0, 90, 0]

    print("대기 위치로 이동 중...")
    movej(JReady, vel=100, acc=100)

    for current_z in range(150, 301, 5):
        print(f"📍 현재 레이어: Z = {current_z} mm")

        p_start  = posx([500,  0, current_z, 180, 180, 180])
        p_target = posx([450,  0, current_z, 180, 180, 180])

        # 💡 정방향과 역방향을 위한 경유점을 각각 만듭니다. (Y축 방향 반대)
        p_via_fwd = posx([475, 25, current_z, 180, 180, 180])  # Y가 +25
        p_via_bwd = posx([475, -25, current_z, 180, 180, 180]) # Y가 -25

        # 1. 시작점으로 이동
        movel(p_start, vel=VELOCITY, acc=ACC)
        
        # 2. 정방향 360도 회전 (p_via_fwd 사용, angle은 양수 360)
        print("   >>> 정방향 360도 회전")
        movec(p_via_fwd, p_target, vel=VELOCITY, acc=ACC, angle=360, ref=DR_BASE, ori=2)
        
        time.sleep(0.5)

        # 3. 역방향 360도 회전 (p_via_bwd 사용, angle은 역시 양수 360)
        print("   <<< 역방향 360도 회전")
        movec(p_via_bwd, p_target, vel=VELOCITY, acc=ACC, angle=360, ref=DR_BASE, ori=2)
        
        time.sleep(0.5)

    print("✅ 모든 레이어 완료! (Z = 300mm)")
    
    print("홈 위치로 복귀 중...")
    movej(JReady, vel=100, acc=100)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("cylinder_orbit_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        draw_stacked_circles_with_radial(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()