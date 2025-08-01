import rclpy
import DR_init
import time
import numpy as np

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0
target = [0, 0, 0]  

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_geer", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            movej,
            movel,
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            wait,
            move_periodic,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE
        )
        from DR_common2 import posj, posx
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # 초기 위치
    init_pos = posj(0, 0, 90, 0, 90, 0)

    # Gear 1
    gear_1_1 = posj(-9.46, 36.46, 45.42, -0.07, 97.85, -9.92)
    gear_1_2 = posx(603.15, -93.48, 279.6, 173.24, -179.56, 172.7)
    gear_1_3 = posj(-26.58, 3.41, 92.68, -0.79, 84.53, -28.26)
    gear_1_4 = posx(348.94, -169.98, 279.59, 18.1, -178.99, 16.22)
    # Gear 2
    gear_2_1 = posj(-17.07, 48.51, 27.84, -0.09, 103.31, -17.86)
    gear_2_2 = posx(644.48, -191.11, 274.05, 169.32, -179.73, 168.5)
    gear_2_3 = posj(-32.59, 16.5, 78.98, -1.19, 85.23, -34.59)
    gear_2_4 = posx(406.54, -256.97, 279.38, 18.71, -178.63, 16.51)
    # Gear 3
    gear_3_1 = posj(-18.91, 28.84, 59.78, -0.09, 91.22, -19.9)
    gear_3_2 = posx(539.59, -178.45, 277.12, 160.42, -179.84, 159.43)
    gear_3_3 = posj(-41.01, 4.95, 91.23, -1.43, 84.3, -42.41)
    gear_3_4 = posx(302.55, -261.24, 279.26, 21.49, -178.53, 19.86)
    # Gear 4
    gear_4_1 = posj(-14.74, 37.59, 47.66, -0.18, 94.95, -16.11)
    gear_4_2 = posx(601.1, -152.62, 276.46, 175.59, 179.59, 174.25)
    gear_4_3 = posj(-32.26, 7.79, 81.07, -1.79, 91.84, -33.94)
    gear_4_4 = posx(354.7, -226.2, 284.01, 36.48, -178.24, 34.8)


    # ✅ 공통 처리 함수
    def process_gear_sequence(j1, x1, j2, x2):
        """
        기어 작업 시퀀스를 처리하는 함수.
        """
        # 접근 위치로 이동
        movej(j1, vel=30, acc=30)


        # 픽 위치 이동
        movel(x1, vel=70, acc=60)
    
        # 픽 동작 (흡착 등)
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        wait(0.5)

        # 되돌아오기
        movej(j1, vel=30, acc=30)
    

        # 조립 위치 접근
        movej(j2, vel=30, acc=30)


        # 조립 위치 이동
        movel(x2, vel=70, acc=60)
    

        # 언로딩
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)

        # 복귀
        movej(j2, vel=30, acc=30)
        
    # ✅ 공통 처리 함수
    def process_gear_sequence_last(j1, x1, j2, x2):

        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        """
        기어 작업 시퀀스를 처리하는 함수.
        """
        # 접근 위치로 이동
        movej(j1, vel=30, acc=30)


        # 픽 위치 이동
        movel(x1, vel=70, acc=60)
    
        # 픽 동작 (흡착 등)
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        wait(0.5)

        # 되돌아오기
        movej(j1, vel=30, acc=30)
    

        # 조립 위치 접근
        movej(j2, vel=30, acc=30)


        # 조립 위치 이동
        gear_4_4[2] += 20
        movel(gear_4_4, vel=60, acc=60)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)
        fd = [0, 0, -10, 0, 0, 0]
        fctrl_dir= [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL) 

        force_check = 10.0
        state = 0
        force_condition = check_force_condition(DR_AXIS_Z, max=force_check)
        print(f'{force_condition} 1')
        while (True):
            force_condition = check_force_condition(DR_AXIS_Z, max=force_check)
            # print(f'{force_condition} 2')
            if force_condition == -1 and state == 0:
                print(f'{force_condition} 3')
                move_periodic(
                    amp=[0, 0, 0, 0, 0, 10],     # 10 deg 진폭 (Joint 6)
                    period=1,                  # 0.5초 주기
                    atime=0.5,
                    repeat=10,         # 약 3초 반복
                    ref=DR_BASE,
                )    
                state = 1
            elif force_condition == 0 and state ==1:
                print(f'{force_condition} 4')
                # movel(gear_4_4, vel=60, acc=60)
                break
                
    # movel(x2, vel=30, acc=30)
        release_force()
        wait(0.5)
        release_compliance_ctrl()

        # 언로딩
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)

        # 복귀
        movej(j2, vel=30, acc=30)

    # ✅ 메인 루틴 시작

    # 디지털 초기 설정
    set_digital_output(1, OFF)
    set_digital_output(2, OFF)

    # 초기 위치로 이동
    movej(init_pos, vel=30, acc=30)
    #while not motion_done(): wait(0.1)

    # 기어 1 작업
    process_gear_sequence(gear_1_1, gear_1_2, gear_1_3, gear_1_4)

    # 기어 2 작업
    process_gear_sequence(gear_2_1, gear_2_2, gear_2_3, gear_2_4)
    #
    ## 기어 3 작업
    process_gear_sequence(gear_3_1, gear_3_2, gear_3_3, gear_3_4)

    ## 기어 4 작업
    process_gear_sequence_last(gear_4_1, gear_4_2, gear_4_3, gear_4_4)