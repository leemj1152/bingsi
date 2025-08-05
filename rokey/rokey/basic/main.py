import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 50, 50
prePosCupX = None
  

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_getting_position", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, get_current_posx, check_force_condition, release_compliance_ctrl,
            movel, set_digital_output, set_tool, set_tcp, release_force,
            task_compliance_ctrl, set_desired_force, DR_AXIS_Z, DR_FC_MOD_REL,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    ON, OFF = 1,0
   

    def wait(sec):
        time.sleep(sec)

    def full_grip():
        set_digital_output(1,ON)
        set_digital_output(2,ON)       
        wait(1)

    def release():
        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        wait(1)
        
    def normal_grip():
        set_digital_output(1,OFF)
        set_digital_output(2,OFF)
        wait(2)

    

    def force_pick_cup():
        global prePosCupX
        if prePosCupX == None:

            normal_grip()
            print("[Force Mode] Activate compliance")
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            wait(0.5)

            print("[Force Mode] Set desired force")
            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            print("[Force Mode] Wait for contact")
            while not check_force_condition(DR_AXIS_Z, max=10):
                wait(0.5)
            print("[Force Mode] Contact detected! Releasing")
            release_force()
            wait(0.5)
            print("[Force Mode] Release compliance")
            release_compliance_ctrl()
            wait(0.5)
            print("[Force Mode] Compliance released")
            poscup_x, _ = get_current_posx()
        
            prePosCupX = poscup_x.copy()
            poscup_x[2] += 53
            movel(poscup_x, vel=VELOCITY, acc=ACC)
    
            release()
            wait(1)
            poscup_x[2] -= 31
            poscup_x[0] -= 5
            movel(poscup_x, vel=30, acc=30)
        else :
            prePosCupX[2] +=  53 
            prePosCupX[2] -=  6.5 
            movel(prePosCupX, vel=VELOCITY, acc=ACC)

            release()
            wait(1)
            prePosCupX[2] -= 31
            prePosCupX[0] -= 5
            movel(prePosCupX, vel=30, acc=30)

    def move_cup(JReady, posxej, bottle1top, bottle1bottom, bottle2top, bottle2bottom):
        # First cup pick
        movej(JReady, vel=VELOCITY, acc=ACC)
        movej(posxej, vel=VELOCITY, acc=ACC)
        force_pick_cup()  # <-- Force & compliance로 컵 집기
        full_grip()
        wait(1)
        poscup_x, _ = get_current_posx()
        poscup_x[2] += 100
        movel(poscup_x, vel=VELOCITY, acc=ACC)
        # First cup place
        movej(bottle1top, vel=VELOCITY, acc=ACC)
        movej(bottle1bottom, vel=VELOCITY, acc=ACC)
        
        pos_down = list(bottle1bottoml) 
        pos_down[2] -= 35
        movel(posx(pos_down), vel=VELOCITY, acc=ACC)
        wait(1)
        
        release()
        wait(1)
        
        # Second cup pick
        movej(posxej, vel=VELOCITY, acc=ACC)
        force_pick_cup()  # 두 번째 컵도 동일하게
        full_grip()
        wait(1)
        poscup_x2,_ = get_current_posx()
        poscup_x2[2] += 100
        movel(poscup_x, vel=VELOCITY, acc=ACC)
        # Second cup place
        movej(bottle2top, vel=VELOCITY, acc=ACC)
        movej(bottle2bottom, vel=VELOCITY, acc=ACC)
        
        pos_down = list(bottle2bottoml)  
        pos_down[2] -= 35 
        movel(posx(pos_down), vel=VELOCITY, acc=ACC)
        wait(1)
        
        release()
        wait(1)

    def ice_sequence(init, shavel1, shavel2, shavel3, locs, ices):
        movej(init, vel=VELOCITY, acc=ACC)
        
        # Move to shovel pickup position
        set_digital_output(2,OFF)
        set_digital_output(1,OFF)
        wait(1)
        set_digital_output(1,ON)
        wait(1)
        # Grab shovel
        movej(shavel1, vel=VELOCITY, acc=ACC)
        movej(shavel2, vel=VELOCITY, acc=ACC)
        wait(1)
        normal_grip()
        movej(shavel3, vel=VELOCITY, acc=ACC)
        # Ice pick sequence
        for loc in locs:
            movej(loc, vel=VELOCITY, acc=ACC)
        for i in [0, 1, 2]:
            movej(ices[i], vel=VELOCITY, acc=ACC)
        for loc in locs:
            movej(loc, vel=VELOCITY, acc=ACC)
        for i in [3, 4, 5]:
            movej(ices[i], vel=VELOCITY, acc=ACC)

    def redbean_sequence(init, redbean1, redbean2, redbean3):
        # Grab red beans
        movej(redbean1, vel=VELOCITY, acc=ACC)
        movej(redbean2, vel=VELOCITY, acc=ACC)
        movej(redbean3, vel=VELOCITY, acc=ACC)

        wait(0.5)
        movej(redbean2, vel=VELOCITY, acc=ACC)
        movej(redbean1, vel=VELOCITY, acc=ACC)
        
        # wait(1)
        movej(redbean4, vel=VELOCITY, acc=ACC)
        for i in [0, 1, 2]:
            movej(ices[i], vel=VELOCITY, acc=ACC)
            
        movej(redbean1, vel=VELOCITY, acc=ACC)
        movej(redbean2, vel=VELOCITY, acc=ACC)
        movej(redbean3, vel=VELOCITY, acc=ACC)

        wait(0.5)
        movej(redbean2, vel=VELOCITY, acc=ACC)
        movej(redbean1, vel=VELOCITY, acc=ACC)

        # wait(1)
        movej(redbean4, vel=VELOCITY, acc=ACC)
        for i in [3, 4, 5]:
            movej(ices[i], vel=VELOCITY, acc=ACC)
        
        movej(shavel1, vel=VELOCITY, acc=ACC)
        movej(shavel2, vel=VELOCITY, acc=ACC)
        wait(1)
        release()
        movej(shavel3, vel=VELOCITY, acc=ACC)
        movej(init, vel=VELOCITY, acc=ACC)

    JReady = posj(0, 0, 90, 0, 90, 0)
    posxej = posj([-23.06, 38.03, 49.96, 1.23, 92.67, -20.62])
    bottle1top = posj(-9.61, 29.33, 51.93, 0, 99.03, -15.43)
    bottle1bottom = posj(-8.62, 32.8, 61.88, 0.34, 86.31, -11.84)
    bottle1bottoml = posx(580.86, -80.2, 328.82, 148.24, 178.91, 144.97)

    bottle2top = posj(6.66, 30, 52.7, 0.01, 97.76, 0.81)
    bottle2bottom = posj(5.98, 32.3, 62.07, 0.49, 86.56, 2.68)
    bottle2bottoml = posx(579.57, 68.86, 332.84, 153.72, 178.91, 150.38)
    init = posj(0, 0, 90, 0, 90, 0)
    shavel1 = posj(6.78, 12.65, 79.05, -0.03, 88.31, 6.85) 
    shavel2 = posj(6.75, 15.91, 94.09, -0.03, 70.28, 6.85)
    shavel3 = posj(6.78, 12.65, 79.05, -0.03, 88.31, 6.85) 
    
    # 팥 퍼내기
    redbean1 = posj(-28.92, 16.77, 67.89, -0.62, 98.14, -115.75)#  posx(421.36, -226.24, 441.28, 162.24, 177.15, 75.39)
    redbean2 = posj(-29, 17.84, 88.19, -0.8, 76.74, -115.75) #posx(416.26, -224.47, 303.11, 165.8, 177.14, 78.75)
    redbean3 = posj(-33.09, 5.42, 101.3, -7.79, 81.4, -120.8) #posx(278.2, -222.74, 305.29, 167.54, 176.91, 80.63)
    # posj(-25.18, 18.43, 96.07, -14.83, 86.99, -110.92)
    redbean4 = posj(-30.52, 19.46, 68.54, 24.9, 102.06, -30.07) # posx(444.26, -195.93, 426.61, 82.93, 153.28, 80.17)
    
    locs = [
        # 얼음 퍼내기
        posj(-32.6, 0.37, 89.9, -0.08, 90.03, -32.26),
        posj(-32.66, 3.66, 107.23, -0.08, 69.52, -32.26), 
        posj(-7.58, -3.72, 114.59, -0.02, 69.48, -12.05),
        posj(-28.56, 4.64, 100.89, 22.92, 93.44, -32.3),
        posj(-5.44, -8.43, 103.25, -1.67, 84.13, -3.29),
        posj(-28.95, -8.08, 102.59, 22.83, 91.79, -24.88) 
    ]

    ices = [
        # 첫번째 컵
        posj(-15.02, 25.97, 66.91, 12.55, 83.68, -10.88),
        posj(9.3, 25.21, 66.14, -23.96, 86.99, 13.19),
        posj(14.18, 26.55, 66.28, -33.22, 87.18, 13.18),
        # 두번째 컵
        posj(-35.08, 17.24, 79.14, 19.91, 81.79, 1.09),
        posj(-11.69, 21.53, 70.71, -14.11, 91.71, 36.04),
        posj(3.06, 26.04, 66.7, -35.06, 97.46, 35.99)
    ]

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    if rclpy.ok():
        release()
        print("[Start] Cup pick & place (with force control)")
        move_cup(JReady, posxej, bottle1top, bottle1bottom, bottle2top, bottle2bottom)

        print("[Start] Ice serving sequence")
        ice_sequence(init, shavel1, shavel2, shavel3, locs, ices)
        print("[Start] Red bean serving sequence")
        redbean_sequence(init, redbean1, redbean2, redbean3)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
