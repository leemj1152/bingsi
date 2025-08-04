import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_getting_position", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, get_current_posx, check_force_condition, release_compliance_ctrl,
            movel, set_digital_output, set_tool, set_tcp, release_force,
            task_compliance_ctrl, set_desired_force, DR_AXIS_Z, DR_FC_MOD_REL, DR_BASE
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    ON, OFF = 1, 0

    def wait(sec):
        time.sleep(sec)

    def full_grip():
        set_digital_output(1,ON)
        set_digital_output(2,ON)       
        wait(2)


    def release():
        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        wait(2)
        
    def normal_grip():
        set_digital_output(1,OFF)
        set_digital_output(2,OFF)
        wait(2)


    def force_pick_cup():
        normal_grip()
        print("[Force Mode] Activate compliance")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)

        print("[Force Mode] Set desired force")
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        print("[Force Mode] Wait for contact")
        while not check_force_condition(DR_AXIS_Z, max=10):
            time.sleep(0.5)
        print("[Force Mode] Contact detected! Releasing")
        release_force()
        time.sleep(0.5)
        print("[Force Mode] Release compliance")
        release_compliance_ctrl()
        time.sleep(0.5)
        print("[Force Mode] Compliance released")
        poscup_x, _ = get_current_posx()
        poscup_x[2] += 53
        movel(poscup_x, vel=VELOCITY, acc=ACC)
        release()
        poscup_x[2] -= 28
        poscup_x[0] -= 5
        movel(poscup_x, vel=VELOCITY, acc=ACC)

    def move_cup(JReady, posxej, bottle1top, bottle1bottom, bottle2top, bottle2bottom):
        # First cup pick
        movej(JReady, vel=VELOCITY, acc=ACC)
        movej(posxej, vel=VELOCITY, acc=ACC)
        force_pick_cup()  # <-- Force & compliance로 컵 집기
        full_grip()
        wait(2)
        poscup_x, _ = get_current_posx()
        poscup_x[2] += 100
        movel(poscup_x, vel=VELOCITY, acc=ACC)
        # First cup place
        movej(bottle1top, vel=VELOCITY, acc=ACC)
        movej(bottle1bottom, vel=VELOCITY, acc=ACC)
        release()
        wait(2)
        # Second cup pick
        movej(posxej, vel=VELOCITY, acc=ACC)
        force_pick_cup()  # 두 번째 컵도 동일하게
        full_grip()
        wait(2)
        poscup_x2,_ = get_current_posx()
        poscup_x2[2] += 100
        movel(poscup_x, vel=VELOCITY, acc=ACC)
        # Second cup place
        movej(bottle2top, vel=VELOCITY, acc=ACC)
        movej(bottle2bottom, vel=VELOCITY, acc=ACC)
        release()
        wait(2)

    def ice_sequence(init, shavel1, shavel2, shavel3, locs, ices):
        movej(init, vel=VELOCITY, acc=ACC)
        # Grab shovel
        movej(shavel1, vel=VELOCITY, acc=ACC)
        movej(shavel2, vel=VELOCITY, acc=ACC)
        wait(2)
        normal_grip()
        movej(shavel3, vel=VELOCITY, acc=ACC)
        # Ice pick sequence
        for loc in locs:
            movej(loc, vel=VELOCITY, acc=ACC)
        for i in [0, 1, 0]:
            movej(ices[i], vel=VELOCITY, acc=ACC)
        for loc in locs:
            movej(loc, vel=VELOCITY, acc=ACC)
        for i in [2, 3, 2]:
            movej(ices[i], vel=VELOCITY, acc=ACC)

    # ==== Position Definitions ====
    release()
    JReady = posj(0, 0, 90, 0, 90, 0)
    posxej = posj([-23.06, 38.03, 49.96, 1.23, 92.67, -20.62])
    bottle1top = posj(-9.61, 29.33, 51.93, 0, 99.03, -15.43)
    bottle1bottom = posj(-9.22, 29.05, 70.56, 0, 81.06, -15.4)
    bottle2top = posj(6.66, 30, 52.7, 0.01, 97.76, 0.81)
    bottle2bottom = posj(6.37, 29.85, 68.78, 0, 81.89, 0.69)
    init = posj(0, 0, 90, 0, 90, 0)
    shavel1 = posj(5.66, 13.13, 75.54, 0.01, 91.87, 0)
    shavel2 = posj(5.64, 18.25, 97.73, 0.01, 64.51, 0)
    shavel3 = posj(5.66, 13.13, 75.54, 0.01, 91.87, 0)
    locs = [
        posj(-25.84, -4.13, 96.49, -0.56, 87.94, -26.65),
        posj(-26.11, 2.1, 116.3, -0.57, 61.93, -26.65),
        posj(-5.95, -0.7, 119.89, -0.34, 61.36, -6.78),
        posj(-24.07, -8.01, 114.61, 25.71, 77.1, -30.78)
    ]
    ices = [
        posj(-14.09, 30.15, 69.21, 10.98, 78.55, -14),
        posj(4.86, 29.41, 66.49, -23.08, 79.42, -3.5),
        posj(-34.23, 26.99, 70.61, 21.18, 88.44, -7.53),
        posj(-9.82, 25.44, 73.19, -23.35, 83.32, 20.86)
    ]

    '''
    bowl pick 1 posj(-23.06, 38.03, 49.96, 1.23, 92.67, -20.62) posx(572.89, -233.46, 350.07, 93.6, 178.51, 95.92)

    '''

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    if rclpy.ok():
        print("[Start] Cup pick & place (with force control)")
        move_cup(JReady, posxej, bottle1top, bottle1bottom, bottle2top, bottle2bottom)

        print("[Start] Ice serving sequence")
        ice_sequence(init, shavel1, shavel2, shavel3, locs, ices)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
