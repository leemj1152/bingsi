# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_getting_position", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            get_current_posx,
            get_current_posj,
            set_tool,
            set_tcp,
            movej,
            movel,
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def move_and_log(move_type, position, index, vel=VELOCITY, acc=ACC):
        if move_type == "joint":
            print(f"Moving to joint position: {position}")
            movej(position, vel=vel, acc=acc)
        elif move_type == "task":
            movel(position, vel=vel, acc=acc)
            posx, _ = get_current_posx()
            print(f"Moving to task position: {posx}")
        else:
            raise ValueError("Invalid move type. Use 'joint' or 'task'.")

        # posx, _ = get_current_posx()
        # posj, _ = get_current_posj()
        # print(f"current position{index} : {posx}")
        # print(f"current position{index} : {posj}")

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    JReady = posj([0, 0, 90, 0, 90, 0])
    # pos1 = posx([496.06, 93.46, 296.92, 20.75, 179.00, 19.09])
    # pos2 = posx([548.70, -193.46, 96.92, 20.75, 179.00, 19.09])
    # pos3 = posx([596.70, -7.46, 196.92, 20.75, 179.00, 19.09])

    # force [-22.895, 31.609, 57.303, 0.635, 92.001, -4.792]


    if rclpy.ok():
        # move_and_log("task", JReady, 1)
        posxej = posj([-22.895, 31.609, 57.303, 0.635, 92.001, -4.792])
        print(f"Moving to task position: {posxej}")
        movej(JReady, vel=VELOCITY, acc=ACC)
        movej(posxej, vel=VELOCITY, acc=ACC)
        # move_and_log("task", pos1, 2)
        # move_and_log("task", pos2, 3)
        # move_and_log("task", pos3, 4)
        # move_and_log("joint", JReady, 5)
        print("Starting task_compliance_ctrl")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)

        print("Starting set_desired_force")
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        # 외력이 0 이상 5 이하이면 0
        # 외력이 5 초과이면 -1
        while not check_force_condition(DR_AXIS_Z, max=6):
            print("Waiting for an external force greater than 2 ")
            time.sleep(0.5)
            pass

        print("Starting release_force")
        poscup_x, _ = get_current_posx()
        poscup_x[2] += 40
        movel(poscup_x, vel=VELOCITY, acc=ACC)
        print(poscup_x)

        release_force()
        time.sleep(0.5)
        
        print("Starting release_compliance_ctrl")      
        release_compliance_ctrl()

    rclpy.shutdown()


if __name__ == "__main__":
    main()