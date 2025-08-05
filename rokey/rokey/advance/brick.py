# Brik Assemble @20250515

import rclpy
import DR_init
import time
import sys
from dsr_msgs2.srv import MoveStop

# Configuration for a single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
ON, OFF = 1, 0

# Initialize DR_init with robot parameters
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("brick_assemble_node", namespace=ROBOT_ID)
DR_init.__dsr__node = node

client_move_stop = node.create_client(MoveStop, "motion/move_stop")
while not client_move_stop.wait_for_service(timeout_sec=1.0):
    node.get_logger().info("Waiting for MoveStop service...")
def movestop(node, client_move_stop):
    req = MoveStop.Request()
    req.stop_mode = 1
    future = client_move_stop.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    try:
        result = future.result()
    except Exception as e:
        node.get_logger().error(f"MoveStop service call failed: {e}")
    else:
        if result == None:
            ret = -1
        else:
            ret = 0 if (result.success == True) else -1
    return ret

############ Main ##############
def main(args=None):


    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            amovel,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            release_force,
            release_compliance_ctrl,
            get_digital_input,
            set_digital_output,
            wait,
            get_current_posx,
            move_periodic,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_TOOL,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    ############ Grippper Grip & Rease ##############
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            time.sleep(0.5)
            print("Wait for digital input", sig_num)
            pass

    def release():
        ret = task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        if ret == 0:
             print("Compliance_ctrl Set")
        else:
             print("Compliance_ctrl failed!!")

        time.sleep(0.1)
        ret = set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        if ret == 0:
             print("set_desired_force Set")
        else:
             print("set_desired_force Set Failed!!!!!")

        time.sleep(0.1)

        force_condition = 0
        while (force_condition > -1): # 힘제어로 블럭 놓기
            force_condition = check_force_condition(DR_AXIS_Z, max=10)
            time.sleep(0.1)
        
        if(release_force() == 0):
            print("release force")
        else:
             print("release force Failed!!!!")
        time.sleep(0.1)
        if(release_compliance_ctrl() == 0) :
             print("release compliance ctrl")
        else:
             print("release compliance ctrl Failed!!!!")

        set_digital_output(1,ON)
        set_digital_output(2,OFF)
        wait(2)

    def grip():
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        wait(2)

        # 힘 제어로 블럭 접촉
        ret = task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        if ret == 0:
            print("Compliance_ctrl Set")
        else:
            print("Compliance_ctrl failed!!")
        time.sleep(0.1)

        ret = set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        if ret == 0:
            print("set_desired_force Set")
        else:
            print("set_desired_force Set Failed!!!!!")
        time.sleep(0.1)

        force_condition = 0
        while (force_condition > -1):  # Z축 힘 조건 만족 대기
            force_condition = check_force_condition(DR_AXIS_Z, max=10)
            time.sleep(0.1)

        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()
        time.sleep(0.1)

        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(2)

        poscup_x, _ = get_current_posx()
        poscup_x[2] -= 20
        movel(poscup_x, vel=30, acc=30)

        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        wait(2)        

    ##### Pose Define ####
    pos_home = posj(3.45, 5.33, 81.73, -2.57, 94.49, 0.52)
    block_1_grip = posj(5.92, 34.79, 69.59, -0.89, 77.11, 5.43)
    block_2_grip = posj(2.98, 34.86, 69.65, -1.07, 77.01, 3.02)

    target = posj(-2.77, 37.87, 62.98, -4.34, 79.9, -0.84) # 블럭1 놓을 위치의 위로 이동
    # target2 = posj(-3.97, 35.12, 63.37, -3.18, 84.3, -3.28) # 블럭2 놓을 위치의 위로 이동

    # init = posj(3.45, 5.33, 81.73, -2.57, 94.49, 0.52) posx(400.86, 25.77, 441.59, 61.98, -177.03, 59.11)

    # block1 = posj(5.92, 34.79, 69.59, -0.89, 77.11, 5.43) posx(583.11, 65.78, 259.66, 33.83, -178.32, 33.06)
    # block2 = posj(2.98, 34.86, 69.65, -1.07, 77.01, 3.02) posx(585.89, 35.44, 258.52, 35.17, -178.21, 34.88)

    # obj_loc = posj(-4.57, 33.82, 67.86, -1.43, 79.66, -3.93) posx(583.74, -42.47, 280.48, 40.06, -178.13, 40.35)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():
        set_digital_output(1,ON)
        set_digital_output(2,OFF)
        wait(2)

        # 동작 전 그리퍼 열림 확인
        # release()
        print("Gripper Release")

        # Home Pose
        movej(pos_home, vel = VELOCITY, acc = ACC)
        print("HomePose Moved")
        time.sleep(0.1)

        ########### Block 1 Assemble ############
        # Block_1 Grip
        
        movej(block_1_grip, vel = VELOCITY, acc = ACC) #블럭 위로 이동
        print("블럭 위로 이동")
        time.sleep(0.1)

        # movel(block_1_grip_down, vel = VELOCITY, acc = ACC, mod=1) #블럭 위치로 이동
        # print("블럭 그립 위치로 다운")
        # time.sleep(0.1)
        grip() # 블럭 집기
        print("블럭 집기")
        time.sleep(0.1)
        movej(posj(6.02, 18.51, 88.49, -0.84, 60.57, 5.42), vel = VELOCITY, acc = ACC) #블럭 들어올리기
        # print("블럭 들기")


        # Assemble_1
        movej(target, vel = VELOCITY, acc = ACC)

        release()

        time.sleep(0.1)

        # target1_up = posx(0, 0, 100, 0, 0, 0)
        # movel(target1_up, vel = VELOCITY, acc = ACC, mod=1)


        ########### Block 2 Assemble ############
        # Block_2 Grip
        movej(block_2_grip, vel = VELOCITY, acc = ACC)

        # movel(block_2_grip_down, vel = VELOCITY, acc = ACC, mod=1)
        # time.sleep(0.1)
        grip()
        # time.sleep(0.1)
        movej(posj(2.84, 19.49, 90.57, -0.77, 59.28, -0.33), vel = VELOCITY, acc = ACC)
        
        # movel(block_2_grip_up, vel = VELOCITY, acc = ACC, mod=1)

        # Assemble_2
        movej(target, vel = VELOCITY, acc = ACC)

        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.1)

        force_condition = 0
        while (force_condition > -1): # 힘제어로 블럭 놓기
            force_condition = check_force_condition(DR_AXIS_Z, max=15)
            time.sleep(0.1)

        
        if(release_force() == 0):
            print("release force")
        else:
             print("release force Failed!!!!")
        time.sleep(0.1)
        if(release_compliance_ctrl() == 0) :
             print("release compliance ctrl")
        else:
             print("release compliance ctrl Failed!!!!")

        release()

        time.sleep(0.1)

        target2_up = posx(0, 0, 100, 0, 0, 0)
        movel(target2_up, vel = 30, acc = 30, mod=1)


    rclpy.shutdown()
if __name__ == "__main__":
    main()


