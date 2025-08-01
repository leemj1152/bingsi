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
    node = rclpy.create_node("rokey_pick_place_with_force", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            get_current_posx,
            set_digital_output,
            set_tool,
            set_tcp,
            movej,
            movel,
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            wait,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
        )
        from DR_common2 import posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def release():
        print("[Release] Open gripper")
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        wait(0.5)
        time.sleep(0.2)

    def grip():
        print("[Grip] Close gripper")
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        wait(0.5)
        time.sleep(0.2)

    def interpolate_posj_grid(a_corners, b_corners, a_pre_corners, b_pre_corners, rows=3, cols=3):
        def interpolate(corners):
            lt, rt, rb, lb = [np.array(c) for c in corners]
            grid = np.empty((rows, cols), dtype=object)
            for i in range(rows):
                t = i / (rows - 1)
                left = (1 - t) * lt + t * lb
                right = (1 - t) * rt + t * rb
                for j in range(cols):
                    s = j / (cols - 1)
                    joint = (1 - s) * left + s * right
                    grid[i, j] = posj(*joint)
            return grid

        a_grid = interpolate(a_corners)
        b_grid = interpolate(b_corners)
        a_pre_grid = interpolate(a_pre_corners)
        b_pre_grid = interpolate(b_pre_corners)

        return [[(a_grid[i, j], b_grid[i, j], a_pre_grid[i, j], b_pre_grid[i, j]) for j in range(cols)] for i in range(rows)]

    def pick_and_place_with_force(a_main, a_pre, grid, label):
        movej(a_main, vel=VELOCITY, acc=ACC)
        movej(a_pre, vel=VELOCITY, acc=ACC)

        grip()

        # Force approach
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max=10):
            time.sleep(0.5)

        release_force()
        time.sleep(0.5)
        release_compliance_ctrl()
        time.sleep(0.5)
        release()

        current_posx, _ = get_current_posx()
        current_posx[2] -= 20
        movel(current_posx, vel=VELOCITY, acc=ACC)

        grip()
        time.sleep(0.5)

        current_posx, _ = get_current_posx()
        current_posx[2] += 50
        movel(current_posx, vel=VELOCITY, acc=ACC)

        # ✅ A에서의 z축 위치를 높이로 사용
        z_height = current_posx[2]  
        b_main, b_pre = update_target_counter(z_height, grid)

        movej(b_main, vel=VELOCITY, acc=ACC)
        movej(b_pre, vel=VELOCITY, acc=ACC)

        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max=10):
            time.sleep(0.5)

        release_force()
        time.sleep(0.5)
        release_compliance_ctrl()
        time.sleep(0.5)
        release()

        current_posx, _ = get_current_posx()
        current_posx[2] += 50
        movel(current_posx, vel=VELOCITY, acc=ACC)

       # 상단 전역 위치
    def update_target_counter(height, grid, thresholds=(304, 314, 324)):
        global target
        print('height',height)
        if height <= thresholds[0]:
            row = 0
            col = target[0]
            target[0] += 1
        elif height <= thresholds[1]:
            row = 1
            col = target[1]
            target[1] += 1
        elif height <= thresholds[2]:
            row = 2
            col = target[2]
            target[2] += 1
        else:
            raise ValueError(f"[Error] Too high object (height={height}), increase thresholds or skip.")

        return grid[row][col][1], grid[row][col][3]  # b_main, b_pre 반환


    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    JReady = posj(0, 0, 90, 0, 90, 0)

    A_posj_corners = [
        [1.23, 45.83, 29.98, -3.21, 104.82, 0.55],
        [-7.81, 48.4, 29.31, -3.11, 103.39, -7.44],
        [-9.34, 29.45, 60.28, -3.02, 91.7, -7.93],
        [1.32, 28.2, 60.57, -2.8, 93.23, 2.72]
    ]

    B_posj_corners = [
        [-24.48, -3.15, 99.59, -2.92, 84.57, -22.24],
        [-38.33, 4.01, 92.37, -2.81, 83.96, -35.69],
        [-49.49, -6.87, 98.51, -2.82, 88.07, -46.24],
        [-34.49, -15.81, 112.17, -2.68, 84.28, -31.16]
    ]

    A_posj_corners_pre = [
        [-1, 44.29, 47.49, -0.16, 88.97, -1.04],
        [-9.64, 45.7, 45.27, -0.58, 89.44, -9.05],
        [-11.43, 29.92, 71.66, -0.58, 78.93, -10.54],
        [-0.97, 28.49, 73.75, -0.46, 78.7, -0.05]
    ]

    B_posj_corners_pre = [
        [-27.43, -0.34, 110.78, -0.83, 70.44, -26.05],
        [-40.53, 6.88, 103.28, -1.08, 70.64, -38.93],
        [-51.81, -2.42, 112.88, -1.57, 70.54, -49.71],
        [-37.08, -12.83, 121.89, -1.47, 72.25, -35.05]
    ]

    if rclpy.ok():
        grid = interpolate_posj_grid(A_posj_corners, B_posj_corners, A_posj_corners_pre, B_posj_corners_pre)

        print("[Start] Pick and Place with Force Control")
        movej(JReady, vel=VELOCITY, acc=ACC)

        for i in range(3):
            for j in range(3):
                a_main, _, a_pre, _ = grid[i][j]
                pick_and_place_with_force(a_main, a_pre, grid, f"{i},{j}")


        print("[Done] All 9 objects transferred.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
