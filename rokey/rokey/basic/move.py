import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_MV_RA_OVERRIDE
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    JReady = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    pos1 = posj([-23.06, 38.03, 49.96, 1.23, 92.67, -20.62])
    pos2 = posj([6.75, 15.91, 94.09, -0.03, 70.28, 6.85])


    while rclpy.ok():

        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)

        print(f"Moving to task position: {pos1}")
        movej(pos1, vel=VELOCITY, acc=ACC, r=200)

        print(f"Moving to task position: {pos2}")
        movej(pos2, vel=VELOCITY, acc=ACC, ra=DR_MV_RA_OVERRIDE)

        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)

        print(f"Moving to task position: {pos1}")
        movej(pos1, vel=VELOCITY, acc=ACC)

        print(f"Moving to task position: {pos2}")
        movej(pos2, vel=VELOCITY, acc=ACC)

    rclpy.shutdown()

if __name__ == "__main__":
    main()