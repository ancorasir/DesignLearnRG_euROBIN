import rtde_control
import rtde_receive
import os
import sys

root_dir = os.path.dirname(os.path.dirname(__file__))
sys.path.append(f"{root_dir}/lib")
import gripper
from tasks.press_button import PressRedButton, PressBlueButton

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")

hande = gripper.Hand_E("/dev/ttyUSB0")
hande.connect()
hande.activate()

taskboard_pose_in_world = [
    0.1416904305,
    -0.6037241808,
    0.1020000000,
    0.0000000000,
    0.0000000000,
    3.0212879884,
]

if __name__ == "__main__":
    press_button = PressBlueButton(
        rtde_rec=rtde_r,
        rtde_ctrl=rtde_c,
        gripper_ctrl=hande,
        taskboard_pose_in_world=taskboard_pose_in_world,
        default_tcp_rot_mat_in_board=None,
    )

    press_button.execute()
