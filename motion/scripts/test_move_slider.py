import rtde_control
import os
import sys

root_dir = os.path.dirname(os.path.dirname(__file__))
sys.path.append(f"{root_dir}/lib")
import gripper
from tasks.move_slider import MoveSlider

# rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")
rtde_r = None
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

# Set up the context and socket
import zmq

context = zmq.Context()
socket = context.socket(zmq.REQ)  # Create a request socket
socket.connect("tcp://192.168.1.13:5556")

if __name__ == "__main__":
    move_slider = MoveSlider(
        rtde_rec=rtde_r,
        rtde_ctrl=rtde_c,
        gripper_ctrl=hande,
        taskboard_pose_in_world=taskboard_pose_in_world,
        default_tcp_rot_mat_in_board=None,
        socket=socket,
    )

    move_slider.execute()
