import rtde_control
import zmq
from message import detection_pb2  # Import the generated code from the .proto file
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import sys

root_dir = os.path.dirname(os.path.dirname(__file__))
sys.path.append(f"{root_dir}/lib")
import gripper

rtde_r = None  # rtde_receive.RTDEReceiveInterface("192.168.1.102")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")

hande = gripper.Hand_E("/dev/ttyUSB0")
hande.connect()
hande.activate()

# Set up the context and socket
context = zmq.Context()
socket = context.socket(zmq.REQ)  # Create a request socket

socket.connect("tcp://192.168.1.13:5556")  # Connect to the server

print("Moving to observation pose...")
photo_pose = [0.14914, -0.5282, 0.2678, 0.000, 3.1415, 0.00]
hande.setPosition(250)

rtde_c.moveL(photo_pose, 0.6, 1.2)

# Send a message
message = "pose"
socket.send_string(message)
print(f"Sent: {message}")

# Wait for a reply
reply = socket.recv()
print("Received raw bytes:", reply)

# Decode the received data using Protocol Buffers
vision_message = detection_pb2.Vision()
vision_message.ParseFromString(reply)  # Parse the incoming bytes

taskboard_pose_in_world = vision_message.pose
taskboard_pose_in_world[0] -= 0.002
taskboard_pose_in_world[1] += 0.003
taskboard_pose_in_world[5] -= 4.0 / 180 * np.pi

# taskboard_pose_in_world = [0.12277797945080775, -0.5871794980257506, 0.1, 0.0, 0.0, -3.158948607193385]

from tasks.press_button import PressBlueButton

press_blue_button = PressBlueButton(
    rtde_rec=rtde_r,
    rtde_ctrl=rtde_c,
    gripper_ctrl=hande,
    taskboard_pose_in_world=taskboard_pose_in_world,
    default_tcp_rot_mat_in_board=None,
)
print("Press blue button task in motion...")
press_blue_button.execute()

from tasks.move_slider import MoveSlider

move_slider = MoveSlider(
    rtde_rec=rtde_r,
    rtde_ctrl=rtde_c,
    gripper_ctrl=hande,
    taskboard_pose_in_world=taskboard_pose_in_world,
    default_tcp_rot_mat_in_board=None,
    socket=socket,
)
print("Move slider task in motion...")
move_slider.execute()

from tasks.plug_left import PlugLeft

plug_left = PlugLeft(
    rtde_rec=rtde_r,
    rtde_ctrl=rtde_c,
    gripper_ctrl=hande,
    taskboard_pose_in_world=taskboard_pose_in_world,
    default_tcp_rot_mat_in_board=None,
)
print("Plug left task in motion...")
plug_left.execute()

from tasks.probe_circuit import ProbeCircuit

probe_circuit = ProbeCircuit(
    rtde_rec=rtde_r,
    rtde_ctrl=rtde_c,
    gripper_ctrl=hande,
    taskboard_pose_in_world=taskboard_pose_in_world,
    default_tcp_rot_mat_in_board=None,
)
print("Probe circuit task in motion...")
probe_circuit.execute()

from tasks.wrap_cable import WrapCable

wrap_cable = WrapCable(
    rtde_rec=rtde_r,
    rtde_ctrl=rtde_c,
    gripper_ctrl=hande,
    taskboard_pose_in_world=taskboard_pose_in_world,
    default_tcp_rot_mat_in_board=None,
)
print("Wrap cable task in motion...")
wrap_cable.execute()

from tasks.press_button import PressRedButton

press_red_button = PressRedButton(
    rtde_rec=rtde_r,
    rtde_ctrl=rtde_c,
    gripper_ctrl=hande,
    taskboard_pose_in_world=taskboard_pose_in_world,
    default_tcp_rot_mat_in_board=None,
)

hande.setPosition(250)
print("Press red button task in motion...")
press_red_button.execute()

print("Moving to observation pose...")
rtde_c.moveL(photo_pose, 0.6, 1.2)
hande.setPosition(250)
