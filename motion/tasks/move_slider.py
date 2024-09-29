from tasks.base_task import Task
from copy import deepcopy
import numpy as np
from message import detection_pb2  # Import the generated code from the .proto file  

class MoveSlider(Task):
    def __init__(
            self, 
            rtde_rec, 
            rtde_ctrl, 
            gripper_ctrl, 
            taskboard_pose_in_world, 
            default_tcp_rot_mat_in_board=None,
            socket=None
        ) -> None:

        super().__init__(
            rtde_rec=rtde_rec, 
            rtde_ctrl=rtde_ctrl, 
            gripper_ctrl=gripper_ctrl, 
            task_motions=None, 
            taskboard_pose_in_world=taskboard_pose_in_world, 
            default_tcp_rot_mat_in_board=default_tcp_rot_mat_in_board)

        self.__slider_pos_in_taskboard = [0.0369, 0.095, 0.0]
        self.__slider_z_in_board = 0.003 # 0.105 for real testing
        self.__ready_to_grip_z_in_board = 0.004 # 0.115 for real testing
        self.__z_rot = np.pi/2.0 # No need to change orientation
        self.gripper_pos = 255 # Always close the gripper
        self.socket = socket

        pre_garsp_pos = deepcopy(self.__slider_pos_in_taskboard)
        pre_garsp_pos[2] += 0.02 # 10 cm above the red button
        self.pre_grasp_pose = {
            "pos": pre_garsp_pos,
            "z_rot": self.__z_rot,
            "gripper_pos": 150, # Open the gripper
            "vel": 2.0,
            "acc": 4.0
        }

        ready_to_grip_slider_pos = deepcopy(self.__slider_pos_in_taskboard)
        ready_to_grip_slider_pos[2] = self.__ready_to_grip_z_in_board
        self.ready_to_grip_slider_pose = {
            "pos": ready_to_grip_slider_pos,
            "z_rot": self.__z_rot,
            "gripper_pos": 150, # Open the gripper
            "vel": 2.0,
            "acc": 4.0
        }

        self.grip_slider_pose = {
            "pos": ready_to_grip_slider_pos,
            "z_rot": self.__z_rot,
            "gripper_pos": 160, # Close the gripper
            "vel": 0.6,
            "acc": 1.5
        }

        self.slide_to_center_pose = deepcopy(self.grip_slider_pose)
        self.slide_to_center_pose["pos"][1] -= 0.02
        self.slide_to_center_pose["vel"] = 0.1
        self.slide_to_center_pose["acc"] = 0.1

    def execute(self):
        # Go to the pre grasp pose
        self.go_to_pose(self.pre_grasp_pose)

        # Ready to grid the slider
        self.go_to_pose(self.ready_to_grip_slider_pose)
        self.go_to_pose(self.grip_slider_pose)

        # Move to the center of the slide
        self.go_to_pose(self.slide_to_center_pose)

        import time

        # TODO: servo the tcp to move in y direction
        # Send a message  
        message = "direction" 
        self.socket.send_string(message)  
        print(f"Sent: {message}")

        time.sleep(1.0)

        reply = self.socket.recv()
        vision_message = detection_pb2.Vision()  
        vision_message.ParseFromString(reply)  # Parse the incoming bytes  
        direction = vision_message.direction
        print(direction)

        y_increment = -0.01 * direction
        if y_increment >= 0.017:
            y_increment = 0.017
        elif y_increment <= -0.015:
            y_increment = -0.015

        goal_slider_pose = deepcopy(self.slide_to_center_pose)
        goal_slider_pose["pos"][1] += y_increment
        self.go_to_pose(goal_slider_pose)

        # Release the gripper
        release_gripper_pose = deepcopy(goal_slider_pose)
        release_gripper_pose["gripper_pos"] = 130
        self.go_to_pose(release_gripper_pose)

        # Open the gripper
        self.ready_to_grip_slider_pose["pos"] = release_gripper_pose["pos"]
        self.ready_to_grip_slider_pose["pos"][2] += 0.03 
        self.go_to_pose(self.ready_to_grip_slider_pose)

        # Return back to the pre grasp pose
        # self.go_to_pose(self.pre_grasp_pose)

