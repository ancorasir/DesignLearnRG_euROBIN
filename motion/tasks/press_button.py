from tasks.base_task import Task
from copy import deepcopy
import numpy as np


class PressButton(Task):
    def __init__(
        self,
        button_pos_in_board,
        rtde_rec,
        rtde_ctrl,
        gripper_ctrl,
        taskboard_pose_in_world,
        default_tcp_rot_mat_in_board=None,
    ) -> None:

        super().__init__(
            rtde_rec=rtde_rec,
            rtde_ctrl=rtde_ctrl,
            gripper_ctrl=gripper_ctrl,
            task_motions=None,
            taskboard_pose_in_world=taskboard_pose_in_world,
            default_tcp_rot_mat_in_board=default_tcp_rot_mat_in_board,
        )

        self.__button_pos_in_taskboard = button_pos_in_board
        self.__press_button_z_in_board = 0.002  # 0.105 for real testing
        self.__ready_to_press_z_in_board = 0.02  # 0.115 for real testing
        self.__z_rot = np.pi / 2.0  # No need to change orientation
        self.gripper_pos = 255  # Always close the gripper

        pre_garsp_pos = deepcopy(self.__button_pos_in_taskboard)
        pre_garsp_pos[2] += 0.02  # 2 cm above the red button
        pre_grasp_pose = {
            "pos": pre_garsp_pos,
            "z_rot": self.__z_rot,
            "gripper_pos": self.gripper_pos,
            "vel": 2.0,
            "acc": 4.0,
        }

        # ready_to_press_button_pos = deepcopy(self.__button_pos_in_taskboard)
        # ready_to_press_button_pos[2] = self.__ready_to_press_z_in_board
        # ready_to_press_pose = {
        #     "pos": ready_to_press_button_pos,
        #     "z_rot": self.__z_rot,
        #     "gripper_pos": self.gripper_pos
        # }

        press_pos = deepcopy(self.__button_pos_in_taskboard)
        press_pos[2] = self.__press_button_z_in_board
        press_pose = {
            "pos": press_pos,
            "z_rot": self.__z_rot,
            "gripper_pos": self.gripper_pos,
            "vel": 2.0,
            "acc": 4.0,
        }

        self.set_task_motions(
            [
                pre_grasp_pose,
                # ready_to_press_pose,
                press_pose,
                pre_grasp_pose,
            ]
        )


class PressRedButton(PressButton):
    def __init__(
        self,
        rtde_rec,
        rtde_ctrl,
        gripper_ctrl,
        taskboard_pose_in_world,
        default_tcp_rot_mat_in_board=None,
    ) -> None:

        red_button_pos_in_taskboard = [0.0, 0.014, 0.0]

        super().__init__(
            button_pos_in_board=red_button_pos_in_taskboard,
            rtde_rec=rtde_rec,
            rtde_ctrl=rtde_ctrl,
            gripper_ctrl=gripper_ctrl,
            taskboard_pose_in_world=taskboard_pose_in_world,
            default_tcp_rot_mat_in_board=default_tcp_rot_mat_in_board,
        )


class PressBlueButton(PressButton):
    def __init__(
        self,
        rtde_rec,
        rtde_ctrl,
        gripper_ctrl,
        taskboard_pose_in_world,
        default_tcp_rot_mat_in_board=None,
    ) -> None:

        blue_button_pos_in_taskboard = [0.0, 0.0, 0.0]

        super().__init__(
            button_pos_in_board=blue_button_pos_in_taskboard,
            rtde_rec=rtde_rec,
            rtde_ctrl=rtde_ctrl,
            gripper_ctrl=gripper_ctrl,
            taskboard_pose_in_world=taskboard_pose_in_world,
            default_tcp_rot_mat_in_board=default_tcp_rot_mat_in_board,
        )
