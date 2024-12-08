from tasks.base_task import Task
from copy import deepcopy
import numpy as np


class PlugLeft(Task):
    def __init__(
        self,
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

        self.__init_left_plug_pos = [0.058, 0.0, 0.0]
        self.__goal_left_plug_pos = [0.058, 0.025, 0.0]

        self._z_rot = (110 / 180) * np.pi

        pre_garsp_pos = deepcopy(self.__init_left_plug_pos)
        pre_garsp_pos[2] = 0.03  # 10 cm above the red button
        pre_grasp_pose = {
            "pos": pre_garsp_pos,
            "z_rot": self._z_rot,  # 100 degrees
            "gripper_pos": 150,  # Open the gripper
            "vel": 2.0,
            "acc": 4.0,
        }

        ready_to_grip_pos = deepcopy(self.__init_left_plug_pos)
        ready_to_grip_pos[2] = 0.005
        ready_to_grip_pose = {
            "pos": ready_to_grip_pos,
            "z_rot": self._z_rot,  # 100 degrees
            "gripper_pos": 150,  # Open the gripper
            "vel": 2.0,
            "acc": 4.0,
        }

        grip_pose = {
            "pos": ready_to_grip_pos,
            "z_rot": self._z_rot,  # 100 degrees
            "gripper_pos": 200,  # Close the gripper
            "vel": 2.0,
            "acc": 4.0,
        }

        pull_up_pose = {
            "pos": pre_garsp_pos,
            "z_rot": self._z_rot,  # 100 degrees
            "gripper_pos": 200,  # Open the gripper
            "vel": 2.0,
            "acc": 4.0,
        }

        plug_in_prep_pos = deepcopy(self.__goal_left_plug_pos)
        plug_in_prep_pos[2] = 0.03
        plug_in_prep_pose = {
            "pos": plug_in_prep_pos,
            "z_rot": self._z_rot,  # 100 degrees
            "gripper_pos": 200,  # Open the gripper
            "vel": 2.0,
            "acc": 4.0,
        }

        plug_in_pos = deepcopy(self.__goal_left_plug_pos)
        plug_in_pos[2] = 0.005
        plug_in_pose = {
            "pos": plug_in_pos,
            "z_rot": (125 / 180) * np.pi,  # 100 degrees
            "gripper_pos": 200,  # Close the gripper
            "vel": 2.0,
            "acc": 4.0,
        }

        open_gripper_pose = deepcopy(plug_in_pose)
        open_gripper_pose["gripper_pos"] = 150

        final_pull_up_pose = deepcopy(open_gripper_pose)
        final_pull_up_pose["pos"][2] += 0.03

        self.set_task_motions(
            [
                pre_grasp_pose,
                ready_to_grip_pose,
                grip_pose,
                pull_up_pose,
                plug_in_prep_pose,
                # plug_in_pose,
                open_gripper_pose,
                final_pull_up_pose,
            ]
        )
