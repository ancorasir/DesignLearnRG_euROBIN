from tasks.base_task import Task
from copy import deepcopy
import numpy as np
from motion.utils import PlugProbeCfg, config_dir

class PlugProbe(Task):
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
        cfg = PlugProbeCfg(cfg_path=f"{config_dir}/plug_probe.yaml")
        self.cfg = cfg

        pre_garsp_pos = deepcopy(cfg.pos_on_board)
        pre_garsp_pos[2] = cfg.pre_grasp_z  # 10 cm above the red button
        pre_grasp_pose = {
            "pos": pre_garsp_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_open_pos,  # Open the gripper
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        ready_to_grip_pos = deepcopy(cfg.pos_on_board)
        ready_to_grip_pos[2] = cfg.grasp_z
        ready_to_grip_pose = {
            "pos": ready_to_grip_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_open_pos,  # Open the gripper
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        grip_pose = {
            "pos": ready_to_grip_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,  # Close the gripper
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        pull_up_pose = {
            "pos": pre_garsp_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,  # Open the gripper
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        plug_in_prep_pos = deepcopy(cfg.goal_pos_on_board)
        plug_in_prep_pos[2] = cfg.pre_grasp_z
        plug_in_prep_pose = {
            "pos": plug_in_prep_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,  # Open the gripper
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        plug_in_pos = deepcopy(cfg.goal_pos_on_board)
        plug_in_pos[2] = cfg.grasp_z
        plug_in_pose = {
            "pos": plug_in_pos,
            "z_rot": cfg.plug_in_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,  # Close the gripper
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        open_gripper_pose = deepcopy(plug_in_pose)
        open_gripper_pose["gripper_pos"] = cfg.default_gripper_open_pos

        final_pull_up_pose = deepcopy(open_gripper_pose)
        final_pull_up_pose["pos"][2] = cfg.pre_grasp_z

        self.set_task_motions(
            [
                pre_grasp_pose,
                ready_to_grip_pose,
                grip_pose,
                pull_up_pose,
                plug_in_prep_pose,
                open_gripper_pose,
                final_pull_up_pose,
            ]
        )
