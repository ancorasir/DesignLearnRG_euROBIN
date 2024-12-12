from tasks.base_task import Task
from copy import deepcopy
import numpy as np
from motion.utils import PressButtonCfg, config_dir

class PressButton(Task):
    def __init__(
        self,
        cfg_path,
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

        cfg = PressButtonCfg(cfg_path=cfg_path)

        pre_garsp_pos = deepcopy(cfg.pos_on_board)
        pre_garsp_pos[2] += cfg.press_prep_height_offset  # go above the button
        pre_grasp_pose = {
            "pos": pre_garsp_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        press_pos = deepcopy(cfg.pos_on_board)
        press_pos[2] = cfg.press_height
        press_pose = {
            "pos": press_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        self.set_task_motions(
            [
                pre_grasp_pose,
                press_pose,
                pre_grasp_pose,
            ]
        )

class PressBlueButton(PressButton):
    def __init__(self, rtde_rec, rtde_ctrl, gripper_ctrl, taskboard_pose_in_world, default_tcp_rot_mat_in_board=None) -> None:
        cfg_path = f"{config_dir}/press_blue_button.yaml"
        super().__init__(cfg_path, rtde_rec, rtde_ctrl, gripper_ctrl, taskboard_pose_in_world, default_tcp_rot_mat_in_board)

class PressRedButton(PressButton):
    def __init__(self, rtde_rec, rtde_ctrl, gripper_ctrl, taskboard_pose_in_world, default_tcp_rot_mat_in_board=None) -> None:
        cfg_path = f"{config_dir}/press_red_button.yaml"
        super().__init__(cfg_path, rtde_rec, rtde_ctrl, gripper_ctrl, taskboard_pose_in_world, default_tcp_rot_mat_in_board)