from tasks.base_task import Task
from copy import deepcopy
import numpy as np
from motion.message.vision_socket import VisionSocket
from motion.utils import MoveSliderCfg, config_dir

class MoveSlider(Task):
    def __init__(
        self,
        rtde_rec,
        rtde_ctrl,
        gripper_ctrl,
        taskboard_pose_in_world,
        default_tcp_rot_mat_in_board=None,
        vision_server: VisionSocket=None,
    ) -> None:

        super().__init__(
            rtde_rec=rtde_rec,
            rtde_ctrl=rtde_ctrl,
            gripper_ctrl=gripper_ctrl,
            task_motions=None,
            taskboard_pose_in_world=taskboard_pose_in_world,
            default_tcp_rot_mat_in_board=default_tcp_rot_mat_in_board,
        )
        cfg = MoveSliderCfg(cfg_path=f"{config_dir}/move_slider.yaml")
        self.cfg = cfg

        self._vision_server = vision_server

        pre_garsp_pos = deepcopy(cfg.pos_on_board)
        pre_garsp_pos[2] += cfg.grip_slider_prep_height_offset  # above the slider
        self.pre_grasp_pose = {
            "pos": pre_garsp_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_open_pos,  # Open the gripper
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        ready_to_grip_slider_pos = deepcopy(cfg.pos_on_board)
        ready_to_grip_slider_pos[2] = cfg.grip_slider_z
        self.ready_to_grip_slider_pose = {
            "pos": ready_to_grip_slider_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_open_pos,  # Open the gripper
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        self.grip_slider_pose = {
            "pos": ready_to_grip_slider_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,  # Close the gripper
            "vel": cfg.slide_vel,
            "acc": cfg.slide_acc,
        }

        self.slide_to_center_pose = deepcopy(self.grip_slider_pose)

    def execute(self):
        # Go to the pre grasp pose
        self.go_to_pose(self.pre_grasp_pose)

        # Ready to grid the slider
        self.go_to_pose(self.ready_to_grip_slider_pose)
        self.go_to_pose(self.grip_slider_pose)

        # Move to the center of the slide
        self.go_to_pose(self.slide_to_center_pose)

        pixel_distance = self._vision_server.get_slider_moving_pixels()

        y_increment = self.cfg.pixel_to_distance_ratio * pixel_distance
        if y_increment >= self.cfg.max_y_increment:
            y_increment = self.cfg.max_y_increment
        elif y_increment <= self.cfg.min_y_increment:
            y_increment = self.cfg.min_y_increment

        goal_slider_pose = deepcopy(self.slide_to_center_pose)
        goal_slider_pose["pos"][1] += y_increment
        self.go_to_pose(goal_slider_pose)

        # Release the gripper
        release_gripper_pose = deepcopy(goal_slider_pose)
        release_gripper_pose["gripper_pos"] = self.cfg.release_gripper_pos
        self.go_to_pose(release_gripper_pose)

        # Open the gripper
        self.ready_to_grip_slider_pose["pos"] = release_gripper_pose["pos"]
        self.ready_to_grip_slider_pose["pos"][2] += self.cfg.grip_slider_prep_height_offset
        self.go_to_pose(self.ready_to_grip_slider_pose)
