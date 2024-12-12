from tasks.base_task import Task
from copy import deepcopy
import numpy as np
from motion.utils import ProbeCircuitCfg, config_dir

class ProbeCircuit(Task):
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
        
        cfg = ProbeCircuitCfg(cfg_path=f"{config_dir}/probe_circuit.yaml")
        self.cfg = cfg

        pre_grasp_pos = deepcopy(cfg.pen_grasp_pos)
        pre_grasp_pos[2] += cfg.pen_grasp_prep_height_offset
        pre_grasp_pose = {
            "pos": pre_grasp_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_open_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        ready_to_grip_pos = deepcopy(cfg.pen_grasp_pos)
        ready_to_grip_pos[2] = cfg.pen_grasp_height
        ready_to_grip_pose = {
            "pos": ready_to_grip_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_open_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        grip_pose = {
            "pos": ready_to_grip_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        pull_out_pos = deepcopy(ready_to_grip_pos)
        pull_out_pos[1] += cfg.pull_out_y_offset
        pull_out_pose = {
            "pos": pull_out_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        pull_up_pos = deepcopy(pull_out_pos)
        pull_up_pos[2] += cfg.pull_up_z_offset
        pull_up_pose = {
            "pos": pull_up_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        reorient_pos = deepcopy(pull_up_pos)
        reorient_pos[0] = cfg.reorient_xy_pos[0]
        reorient_pos[1] = cfg.reorient_xy_pos[1]
        incline_angle = cfg.reorient_incline_angle
        reorient_rot_mat = np.array(
            [
                [0.0, np.cos(incline_angle), np.sin(incline_angle)],
                [1.0, 0.0, 0.0],
                [0.0, np.sin(incline_angle), -np.cos(incline_angle)],
            ]
        )

        flip_door_prep_pos = deepcopy(reorient_pos)
        flip_door_prep_pos[2] = cfg.flip_door_goal_height
        flip_doof_prep_pose = {
            "pos": flip_door_prep_pos,
            "bRg": reorient_rot_mat,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        flip_door_pos = deepcopy(flip_door_prep_pos)
        flip_door_pos[0] += cfg.flip_door_x_offset
        flip_door_pose = {
            "pos": flip_door_pos,
            "bRg": reorient_rot_mat,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        door_opened_pos = deepcopy(flip_door_pos)
        door_opened_pos[1] += cfg.open_door_x_offset
        door_opened_pos[2] = cfg.open_door_goal_height
        door_opened_pose = {
            "pos": door_opened_pos,
            "z_rot": cfg.door_open_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        safe_height_pos = [
            door_opened_pos[0] - cfg.post_open_door_safety_x_offset, 
            0.0, # leave y unchanged
            cfg.post_open_door_safety_goal_height
        ]
        # Make the z_rot to 0.0
        no_rot_pose = {
            "pos": safe_height_pos,
            "z_rot": cfg.default_z_rot,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        # Make the pen point downward
        pen_downward_pos = deepcopy(safe_height_pos)
        pen_downward_bRg = np.array(
            [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        )

        # Move to right above the hole
        pre_insert_pos = deepcopy(pen_downward_pos)
        pre_insert_pos[0] = cfg.probe_circuit_xy[0]  # 0.1435: center of the door, 0.012: offset
        pre_insert_pos[1] = cfg.probe_circuit_xy[1]  # 0.07630: y pos of the door hinge
        pre_insert_pose = {
            "pos": pre_insert_pos,
            "bRg": pen_downward_bRg,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        # Insert the pen into the hole
        insert_pos = deepcopy(pre_insert_pos)
        insert_pos[2] = cfg.probe_circuit_height
        insert_pose = {
            "pos": insert_pos,
            "bRg": pen_downward_bRg,
            "gripper_pos": cfg.default_gripper_close_pos,
            "vel": cfg.default_vel,
            "acc": cfg.default_acc,
        }

        # Ready to insert the pen back to the initial position
        insert_back_safe_pos = deepcopy(pre_insert_pos)
        insert_back_safe_pos[1] += cfg.insert_back_safe_xy_offset[0]
        insert_back_safe_pos[2] += cfg.insert_back_safe_xy_offset[1]

        above_ready_to_insert_pose = deepcopy(pull_up_pose)
        above_ready_to_insert_pose["pos"][2] += cfg.ready_to_insert_height_offset
        above_ready_to_insert_pose["z_rot"] = cfg.insert_z_rot

        fling_cable_pose = deepcopy(above_ready_to_insert_pose)
        fling_cable_pose["pos"][0] += cfg.fling_cable_xy_offset[0]
        fling_cable_pose["pos"][1] += cfg.fling_cable_xy_offset[1]

        ready_to_insert_pose = deepcopy(pull_out_pose)
        ready_to_insert_pose["pos"][2] = cfg.ready_to_insert_height_offset
        ready_to_insert_pose["z_rot"] = cfg.insert_z_rot

        insert_back_pose = deepcopy(grip_pose)
        insert_back_pose["pos"][1] += cfg.insert_back_x_offset
        insert_back_pose["pos"][2] += cfg.ready_to_insert_height_offset
        insert_back_pose["z_rot"] = cfg.insert_z_rot

        soft_insert_back_pose = deepcopy(insert_back_pose)
        soft_insert_back_pose["pos"][1] += cfg.soft_insert_back_y_offset
        soft_insert_back_pose["gripper_pos"] = cfg.default_gripper_open_pos

        release_gripper_pose = deepcopy(insert_back_pose)
        release_gripper_pose["pos"][2] = cfg.pen_grasp_prep_height_offset
        release_gripper_pose["gripper_pos"] = cfg.default_gripper_open_pos

        self.set_task_motions(
            [
                pre_grasp_pose,
                ready_to_grip_pose,
                grip_pose,
                pull_out_pose,
                flip_doof_prep_pose,
                flip_door_pose,
                door_opened_pose,

                # Ready to probe circuit
                no_rot_pose,
                pre_insert_pose,

                # Insert the pen into the hole
                insert_pose,
                pre_insert_pose,

                # Insert the pen back to its initial position
                fling_cable_pose,
                ready_to_insert_pose,
                soft_insert_back_pose,
                release_gripper_pose,
            ]
        )


def normalize_rot_mat(rot_mat: np.array):
    result_rot_mat = rot_mat.copy()
    for col in range(rot_mat.shape[1]):
        col_vec = rot_mat[:, col]
        result_rot_mat[:, col] = col_vec / np.linalg.norm(col_vec)
    return result_rot_mat
