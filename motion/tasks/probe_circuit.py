from tasks.base_task import Task
from copy import deepcopy
import numpy as np
import time

class ProbeCircuit(Task):
    def __init__(
            self, 
            rtde_rec, 
            rtde_ctrl, 
            gripper_ctrl, 
            taskboard_pose_in_world, 
            default_tcp_rot_mat_in_board=None
        ) -> None:

        super().__init__(
            rtde_rec=rtde_rec, 
            rtde_ctrl=rtde_ctrl, 
            gripper_ctrl=gripper_ctrl, 
            task_motions=None, 
            taskboard_pose_in_world=taskboard_pose_in_world, 
            default_tcp_rot_mat_in_board=default_tcp_rot_mat_in_board)
        
        self.pen_grasp_pos = [0.1985, -0.02, 0.0]

        self._gripping_pos = 230
        
        pre_grasp_pos = deepcopy(self.pen_grasp_pos)
        pre_grasp_pos[2] += 0.03
        pre_grasp_pose = {
            "pos": pre_grasp_pos,
            "z_rot": 0.0,
            "gripper_pos": 150,
            "vel": 2.0,
            "acc": 4.0
        }

        ready_to_grip_pos = deepcopy(self.pen_grasp_pos)
        ready_to_grip_pos[2] = 0.001
        ready_to_grip_pose = {
            "pos": ready_to_grip_pos,
            "z_rot": 0.0,
            "gripper_pos": 150,
            "vel": 2.0,
            "acc": 4.0
        }

        grip_pose = {
            "pos": ready_to_grip_pos,
            "z_rot": 0.0,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        pull_out_pos = deepcopy(ready_to_grip_pos)
        pull_out_pos[1] -= 0.1
        pull_out_pose = {
            "pos": pull_out_pos,
            "z_rot": 0.0,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        pull_up_pos = deepcopy(pull_out_pos)
        pull_up_pos[2] += 0.1
        pull_up_pose = {
            "pos": pull_up_pos,
            "z_rot": 0.0,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        reorient_pos = deepcopy(pull_up_pos)
        reorient_pos[0] = 0.32
        reorient_pos[1] = 0.01
        incline_angle = 30.0 / 180.0 * np.pi
        reorient_rot_mat = np.array([
            [0.0, np.cos(incline_angle), np.sin(incline_angle)],
            [1.0, 0.0, 0.0],
            [0.0, np.sin(incline_angle), -np.cos(incline_angle)]
        ])
        reorient_pose = {
            "pos": reorient_pos,
            "bRg": reorient_rot_mat,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        flip_door_prep_pos = deepcopy(reorient_pos)
        flip_door_prep_pos[2] = 0.046
        flip_doof_prep_pose = {
            "pos": flip_door_prep_pos,
            "bRg": reorient_rot_mat,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        flip_door_pos = deepcopy(flip_door_prep_pos)
        flip_door_pos[0] -= 0.035
        flip_door_pose = {
            "pos": flip_door_pos,
            "bRg": reorient_rot_mat,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        door_opened_pos = deepcopy(flip_door_pos)
        door_opened_pos[0] -= 0.0 # X to negative a little bit, avoid pen tip losing contact with the door
        door_opened_pos[1] += 0.13
        door_opened_pos[2] = 0.05
        door_opened_pose = {
            "pos": door_opened_pos,
            "z_rot": np.pi/2.0,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        safe_height_pos = [door_opened_pos[0]-0.05, 0.0, 0.15]
        safe_height_pose = {
            "pos": safe_height_pos,
            "z_rot": np.pi/2.0,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        # Make the z_rot to 0.0
        no_rot_pose = {
            "pos": safe_height_pos,
            "z_rot": 0.0,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        # Make z_rot to -pi/2.0
        z_rot_neg_90deg_pose = {
            "pos": safe_height_pos,
            "z_rot": -np.pi/2.0,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        # Make the pen point downward
        pen_downward_pos = deepcopy(safe_height_pos)
        pen_downward_bRg = np.array([
            [0.0,  0.0,  -1.0],
            [-1.0, 0.0, 0.0],
            [0.0,  1.0,  0.0]
        ])
        pen_downward_pose = {
            "pos": pen_downward_pos,
            "bRg": pen_downward_bRg,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        # Move to right above the hole
        pre_insert_pos = deepcopy(pen_downward_pos)
        pre_insert_pos[0] = 0.1435 - 0.0025 # 0.1435: center of the door, 0.012: offset
        pre_insert_pos[1] = 0.07630 - 0.024 # 0.07630: y pos of the door hinge
        pre_insert_pose = {
            "pos": pre_insert_pos,
            "bRg": pen_downward_bRg,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        # Insert the pen into the hole
        insert_pos = deepcopy(pre_insert_pos)
        insert_pos[2] = 0.088
        insert_pose = {
            "pos": insert_pos,
            "bRg": pen_downward_bRg,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        # Ready to insert the pen back to the initial position
        insert_back_safe_pos = deepcopy(pre_insert_pos)
        insert_back_safe_pos[1] -= 0.1
        insert_back_safe_pos[2] -= 0.05
        insert_back_safe_pose = {
            "pos": insert_back_safe_pos,
            "z_rot": 0.0,
            "gripper_pos": self._gripping_pos,
            "vel": 2.0,
            "acc": 4.0
        }

        insert_z_rot = -2 / 180.0 * np.pi # 5 degrees of offset
        above_ready_to_insert_pose = deepcopy(pull_up_pose)
        above_ready_to_insert_pose["pos"][2] += 0.004
        above_ready_to_insert_pose["z_rot"] = insert_z_rot

        fling_cable_pose = deepcopy(above_ready_to_insert_pose)
        fling_cable_pose["pos"][0] += 0.15
        fling_cable_pose["pos"][1] += 0.05

        ready_to_insert_pose = deepcopy(pull_out_pose)
        ready_to_insert_pose["pos"][2] = 0.004
        ready_to_insert_pose["z_rot"] = insert_z_rot

        insert_back_pose = deepcopy(grip_pose)
        insert_back_pose["pos"][1] -= 0.01 # 0.012
        insert_back_pose["pos"][2] += 0.004
        insert_back_pose["z_rot"] = insert_z_rot
        # insert_back_pose["gripper_pos"] = 230

        soft_insert_back_pose = deepcopy(insert_back_pose)
        soft_insert_back_pose["pos"][1] += 0.015
        soft_insert_back_pose["gripper_pos"] = 150

        release_gripper_pose = deepcopy(insert_back_pose)
        release_gripper_pose["pos"][2] = 0.05
        release_gripper_pose["gripper_pos"] = 150

        self.set_task_motions([
            pre_grasp_pose,
            ready_to_grip_pose,
            grip_pose,
            pull_out_pose,
            # pull_up_pose,

            # Open the door
            # reorient_pose,
            flip_doof_prep_pose,
            flip_door_pose,
            door_opened_pose,

            # Ready to probe circuit
            # safe_height_pose,
            no_rot_pose,
            # z_rot_neg_90deg_pose,
            # pen_downward_pose,
            pre_insert_pose,

            # Insert the pen into the hole
            insert_pose,
            pre_insert_pose,

            # Insert the pen back to its initial position
            # insert_back_safe_pose,
            fling_cable_pose,
            # above_ready_to_insert_pose,
            ready_to_insert_pose,
            # insert_back_pose,
            soft_insert_back_pose,
            release_gripper_pose,

            # pre_grasp_pose,
        ])

def normalize_rot_mat(rot_mat: np.array):
    result_rot_mat = rot_mat.copy()
    for col in range(rot_mat.shape[1]):
        col_vec = rot_mat[:, col]
        result_rot_mat[:, col] = col_vec / np.linalg.norm(col_vec)
    return result_rot_mat