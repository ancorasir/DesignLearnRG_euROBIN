from tasks.base_task import Task
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation as R

class Trajectory:
    poses = [
        [0.03281799, 0.00984251, 0.05152254, -2.75315733,  1.45775915,  0.02300371],
        [0.03281799, 0.00984251, 0.00152254, -2.75315733,  1.45775915,  0.02300371],
        [ 0.01836395, -0.00107665,  0.00046053, -2.75856953,  1.44204326, -0.00700887],
        
        [-0.01201096, -0.02919674,  0.00046053, -2.75856953,  1.44204326, -0.00700887],

        [-0.01255011, -0.05824063,  0.01034338, -3.10735384e+00,  5.91342067e-02,  4.41367544e-04],
        [-0.0, -0.0709381,  -0.03352938, -2.27033071,  0.04323186,  0.05149873],

        [-0.0, -0.0709381,  -0.025, -2.27033071,  0.04323186,  0.05149873],
        [0.008, -0.0559381,  -0.025, -2.27033071,  0.04323186,  0.05149873],

        [ 0.18840477, -0.05612671, -0.03284369, -2.26854673,  0.04494133,  0.05404138],

        # [ 0.18724272, -0.04185066,  0.00160869, -2.26860256,  0.04499251,  0.05398487],
        [ 0.18858682, -0.04717545,  0.02086055, -2.96779512,  0.01814863,  0.01164791],

        [ 0.18739538, -0.05138632,  0.02806798, -2.8730574,  -0.02125883,  0.01721146],
        [-0.02074097, -0.04952195,  0.02807233, -2.87307385, -0.02127435,  0.01725451],
        [-0.00684195, -0.0583229,  -0.0277897 , -2.41079444, -0.02099093,  0.02537869],
        [ 0.18737535, -0.05438033, -0.03002041, -2.41076831, -0.02105357,  0.02536871],
        [ 0.18779979, -0.04566536, -0.00074528, -2.41081376, -0.02102119,  0.02532422],

        [ 0.18858682, -0.04717545,  0.02086055, -2.96779512,  0.01814863,  0.01164791],

        [-0.01130823, -0.04633783,  0.02082222, -2.96786907,  0.01809708,  0.0115993 ],
        [-0.00531653, -0.05971264, -0.01688192, -2.33114159,  0.21617186, -0.38724675],
        [ 0.01477842, -0.0703503,  -0.00226553, -2.30918127, -0.10027959, -1.08731785],

        [ 0.01477842, -0.1003503,  -0.00226553, -2.30918127, -0.10027959, -1.08731785],

        [ 0.00458486, -0.08392065,  0.0497292 , -3.06617412, -0.08224906,  0.01348425],
    ]

    gripper_pos = [
        120,
        190,
        250,
        250,
        250,
        250,
        250,
        250,
        250,
        250,
        250,
        250,
        250,
        250,
        250,
        250,
        250,
        150,
        150,
        150,
        150
    ]

    vel = [
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
        2.0,
    ]

    acc = [
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
        4.0,
    ]

class WrapCable(Task):
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
        
        self.task_motions = []

        for i in range (len(Trajectory.poses)):
            pose_xyz = Trajectory.poses[i][:3]
            pose_rot_vec = Trajectory.poses[i][3:]
            pose_rot_mat = R.from_rotvec(pose_rot_vec).as_matrix()

            motion = {
                "pos": pose_xyz,
                "bRg": pose_rot_mat,
                "gripper_pos": Trajectory.gripper_pos[i],
                "vel": Trajectory.vel[i],
                "acc": Trajectory.acc[i]
            }

            self.task_motions.append(motion)

        self.set_task_motions(self.task_motions)

    def execute(self):
        for i, pose in enumerate(self.task_motions):
            # if i == len(self.task_motions) - 2:
            #     # The second last one, first close gripper without waiting
            #     # at the same time move the TCP
            #     self._gripper_ctrl.setPosition(250)
            self.go_to_pose(pose)
