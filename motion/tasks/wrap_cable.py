from tasks.base_task import Task
from scipy.spatial.transform import Rotation as R
from motion.utils import WrapCableCfg, config_dir

class WrapCable(Task):
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

        cfg = WrapCableCfg(f"{config_dir}/wrap_cable.yaml")
        self.cfg = cfg

        self.task_motions = []

        for i in range(len(cfg.poses)):
            pose_xyz = cfg.poses[i][:3]
            pose_rot_vec = cfg.poses[i][3:]
            pose_rot_mat = R.from_rotvec(pose_rot_vec).as_matrix()

            motion = {
                "pos": pose_xyz,
                "bRg": pose_rot_mat,
                "gripper_pos": cfg.gripper_pos[i],
                "vel": cfg.vel[i],
                "acc": cfg.acc[i],
            }

            self.task_motions.append(motion)

        self.set_task_motions(self.task_motions)
