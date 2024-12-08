import rtde_control
import rtde_receive
import numpy as np
from scipy.spatial.transform import Rotation as R

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")

pose = rtde_r.getActualTCPPose()
taskboard_pose_in_world = [
    0.1416904305,
    -0.6037241808,
    0.1020000000,
    0.0000000000,
    0.0000000000,
    3.0212879884,
]

while True:
    input("Press anything to save")
    pose = rtde_r.getActualTCPPose()
    pose_xyz = pose[:3]
    pose_rot_vec = pose[3:]

    wRg = R.from_rotvec(pose_rot_vec).as_matrix()

    wTg = np.eye(4)
    wTg[:3, :3] = wRg
    wTg[:3, 3] = pose_xyz
    wTb = np.eye(4)
    wTb[:3, :3] = R.from_rotvec(taskboard_pose_in_world[3:]).as_matrix()
    wTb[:3, 3] = taskboard_pose_in_world[:3]

    bTg = np.dot(np.linalg.inv(wTb), wTg)

    pos = bTg[:3, 3]
    rot_vec = R.from_matrix(bTg[:3, :3]).as_rotvec()
    print(pos, rot_vec)
