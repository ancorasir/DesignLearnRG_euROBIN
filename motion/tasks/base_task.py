import numpy as np
from scipy.spatial.transform import Rotation as R
import time

class Task:
    '''
        Pipeline of the task:
            1. Go to the pre grasp pose (right above the element with a z offset value)
            2. Execute a sequence of pose defined by task_motions
            3. Return back to the pre grasp pose

        Each pose (pre grasp pose and poses in task_motions) is a dict:
            pose = {
                "pos": [x, y, z], relative to the task board frame
                "z_rot": rotation of the tcp about the z-axis, with 0 rad pointing towards the positive x-axis of the taskboard
                "gripper_pos": 0~255 (0: open, 255: close)
            }

        The gripper will be controlled after the tcp goes to the desired pose
        
    '''
    def __init__(
            self, 
            rtde_rec, 
            rtde_ctrl, 
            gripper_ctrl,
            task_motions=[], 
            taskboard_pose_in_world=None,
            default_tcp_rot_mat_in_board=None,
        ) -> None:
        '''
        Args:
            rtde_rec (rtde_receive.RTDEReceiveInterface): RTDE receiver
            rtde_ctrl (rtde_control.RTDEControlInterface): RTDE controller
            gripper_ctrl (gripper.Hand_E): gripper controller
            pre_grasp_pose (dict): pre grasp pose
            task_motions (list): list of poses to complete the task
            default_tcp_rot_mat_in_board (np.array): default tcp rotation matrix relative to the task board frame
            taskboard_pose_in_world: [x, y, z, rx, ry, rz] 

        Symbols:
            w: world frame
            b: task board frame
            e: task board element frame
            g: gripper frame
        '''

        self.taskboard_pose_in_world = taskboard_pose_in_world
        self.wRb = R.from_rotvec(taskboard_pose_in_world[3:]).as_matrix()
        self.wTb = np.eye(4)
        self.wTb[:3, :3] = self.wRb
        self.wTb[:3, 3] = taskboard_pose_in_world[:3]

        self.task_motions = task_motions

        self._rtde_rec = rtde_rec
        self._rtde_ctrl = rtde_ctrl
        self._gripper_ctrl = gripper_ctrl

        self.__gripper_stop_threshold = 20 # if the gripper is within 20 from the target position, stop the loop
        self.__gripper_max_waiting_time = 3 # 3 seconds max waiting time
        if default_tcp_rot_mat_in_board is not None:
            self.__bRg = default_tcp_rot_mat_in_board
        else:
            self.__bRg = np.array([ # rotation of the gripper in the task board frame
                [1.0, 0.0,  0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0,  -1.0]
            ])

    def set_task_motions(self, task_motions):
        self.task_motions = task_motions

    def go_to_pose(self, pose, asynchronous=False, wait_for_gripper=True):
        '''
            Command the TCP to the pose defined by the pose["pos"] and pose["z_rot"]
            Control the gripper to the pose["gripper_pos"]
        '''
        
        ur_tcp_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # self._rtde_rec.getActualTCPPose()
        ur_tcp_pose[:3] = self.calculate_pos_in_world(pose["pos"])

        # make the tcp's x axis align with the task board's x-axis
        wRg = np.dot(self.wRb, self.__bRg)

        if "bRg" in pose.keys():
            bRg = pose["bRg"]
            tcp_rot_world = np.dot(self.wRb, bRg)
            tcp_rot_vec_new = R.from_matrix(tcp_rot_world).as_rotvec()
        else:
            # For the z_rot, we will apply the rotation to the current tcp pose
            tcp_rot_vec = np.array([0.0, 0.0, -pose["z_rot"]])
            tcp_rot_mat = R.from_rotvec(tcp_rot_vec).as_matrix()
            tcp_rot_mat_new = np.dot(wRg, tcp_rot_mat)
            tcp_rot_vec_new = R.from_matrix(tcp_rot_mat_new).as_rotvec()

        ur_tcp_pose[3:] = tcp_rot_vec_new.tolist()

        if "vel" in pose.keys():
            movel_vel = pose["vel"]
            assert movel_vel > 0 and movel_vel <= 5.0
        else:
            movel_vel = 0.25
        
        if "acc" in pose.keys():
            movel_acc = pose["acc"]
            assert movel_acc > 0 and movel_acc <= 5.0
        else:
            movel_acc = 0.3
            
        self._rtde_ctrl.moveL(ur_tcp_pose, movel_vel, movel_acc, asynchronous)
        
        # Control the gripper
        self._gripper_ctrl.setPosition(pose["gripper_pos"])   
        if wait_for_gripper:
            current_gripper_pos = self._gripper_ctrl.getPosition()
            waiting_time = 0
            while abs(current_gripper_pos - pose["gripper_pos"]) > self.__gripper_stop_threshold and \
                waiting_time < self.__gripper_max_waiting_time:
                current_gripper_pos = self._gripper_ctrl.getPosition()
                waiting_time += 0.1
                time.sleep(0.1)


    def execute(self):
        for pose in self.task_motions:
            self.go_to_pose(pose)

    def calculate_pos_in_world(self, element_pos_in_taskboard):
        '''
            Given the element's position in the task board frame, calculate its position in the world frame

            wPe = wTb * bPe (T is the transformation matrix, P is the position vector)
        '''
        homo_coord = np.array([0.0, 0.0, 0.0, 1.0])
        homo_coord[:3] = element_pos_in_taskboard
        element_pos_in_world = np.dot(self.wTb, homo_coord)[:3]
        return element_pos_in_world.tolist()