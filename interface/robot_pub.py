import os
import cv2
import zmq
import yaml
import time
import numpy as np
from protobuf import robot_pb2


class RobotPublisher:
    def __init__(self, address: str, cam_dict: dict) -> None:
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind(address)
        self.robot = robot_pb2.Robot()
        # self.cam_list = cam_dict.keys()
        # for cam in self.cam_list:
        #     setattr(self, str(cam), getattr(robot_pb2, str(cam).capitalize()))

    def publish_message(
        self,
        color_image: np.ndarray,
        color_extrinsics: np.ndarray,
        joint_angles: np.ndarray=np.zeros(6),
        joint_velocities: np.ndarray=np.zeros(6),
        tcp_pose: np.ndarray=np.zeros(6),
        gripper_position: int=0,
        gripper_force: int=0,
    ):
        self.robot.joint_angles[:] = joint_angles
        self.robot.joint_velocities[:] = joint_velocities
        self.robot.tcp_pose[:] = tcp_pose
        self.robot.gripper_position = gripper_position
        self.robot.gripper_force = gripper_force

        _, color_image_buf = cv2.imencode(
            ".jpg", color_image, [cv2.IMWRITE_JPEG_QUALITY, 50]
        )
        self.robot.color_image = color_image_buf.tobytes()
        self.robot.color_extrinsics[:] = color_extrinsics
        
        self.publisher.send(self.robot.SerializeToString())

def main(fps: int):
    with open("../config/address.yaml", "r") as f:
        address = yaml.load(f.read(), Loader=yaml.FullLoader)["robot_state"]

    with open("../config/camera.yaml", "r") as f:
        cam_dict = yaml.load(f.read(), Loader=yaml.FullLoader)

    robot_publisher = RobotPublisher(address=address, cam_dict=cam_dict)

    color_image_list = [cv2.imread(f"../temp/color/frame_{i}.png") for i in range(444)]

    init_joint_angles = np.array([90, -70, 110, -130, -90, 0]) / 180 * np.pi
    
    frame = len(color_image_list)
    prev_time = time.time()
    try:
        while True:
            for i in range(frame):
                joint_angles = init_joint_angles + np.pi/6*np.ones(6)*(i - frame/2)/frame
                print("%3d:"%i, joint_angles, end="\r")
                if time.time() - prev_time < 1/fps:
                    time.sleep(1/fps - (time.time() - prev_time))

                prev_time = time.time()
                robot_publisher.publish_message(
                    color_image=color_image_list[i],
                    color_extrinsics=np.array([0.5, 0, 0.5, 0, 0, 0]),
                    joint_angles=joint_angles,
                )
    except KeyboardInterrupt:
        print("\nPublisher stopped.")

if __name__ == "__main__":
    main(fps=20)