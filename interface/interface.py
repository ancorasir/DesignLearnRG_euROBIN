#!/usr/bin/env python3
import os
import cv2
from pathlib import Path
import numpy as np
from rerun_loader_urdf import URDFLogger
from scipy.spatial.transform import Rotation
from common import log_angle_rot, blueprint_row_images, link_to_world_transform
import rerun as rr
import argparse
import yaml


class RobotVis:
    def __init__(self, cam_dict: dict[str, dict[str, str]]):
        self.prev_joint_origins = None
        self.cam_dict = cam_dict

    def log_robot_states(
        self,
        joint_angles: np.ndarray,
        entity_to_transform: dict[str, tuple[np.ndarray, np.ndarray]],
    ):
        joint_origins = []
        for joint_idx, angle in enumerate(joint_angles):
            transform = link_to_world_transform(
                entity_to_transform, joint_angles, joint_idx + 1
            )
            joint_org = (transform @ np.array([0.0, 0.0, 0.0, 1.0]))[:3]
            joint_origins.append(joint_org)

            log_angle_rot(entity_to_transform, joint_idx + 1, angle)

        if self.prev_joint_origins is not None:
            for traj in range(len(joint_angles)):
                rr.log(
                    f"trajectory/{traj}",
                    rr.LineStrips3D(
                        [joint_origins[traj], self.prev_joint_origins[traj]],
                    ),
                )

        self.prev_joint_origins = joint_origins

    def log_camera(
        self,
        color_imgs: dict[str, np.ndarray],
        depth_imgs: dict[str, np.ndarray],
        color_extrinsics: dict[str, np.ndarray],
        color_intrinsics: dict[str, np.ndarray],
        depth_extrinsics: dict[str, np.ndarray],
        depth_intrinsics: dict[str, np.ndarray],
    ):
        for cam in self.cam_dict.keys():
            color_extrinsic = color_extrinsics[cam]
            color_intrinsic = color_intrinsics[cam]
            color_img = color_imgs[cam]
            if color_img is not None:
                rr.log(
                    f"/cameras/{cam}/color",
                    rr.Pinhole(
                        image_from_camera=color_intrinsic,
                    ),
                )
                rr.log(
                    f"/cameras/{cam}/color",
                    rr.Transform3D(
                        translation=np.array(color_extrinsic[:3]),
                        mat3x3=Rotation.from_euler(
                            "xyz", np.array(color_extrinsic[3:])
                        ).as_matrix(),
                    ),
                )
                rr.log(f"/cameras/{cam}/color", rr.Image(color_imgs[cam]))

            depth_extrinsic = depth_extrinsics[cam]
            depth_intrinsic = depth_intrinsics[cam]
            depth_img = depth_imgs[cam]
            if depth_img is not None:
                rr.log(
                    f"/cameras/{cam}/depth",
                    rr.Pinhole(
                        image_from_camera=depth_intrinsic,
                    ),
                )
                rr.log(
                    f"/cameras/{cam}/depth",
                    rr.Transform3D(
                        translation=np.array(depth_extrinsic[:3]),
                        mat3x3=Rotation.from_euler(
                            "xyz", np.array(depth_extrinsic[3:])
                        ).as_matrix(),
                    ),
                )
                rr.log(
                    f"cameras/{cam}/depth",
                    rr.DepthImage(depth_img, meter=10000),
                )

    def log_action_dict(
        self,
        tcp_pose: np.array,
        tcp_velocity: np.array,
        joint_velocity: np.array,
        gripper_position: int,
        gripper_velocity: int,
    ):
        translation = tcp_pose[:3]
        rotation_mat = Rotation.from_euler("xyz", tcp_pose[3:]).as_matrix()
        rr.log(
            "/action_dict/tcp_pose/cord",
            rr.Transform3D(translation=translation, mat3x3=rotation_mat),
        )
        rr.log("/action_dict/tcp_pose/origin", rr.Points3D([translation]))

        for i, vel in enumerate(tcp_velocity):
            rr.log(f"/action_dict/tcp_velocity/{i}", rr.Scalar(vel))

        for i, vel in enumerate(joint_velocity):
            rr.log(f"/action_dict/joint_velocity/{i}", rr.Scalar(vel))

        rr.log(
            "/action_dict/gripper_position",
            rr.Scalar(gripper_position),
        )
        rr.log(
            "/action_dict/gripper_velocity",
            rr.Scalar(gripper_velocity),
        )

    def run(self, entity_to_transform: dict[str, tuple[np.ndarray, np.ndarray]]):
        cur_time_ns = 0
        joint_angles = np.array([90, -70, 110, -130, -90, 90]) / 180 * np.pi
        gripper_position = 0.01
        self.log_robot_states(joint_angles, entity_to_transform)
        color_imgs = {}
        depth_imgs = {}
        color_extrinsics = {}
        color_intrinsics = {}
        depth_extrinsics = {}
        depth_intrinsics = {}
        color_imgs["rs-d435i"] = cv2.imread("../temp/color.png")
        depth_imgs["rs-d435i"] = cv2.imread("../temp/depth.png", cv2.IMREAD_UNCHANGED)
        color_extrinsics["rs-d435i"] = [0.1, 0.5, 0.5, 0, 0, 0]
        color_intrinsics["rs-d435i"] = [[326, 0, 391], [0, 325, 392], [0, 0, 1]]
        depth_extrinsics["rs-d435i"] = [0.1, 0.5, 0.5, 0, 0, 0]
        depth_intrinsics["rs-d435i"] = [[326, 0, 391], [0, 325, 392], [0, 0, 1]]
        color_imgs["rs-d435"] = None
        depth_imgs["rs-d435"] = None
        color_extrinsics["rs-d435"] = [-0.1, 0.5, 0.5, 0, 0, 0]
        color_intrinsics["rs-d435"] = [[326, 0, 391], [0, 325, 392], [0, 0, 1]]
        depth_extrinsics["rs-d435"] = [0.1, 0.5, 0.5, 0, 0, 0]
        depth_intrinsics["rs-d435"] = [[326, 0, 391], [0, 325, 392], [0, 0, 1]]

        self.log_camera(
            color_imgs,
            depth_imgs,
            color_extrinsics,
            color_intrinsics,
            depth_extrinsics,
            depth_intrinsics,
        )
        # self.log_action_dict()

    #         for episode in self.ds:
    #             for step in episode["steps"]:
    #                 rr.set_time_nanos("real_time", cur_time_ns)
    #                 cur_time_ns += int((1e9 * 1 / 15))
    #                 rr.log("instructions", rr.TextDocument(f'''
    # **instruction 1**: {bytearray(step["language_instruction"].numpy()).decode()}
    # **instruction 2**: {bytearray(step["language_instruction_2"].numpy()).decode()}
    # **instruction 3**: {bytearray(step["language_instruction_3"].numpy()).decode()}
    # ''',
    #                     media_type="text/markdown"))
    #                 self.log_images(step)
    #                 self.log_robot_states(step, entity_to_transform)
    #                 self.log_action_dict(step)
    #                 rr.log("discount", rr.Scalar(step["discount"]))

    def blueprint(self):
        from rerun.blueprint import (
            Blueprint,
            Horizontal,
            Vertical,
            Spatial3DView,
            TimeSeriesView,
            Tabs,
            SelectionPanel,
            TimePanel,
        )

        return Blueprint(
            Horizontal(
                Vertical(
                    Spatial3DView(name="spatial view", origin="/", contents=["/**"]),
                    blueprint_row_images(
                        [f"/cameras/{cam}" for cam in self.cam_dict.keys()]
                    ),
                    row_shares=[3, 1],
                ),
                Vertical(
                    Tabs(
                        Vertical(
                            *(
                                TimeSeriesView(
                                    origin=f"/action_dict/joint_velocity/{i}"
                                )
                                for i in range(6)
                            ),
                            name="joint velocity",
                        ),
                        Vertical(
                            *(
                                TimeSeriesView(origin=f"/action_dict/tcp_velocity/{i}")
                                for i in range(6)
                            ),
                            name="tcp velocity",
                        ),
                        Vertical(
                            TimeSeriesView(origin="/action_dict/gripper_position"),
                            TimeSeriesView(origin="/action_dict/gripper_velocity"),
                            name="gripper",
                        ),
                        active_tab=0,
                    ),
                ),
                column_shares=[3, 1],
            ),
            SelectionPanel(expanded=False),
            TimePanel(expanded=False),
        )


def main(robot: str = "ur10e_hande") -> None:
    cam_dict = yaml.load(open("../config/camera.yaml"), Loader=yaml.FullLoader)

    robot_urdf_dict = {
        "panda": "franka_description/panda.urdf",
        "panda_arg85": "franka_description/panda_arg85.urdf",
        "panda_hande": "franka_description/panda_hande.urdf",
        "panda_hande_d435i": "franka_description/panda_hande_d435i.urdf",
        "ur10e": "ur_description/ur10e.urdf",
        "ur10e_arg85": "ur_description/ur10e_arg85.urdf",
        "ur10e_hande": "ur_description/ur10e_hande.urdf",
        "ur10e_hande_d435i": "ur_description/ur10e_hande_d435i.urdf",
    }

    urdf_logger = URDFLogger(filepath=robot_urdf_dict[robot])
    robot_vis = RobotVis(cam_dict=cam_dict)

    rr.init("DROID-visualized", spawn=True)

    rr.send_blueprint(robot_vis.blueprint())

    rr.set_time_nanos("real_time", 0)
    urdf_logger.log()
    robot_vis.run(urdf_logger.entity_to_transform)


if __name__ == "__main__":
    main("ur10e_hande_d435i")
