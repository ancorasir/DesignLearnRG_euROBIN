#!/usr/bin/env python3
import sys
import os
import time
import yaml
import ast
import socket
import asyncio
import websockets
import contextlib
import zmq
import cv2
import numpy as np
from rerun_loader_urdf import URDFLogger
from scipy.spatial.transform import Rotation
from protobuf import robot_pb2
from common import log_angle_rot, blueprint_row_images, link_to_world_transform
from http.server import (
    SimpleHTTPRequestHandler,
    BaseHTTPRequestHandler,
    ThreadingHTTPServer,
)
from functools import partial
from multiprocessing import Process
import rerun as rr


class DualStackServer(ThreadingHTTPServer):
    def server_bind(self):
        # suppress exception when protocol is IPv4
        with contextlib.suppress(Exception):
            self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        return super().server_bind()


class TestHTTPHandle(BaseHTTPRequestHandler):
    def do_POST(self):
        print(self.headers)
        content_len = int(self.headers.get("content-length", 0))
        post_body = self.rfile.read(content_len)
        print("receive message from server: ")
        print(post_body)
        self.send_response(200)
        self.end_headers()


class RobotSubscriber:
    def __init__(self, address: str) -> None:
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect(address)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.joint_angles = np.zeros(6)
        self.joint_velocities = np.zeros(6)
        self.tcp_pose = np.zeros(6)
        self.gripper_position = 0
        self.gripper_force = 0

    def receive_message(self):
        robot_state = robot_pb2.Robot()
        robot_state.ParseFromString(self.subscriber.recv())

        self.joint_angles = np.array(robot_state.joint_angles).flatten()
        self.joint_velocities = np.array(robot_state.joint_velocities).flatten()
        self.tcp_pose = np.array(robot_state.tcp_pose).flatten()
        self.gripper_position = np.array(robot_state.gripper_position).flatten()
        self.gripper_force = np.array(robot_state.gripper_force).flatten()


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
        depth_units: int = 0.001,
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
                    rr.DepthImage(depth_img, meter=10 / depth_units),
                )

    def log_action_dict(
        self,
        tcp_pose: np.ndarray = np.array([0, 0, 0, 0, 0, 0]),
        joint_velocity: np.ndarray = np.array([0, 0, 0, 0, 0, 0]),
        gripper_position: int = 0,
        gripper_velocity: int = 0,
    ):

        for i, val in enumerate(tcp_pose):
            rr.log(f"/action_dict/tcp_pose/{i}", rr.Scalar(val))

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
        self.log_robot_states(joint_angles, entity_to_transform)
        color_imgs = {}
        depth_imgs = {}
        color_extrinsics = {}
        color_intrinsics = {}
        depth_extrinsics = {}
        depth_intrinsics = {}
        color_imgs["rs-d435i"] = cv2.imread("../temp/color.png")
        # depth_imgs["rs-d435i"] = cv2.imread("../temp/depth.png", cv2.IMREAD_UNCHANGED)
        depth_imgs["rs-d435i"] = None
        color_extrinsics["rs-d435i"] = [0.1, 0.5, 0.5, 0, 0, 0]
        color_intrinsics["rs-d435i"] = [[326, 0, 391], [0, 325, 392], [0, 0, 1]]
        depth_extrinsics["rs-d435i"] = [0.1, 0.5, 0.5, 0, 0, 0]
        depth_intrinsics["rs-d435i"] = [[326, 0, 391], [0, 325, 392], [0, 0, 1]]

        try:
            while True:
                self.log_robot_states(joint_angles, entity_to_transform)
                self.log_camera(
                    color_imgs,
                    depth_imgs,
                    color_extrinsics,
                    color_intrinsics,
                    depth_extrinsics,
                    depth_intrinsics,
                )
                self.log_action_dict()
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

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
                        [f"/cameras/{cam}/color" for cam in self.cam_dict.keys()]
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
                                TimeSeriesView(origin=f"/action_dict/tcp_pose/{i}")
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


def rerun_server(
    robot: str = "ur10e_hande",
):
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

    rr.init("Robot Interface")
    rr.serve(open_browser=False, ws_port=4321, web_port=8000)
    rr.send_blueprint(robot_vis.blueprint())
    rr.set_time_nanos("real_time", 0)
    time.sleep(1)

    urdf_logger.log()
    robot_vis.run(urdf_logger.entity_to_transform)


def web_server(
    server_class=DualStackServer,
    handler_class=SimpleHTTPRequestHandler,
    bind: str = "127.0.0.1",
    port: int = 8000,
):
    handler_class = partial(SimpleHTTPRequestHandler, directory=os.getcwd())
    with server_class((bind, port), handler_class) as httpd:
        print(f"Serving HTTP on {bind} port {port} " f"(http://{bind}:{port}/) ...")
        httpd.serve_forever()


def blueprint_server(
    action_dict: dict,
    bind: str = "localhost",
    port: int = 4322,
):
    async def echo(websocket, path):
        async for message in websocket:
            # save the message to a file
            print(f"receive message: {message}")
            msg_list = ast.literal_eval(message)
            msg_dict = {}
            for i, msg in enumerate(msg_list):
                msg_dict[f"action_{i}"] = {
                    "id": msg["node"],
                    "name": action_dict[str(msg["node"])],
                }
            with open("../temp/action.yaml", "w") as f:
                f.write(yaml.dump(msg_dict))
            await websocket.send(f"Echo: {message}")

    start_server = websockets.serve(echo, bind, port)

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()


def main(
    action_dict: str,
    robot: str = "ur10e_hande",
    server_class=DualStackServer,
    handler_class=SimpleHTTPRequestHandler,
    bind: str = "127.0.0.1",
    port: int = 8000,
) -> None:

    web_process = Process(
        target=web_server,
        args=(
            server_class,
            handler_class,
            bind,
            port,
        ),
    )
    web_process.daemon = True
    web_process.start()

    blueprint_process = Process(
        target=blueprint_server, args=(action_dict, "localhost", 4322)
    )
    blueprint_process.daemon = True
    blueprint_process.start()

    rerun_server(robot=robot)


if __name__ == "__main__":
    action_dict = {
        "0": "find board & press button",
        "1": "move slider to setpoints on screen",
        "2": "plug in probe into test port",
        "3": "open door, probe circuit",
        "4": "wrap cable, replace probe",
        "5": "press stop trial button",
    }
    main(action_dict=action_dict, robot="ur10e_hande_d435i")
