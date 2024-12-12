import os

root_dir = os.path.dirname(os.path.dirname(__file__))
config_dir = f"{root_dir}/config"

import yaml
from math import pi

class BaseCfg:
    def __init__(self, cfg_path: str=f"{root_dir}/config/task.yaml") -> None:
        self._cfg_path = cfg_path
        # Read the config
        with open(self._cfg_path, "r") as yaml_file:
            self._cfg = yaml.load(yaml_file, Loader=yaml.FullLoader)

    def get_cfg(self, *cfg_names: str):
        """
            Get the config from the config file, give the key cfg names

            cfg_names: str, the key names in the config file

            return: the config value

            example:
                get_cfg("press_button_task", "button_height_offset")
        """
        cfg = self._cfg
        for cfg_name in cfg_names:
            cfg = cfg[cfg_name]
        return cfg
    
    """
        Some common cfg that might be used for all tasks
    """
    @property
    def pos_on_board(self):
        return self.get_cfg("pos_on_board")
    
    @property
    def default_vel(self):
        return self.get_cfg("default_vel")
    
    @property
    def default_acc(self):
        return self.get_cfg("default_acc")
    
    @property
    def default_gripper_open_pos(self):
        return self.get_cfg("default_gripper_open_pos")
    
    @property
    def default_gripper_close_pos(self):
        return self.get_cfg("default_gripper_close_pos")
    
    @property
    def default_z_rot_degree(self):
        return self.get_cfg("default_z_rot_degree")
    
    @property
    def default_z_rot(self):
        return (self.default_z_rot_degree / 180) * pi

class TaskCfg(BaseCfg):
    @property
    def ur_robot_ip(self):
        return self.get_cfg("ur_robot_ip")
    
    @property
    def hande_gripper_serial_port(self):
        return self.get_cfg("hande_gripper_serial_port")
    
    @property
    def vision_server_ip(self):
        return self.get_cfg("vision_server_ip")
    
    @property
    def observation_pose(self):
        return self.get_cfg("observation_pose")
    
    @property
    def observation_vel(self):
        return self.get_cfg("observation_vel")
    
    @property
    def observation_acc(self):
        return self.get_cfg("observation_acc")
    
class PressButtonCfg(BaseCfg):
    @property
    def press_height(self):
        return self.get_cfg("press_height")
    
    @property
    def press_prep_height_offset(self):
        return self.get_cfg("press_prep_height_offset")
    
class MoveSliderCfg(BaseCfg):
    @property
    def grip_slider_z(self):
        return self.get_cfg("grip_slider_z")
    
    @property
    def grip_slider_prep_height_offset(self):
        return self.get_cfg("grip_slider_prep_height_offset")
    
    @property
    def slide_vel(self):
        return self.get_cfg("slide_vel")
    
    @property
    def slide_acc(self):
        return self.get_cfg("slide_acc")
    
    @property
    def pixel_to_distance_ratio(self):
        return self.get_cfg("pixel_to_distance_ratio")
    
    @property
    def max_y_increment(self):
        return self.get_cfg("max_y_increment")
    
    @property
    def min_y_increment(self):
        return self.get_cfg("min_y_increment")
    
    @property
    def release_gripper_pos(self):
        return self.get_cfg("release_gripper_pos")
    
class PlugProbeCfg(BaseCfg):
    @property
    def goal_pos_on_board(self):
        return self.get_cfg("goal_pos_on_board")
    
    @property
    def pre_grasp_z(self):
        return self.get_cfg("pre_grasp_z")
    
    @property
    def grasp_z(self):
        return self.get_cfg("grasp_z")
    
    @property
    def plug_in_z_rot(self):
        return (self.get_cfg("plug_in_z_rot_degree")/180) * pi
    
class ProbeCircuitCfg(BaseCfg):
    @property
    def pen_grasp_pos(self):
        return self.get_cfg("pen_grasp_pos")
    
    @property
    def pen_grasp_height(self):
        return self.get_cfg("pen_grasp_height")
    
    @property
    def pen_grasp_prep_height_offset(self):
        return self.get_cfg("pen_grasp_prep_height_offset")
    
    @property
    def door_open_z_rot(self):
        return (self.get_cfg("door_open_z_rot_degree")/180) * pi
    
    @property
    def pull_out_y_offset(self):
        return self.get_cfg("pull_out_y_offset")
    
    @property
    def pull_up_z_offset(self):
        return self.get_cfg("pull_up_z_offset")
    
    @property
    def reorient_xy_pos(self):
        return self.get_cfg("reorient_xy_pos")
    
    @property
    def reorient_incline_angle(self):
        return (self.get_cfg("reorient_incline_degree")/180) * pi
    
    @property
    def flip_door_goal_height(self):
        return self.get_cfg("flip_door_goal_height")
    
    @property
    def flip_door_x_offset(self):
        return self.get_cfg("flip_door_x_offset")
    
    @property
    def open_door_x_offset(self):
        return self.get_cfg("open_door_x_offset")
    
    @property
    def open_door_goal_height(self):
        return self.get_cfg("open_door_goal_height")
    
    @property
    def post_open_door_safety_x_offset(self):
        return self.get_cfg("post_open_door_safe_x_offset")
    
    @property
    def post_open_door_safety_goal_height(self):
        return self.get_cfg("post_open_door_safe_goal_height")
    
    @property
    def probe_circuit_xy(self):
        return self.get_cfg("probe_circuit_xy")
    
    @property
    def probe_circuit_height(self):
        return self.get_cfg("probe_circuit_height")
    
    @property
    def insert_back_safe_xy_offset(self):
        return self.get_cfg("insert_back_safe_xy_offset")
    
    @property
    def insert_z_rot(self):
        return (self.get_cfg("insert_z_rot_degree")/180) * pi
    
    @property
    def ready_to_insert_height_offset(self):
        return self.get_cfg("ready_to_insert_height_offset")
    
    @property
    def fling_cable_xy_offset(self):
        return self.get_cfg("fling_cable_xy_offset")
    
    @property
    def insert_back_x_offset(self):
        return self.get_cfg("insert_back_x_offset")
    
    @property
    def soft_insert_back_y_offset(self):
        return self.get_cfg("soft_insert_back_y_offset")
    
class WrapCableCfg(BaseCfg):
    @property
    def poses(self):
        return self.get_cfg("poses")
    
    @property
    def gripper_pos(self):
        return self.get_cfg("gripper_pos")
    
    @property
    def vel(self):
        return self.get_cfg("vel")
    
    @property
    def acc(self):
        return self.get_cfg("acc")

def parse_execute_sequence(skill_list_cfg_path, skill_lookup_cfg_path):
    with open(skill_list_cfg_path, "r") as yaml_file:
        skill_list_cfg = yaml.load(yaml_file, Loader=yaml.FullLoader)

    with open(skill_lookup_cfg_path, "r") as yaml_file:
        skill_lookup_cfg = yaml.load(yaml_file, Loader=yaml.FullLoader)

    task_list = []

    step_nums = []
    for step_keys in skill_list_cfg.keys():
        # Every step key is in form of step_i,
        # Exact the step number from the key
        step_num = int(step_keys.split("_")[1])
        step_nums.append(step_num)

    # Sort the steps keys by the step number
    step_nums.sort()
    step_keys = [f"step_{step_num}" for step_num in step_nums]

    for step_key in step_keys:
        skill_id = str(skill_list_cfg[step_key]["id"])
        task_list.append(skill_lookup_cfg[skill_id])

    return task_list