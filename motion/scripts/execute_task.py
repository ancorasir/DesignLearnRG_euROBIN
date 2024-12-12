import os
import sys
motion_dir = os.path.dirname(os.path.dirname(__file__))
root_dir = os.path.dirname(motion_dir)
sys.path.append(f"{root_dir}")
sys.path.append(f"{motion_dir}")
sys.path.append(f"{motion_dir}/lib")

import rtde_control
import rtde_receive
from motion.message.vision_socket import VisionSocket
import gripper
from motion.utils import TaskCfg, parse_execute_sequence

cfg_path = f"{root_dir}/config/task.yaml"
skill_list_cfg_path = f"{root_dir}/config/skill_list.yaml"
skill_lookup_cfg_path = f"{root_dir}/config/skill.yaml"
cfg = TaskCfg(cfg_path)
skill_list = parse_execute_sequence(
    skill_list_cfg_path,
    skill_lookup_cfg_path
)

# Establish connection with the robot
rtde_r = rtde_receive.RTDEReceiveInterface(cfg.ur_robot_ip)
rtde_c = rtde_control.RTDEControlInterface(cfg.ur_robot_ip)

# Establish connection with the gripper
hande = gripper.Hand_E(cfg.hande_gripper_serial_port)
hande.connect()
hande.activate()

# Set up the connection with the vision server
vision_server = VisionSocket(cfg.vision_server_ip)

print("Moving to observation pose...")
hande.setPosition(cfg.default_gripper_close_pos) # Close gripper
rtde_c.moveL(cfg.observation_pose, cfg.observation_vel, cfg.observation_acc) # move to obs pose

# Get taskboard pose in world from vision server
taskboard_pose_in_world = vision_server.get_taskboard_pose_in_world()

for skill_desc in skill_list:
    print(f"Executing skill: {skill_desc}")
    if skill_desc == "find board & press button":
        from tasks.press_button import PressBlueButton
        press_blue_button = PressBlueButton(
            rtde_rec=rtde_r,
            rtde_ctrl=rtde_c,
            gripper_ctrl=hande,
            taskboard_pose_in_world=taskboard_pose_in_world,
            default_tcp_rot_mat_in_board=None,
        )
        press_blue_button.execute()

    elif skill_desc == "move slider to setpoints on screen":
        from tasks.move_slider import MoveSlider
        move_slider = MoveSlider(
            rtde_rec=rtde_r,
            rtde_ctrl=rtde_c,
            gripper_ctrl=hande,
            taskboard_pose_in_world=taskboard_pose_in_world,
            default_tcp_rot_mat_in_board=None,
            vision_server=vision_server,
        )
        move_slider.execute()

    elif skill_desc == "plug in probe into test port":
        from tasks.plug_probe import PlugProbe
        plug_probe = PlugProbe(
            rtde_rec=rtde_r,
            rtde_ctrl=rtde_c,
            gripper_ctrl=hande,
            taskboard_pose_in_world=taskboard_pose_in_world,
            default_tcp_rot_mat_in_board=None,
        )
        plug_probe.execute()

    elif skill_desc == "open door, probe circuit":
        from tasks.probe_circuit import ProbeCircuit
        probe_circuit = ProbeCircuit(
            rtde_rec=rtde_r,
            rtde_ctrl=rtde_c,
            gripper_ctrl=hande,
            taskboard_pose_in_world=taskboard_pose_in_world,
            default_tcp_rot_mat_in_board=None,
        )
        probe_circuit.execute()

    elif skill_desc == "wrap cable, replace probe":
        from tasks.wrap_cable import WrapCable
        wrap_cable = WrapCable(
            rtde_rec=rtde_r,
            rtde_ctrl=rtde_c,
            gripper_ctrl=hande,
            taskboard_pose_in_world=taskboard_pose_in_world,
            default_tcp_rot_mat_in_board=None,
        )
        wrap_cable.execute()
    
    elif skill_desc == "press stop trial button":
        from tasks.press_button import PressRedButton
        press_red_button = PressRedButton(
            rtde_rec=rtde_r,
            rtde_ctrl=rtde_c,
            gripper_ctrl=hande,
            taskboard_pose_in_world=taskboard_pose_in_world,
            default_tcp_rot_mat_in_board=None,
        )
        press_red_button.execute()

hande.setPosition(cfg.default_gripper_close_pos)
print("Press red button task in motion...")
press_red_button.execute()

print("Moving to observation pose...")
rtde_c.moveL(cfg.observation_pose, cfg.observation_vel, cfg.observation_acc)
hande.setPosition(cfg.default_gripper_close_pos)
