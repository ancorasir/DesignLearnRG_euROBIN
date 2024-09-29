from task import Taskboard
import rtde_control
import rtde_receive

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")

taskboard_pose_in_world = [0.1416904305, -0.6037241808, 0.1020000000, 0.0000000000, 0.0000000000, 3.0212879884]

if __name__ == "__main__":
    taskboard = Taskboard(taskboard_pose_in_world)
    tcp_pose = rtde_r.getActualTCPPose()
    
    # Let the tcp go to the red button pre grasp position
    print("Red Button Pre Grasp Position: ", taskboard.red_button_world_pos_pregrasp)
    tcp_pose[:3] = taskboard.red_left_plug_world_pos_pregrasp
    rtde_c.moveL(tcp_pose, 0.25, 0.3)
    print("**")

