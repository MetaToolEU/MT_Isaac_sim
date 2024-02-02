from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.motion_generation.lula import RmpFlow
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import cuboid
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.motion_generation.interface_config_loader import load_supported_motion_policy_config
import rtde_control
import rtde_receive
from Gripper_control import UR3eGripper
import numpy as np
import argparse
import sys
import time
import os
script_dir = os.path.dirname(os.path.abspath(__file__))

# Set up argument parsers
parser = argparse.ArgumentParser()
parser.add_argument("--robot-ip", type=str, default="192.168.10.23", help="IP address of the first robot")
arg = parser.parse_args()
rtde_receive_interface = rtde_receive.RTDEReceiveInterface(arg.robot_ip)

ur3e_gripper_instance = UR3eGripper(arg.robot_ip,rtde_receive_interface,30004)

# ur3e_gripper_instance = UR3eGripper(ip_address=arg.robot_ip)

parser2 = argparse.ArgumentParser()
parser2.add_argument("--robot-ip2", type=str, default="192.168.10.25", help="IP address of the second robot")
arg_2 = parser2.parse_args()
rtde_receive_interface_2 = rtde_receive.RTDEReceiveInterface(arg_2.robot_ip2)

ur3e_gripper_instance_2 = UR3eGripper(arg_2.robot_ip2,rtde_receive_interface_2,30004)

# ur3e_gripper_instance_2 = UR3eGripper(ip_address=arg_2.robot_ip2)
# Set up paths and prims
robot_name = "UR3e"
prim_path = "/UR3e"
usd_path = os.path.join(script_dir, "../usd/flatten_updated_1.usd")

robot_name_2 = "UR3e_2"
prim_path_2 = "/UR3e_2"

# Set references to stage in Isaac Sim
add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
add_reference_to_stage(usd_path=usd_path, prim_path=prim_path_2)

# Add world
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# Add robots to the world
robot = my_world.scene.add(Robot(prim_path=prim_path, name=robot_name, position=np.array([0, 1.0, 0])))
robot_2 = my_world.scene.add(Robot(prim_path=prim_path_2, name=robot_name_2))

# Load supported motion policy configurations
rmp_config = load_supported_motion_policy_config(robot_name, "RMPflow")

# Initialize RmpFlow objects and set up
# Robot 1
rmpflow = RmpFlow(            
            robot_description_path = os.path.join(script_dir, "../motion_policy_configs/ur3e/rmpflow/ur3e_gripper_robot_description.yaml"),
            urdf_path =  os.path.join(script_dir, "../motion_policy_configs/ur3e/ur3e_gripper.urdf"),
            rmpflow_config_path = os.path.join(script_dir, "../motion_policy_configs/ur3e/rmpflow/ur3e_gripper_rmpflow_config.yaml"),
            end_effector_frame_name = "gripper_center",
            maximum_substep_size = 0.00334
        )
physics_dt = 1.0 / 60

articulation_rmpflow = ArticulationMotionPolicy(robot, rmpflow, physics_dt)
articulation_controller = robot.get_articulation_controller()

# Robot 2
rmpflow_2 = RmpFlow(            
            robot_description_path = os.path.join(script_dir, "../motion_policy_configs/ur3e/rmpflow/ur3e_gripper_robot_description.yaml"),
            urdf_path =  os.path.join(script_dir, "../motion_policy_configs/ur3e/ur3e_gripper.urdf"),
            rmpflow_config_path = os.path.join(script_dir, "../motion_policy_configs/ur3e/rmpflow/ur3e_gripper_rmpflow_config.yaml"),
            end_effector_frame_name = "gripper_center",
            maximum_substep_size = 0.00334
        )
articulation_rmpflow_2 = ArticulationMotionPolicy(robot_2, rmpflow_2, physics_dt)
articulation_controller_2 = robot_2.get_articulation_controller()

# Make a target to follow
target_cube = cuboid.VisualCuboid(
    "/World/target", position=np.array([0.5, 0, 0.5]), color=np.array([1.0, 0, 0]), size=0.1, scale=np.array([0.5, 0.5, 0.5])
)

# Make an obstacle to avoid
ground = cuboid.VisualCuboid(
    "/World/ground", position=np.array([0.0, 0, -0.0525]), color=np.array([0, 1.0, 0]), size=0.1, scale=np.array([40, 40, 1])
)

# Pre-reset world
my_world.reset()

# IP address of the first robot
try:
    rtde_r = rtde_receive.RTDEReceiveInterface(arg.robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(arg.robot_ip, 500.0, rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP, 50002)
    # robot.set_joint_positions(np.array(rtde_r.getActualQ()))

except:
    print("[ERROR] First robot is not connected")
    simulation_app.close()
    sys.exit()

# IP address of the second robot
try:
    rtde_r_2 = rtde_receive.RTDEReceiveInterface(arg_2.robot_ip2)
    rtde_c_2 = rtde_control.RTDEControlInterface(arg_2.robot_ip2, 500.0, rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP, 50007)
    # robot_2.set_joint_positions(np.array(rtde_r_2.getActualQ()))

except:
    print("[ERROR] Second robot is not connected")
    simulation_app.close()
    sys.exit()

while simulation_app.is_running():
    # On step render
    my_world.step(render=True)
    if my_world.is_playing():
        # First frame -> reset world
        if my_world.current_time_step_index == 0:
            my_world.reset()

        # Query the current obstacle position
        rmpflow.update_world()
        rmpflow_2.update_world()

        # Get actual joint positions from robots and update Isaac model
        actual_q, gripper_position =  ur3e_gripper_instance.extract_position()
        actual_q_2, gripper_position_2 =  ur3e_gripper_instance_2.extract_position()

        robot.set_joint_positions(np.array(list(actual_q)+[gripper_position,gripper_position]))
        robot_2.set_joint_positions(np.array(list(actual_q_2)+[gripper_position_2,gripper_position_2]))

# RTDE control stop script and disconnect
rtde_c.servoStop()
rtde_c.stopScript()
rtde_r.disconnect()

rtde_c_2.servoStop()
rtde_c_2.stopScript()
rtde_r_2.disconnect()

# Close Isaac Sim
simulation_app.close()