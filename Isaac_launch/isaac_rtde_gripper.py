# Isaac Sim app library
from omni.isaac.kit import SimulationApp
from Gripper_control import UR3eGripper
# from Gripper_control import extract_data
simulation_app = SimulationApp({"headless": False})

# Isaac Sim extenstions + core libraries
from omni.isaac.motion_generation.lula import RmpFlow
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import cuboid
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.motion_generation.interface_config_loader import (
    load_supported_motion_policy_config,
)

# ur rtde communication
import rtde_control
import rtde_receive

import numpy as np
import argparse
import sys

import argparse
import logging
import sys

import time
import os
script_dir = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser()
parser.add_argument(
    "--host",
    type=str,
    default="192.168.10.23",
    help="IP adress of robot Real world UR Polyscope or VM UR Polyscope",
)

arg = parser.parse_args()
rtde_receive_interface = rtde_receive.RTDEReceiveInterface(arg.host)

ur3e_gripper_instance = UR3eGripper(arg.host,rtde_receive_interface,30004)

# set up paths and prims
robot_name = "UR3e"
prim_path = "/UR3e"
usd_path = os.path.join(script_dir, "../usd/flatten.usd")

# set references to staget in isaac
add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

# add world
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# add robot to world
robot = my_world.scene.add(Robot(prim_path=prim_path, name=robot_name))

# loading rmp_config
rmp_config = load_supported_motion_policy_config(robot_name, "RMPflow")

# Initialize an RmpFlow object and set up
rmpflow = RmpFlow(            
            robot_description_path = os.path.join(script_dir, "../motion_policy_configs/ur3e/rmpflow/ur3e_gripper_robot_description.yaml"),
            urdf_path =  os.path.join(script_dir, "../motion_policy_configs/ur3e/ur3e_gripper.urdf"),
            rmpflow_config_path = os.path.join(script_dir, "../motion_policy_configs/ur3e/rmpflow/ur3e_gripper_rmpflow_config.yaml"),
            end_effector_frame_name = "gripper_center",
            maximum_substep_size = 0.00334
        )
physics_dt = 1.0/60
articulation_rmpflow = ArticulationMotionPolicy(robot, rmpflow, physics_dt)     
articulation_controller = robot.get_articulation_controller()

# Make a target to follow
target_cube = cuboid.VisualCuboid(
    "/World/target", position=np.array([0.09, 0.22, 0.7]), color=np.array([1.0, 0, 0]), size=0.1, scale=np.array([0.5,0.5,0.5])
)

# Make an obstacle to avoid
ground = cuboid.VisualCuboid(
    "/World/ground", position=np.array([0.0, 0, -0.0525]), color=np.array([0, 1.0, 0]), size=0.1, scale=np.array([40,40,1])
)
# rmpflow.visualize_end_effector_position()
# rmpflow.visualize_collision_spheres()
rmpflow.add_obstacle(ground)

# prereset world
my_world.reset()


try:
    rtde_r = rtde_receive.RTDEReceiveInterface(arg.host)
    rtde_c = rtde_control.RTDEControlInterface(arg.host,500.0,rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)

except:
    print("[ERROR] Robot is not connected")
    # close isaac sim
    simulation_app.close()
    sys.exit()

ur3e_gripper_instance.open()
while simulation_app.is_running():
    # on step render
    my_world.step(render=True)
    if my_world.is_playing():
        # first frame -> reset world
        if my_world.current_time_step_index == 0:
            my_world.reset()

        # set target to RMP Flow
        rmpflow.set_end_effector_target(
            target_position=target_cube.get_world_pose()[0], target_orientation=target_cube.get_world_pose()[1]
        )
        
        # Parameters
        velocity = 1
        acceleration = 1
        dt = 0.1/500  # 2ms
        lookahead_time = 0.1
        gain = 300
        end_effector_pose=rmpflow.get_end_effector_pose(robot.get_joint_positions(joint_indices=np.array([0,1,2,3,4,5])))
        # print(end_effector_pose)
        rounded_translation = np.round(end_effector_pose[0], 2)
        target_translation = np.array([0.09, 0.22, 0.7])
        # print(rounded_translation,target_translation)
        offset_error = np.array([0.01, 0.01, 0.01])
        translation_with_offset = rounded_translation + offset_error
        position_threshold = 0.01  # Adjust the threshold as needed

        if np.linalg.norm(translation_with_offset - target_translation) < position_threshold:
            ur3e_gripper_instance.close()

        joint_q = robot.get_joint_positions(joint_indices=np.array([0,1,2,3,4,5]))

        # time start period
        t_start = rtde_c.initPeriod()

        # run servoJ
        rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
        rtde_c.waitPeriod(t_start)

        rmpflow.update_world()

        actions = articulation_rmpflow.get_next_articulation_action()
        articulation_controller.apply_action(actions)
        actual_q, gripper_position =  ur3e_gripper_instance.extract_position()

        robot.set_joint_positions(np.array(list(actual_q)+[gripper_position,gripper_position]))

# rtde control stop script and disconnect
rtde_c.servoStop()
rtde_c.stopScript()
rtde_r.disconnect()

# close isaac sim
simulation_app.close()
