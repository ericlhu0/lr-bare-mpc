"""Experimental script for one robot moving another."""

import time
import pybullet as p
import random
import pybullet_data
from pybullet_helpers.camera import capture_image
import imageio.v2 as iio
import os
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from pathlib import Path

class RobotSim:
    def __init__(self, control_timestep, init_robot_config: list[float]):
        self.sim_steps = int(control_timestep / (1/240))
        self.sim_id = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,0.0)
        ground_plane_id = p.loadURDF("plane.urdf")
        p.setRealTimeSimulation(0)
        self.urdf_path = str(Path(__file__).parent / "robots" / "urdf" / "panda_limb_repo" / "panda_limb_repo_with_limits.urdf")
        
        base_pose = [0, 0, 0]
        base_orientation = p.getQuaternionFromEuler([0, 0, -np.pi/4])
        
        self.robot_id = p.loadURDF(self.urdf_path, base_pose, base_orientation, useFixedBase=True, flags = p.URDF_USE_INERTIA_FROM_FILE)

        self.joint_ids = []
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[2] != p.JOINT_FIXED:
                self.joint_ids.append(i)
        
        for i, joint_id in enumerate(self.joint_ids):
            p.resetJointState(self.robot_id, joint_id, init_robot_config[i])

        for i in range(p.getNumJoints(self.robot_id)):
            p.changeDynamics(self.robot_id, i, jointDamping=0.0, anisotropicFriction=0.0, maxJointVelocity=5000, linearDamping=0.0, angularDamping=0.0, lateralFriction=0.0, spinningFriction=0.0, rollingFriction=0.0, contactStiffness=0.0, contactDamping=0.0), #, jointLowerLimit=-6.283185 * 200, jointUpperLimit=6.283185 * 200)

        # --- apply velocity control to panda arm to make it stationary --- #
        for joint_id in self.joint_ids:
            p.setJointMotorControl2(self.robot_id, joint_id, p.VELOCITY_CONTROL, targetVelocity=0, force=50)

        for i in range(1000):
            p.stepSimulation()

        # --- try to force panda arm to stop moving by setting forces to zero to k timesteps --- #
        for joint_id in self.joint_ids:
            p.setJointMotorControl2(self.robot_id, joint_id, p.VELOCITY_CONTROL, force=0)
            p.setJointMotorControl2(self.robot_id, joint_id, p.TORQUE_CONTROL, force=0)
 
        for i in range(1000):
            p.stepSimulation()

    def apply_torque(self, tau_robot: list[float]):

        # --- in order to use torque control, velocity control must be disabled at every time step --- #
        for joint_id in self.joint_ids:
            p.setJointMotorControl2(self.robot_id, joint_id, p.VELOCITY_CONTROL, force=0)

        # --- apply torque control to panda arm --- #
        p.setJointMotorControlArray(self.robot_id, self.joint_ids, p.TORQUE_CONTROL, forces=tau_robot)
        
        for i in range(self.sim_steps):
            p.stepSimulation()
    
    def get_state(self):
        
        frame = capture_image(self.sim_id, camera_distance=1.3, image_width=1024, image_height=1024)

        state = []
        for joint_id in self.joint_ids:
            state.append(p.getJointState(self.robot_id, joint_id)[0]) # joint position

        return state, frame

def wait_for_init_config() -> list[float]:
    """Wait for initial robot config from ROS <topic>."""
    rospy.loginfo("Waiting for a message on '/franka_motion_control/joint_states'...")
    received_msg: JointState = rospy.wait_for_message(
        "/franka_motion_control/joint_states", JointState
    )
    print(received_msg.position)
    return received_msg.position

def main():

    rospy.init_node("mpc_controller")
    init_robot_config: list[float] = wait_for_init_config()

    print("init_robot_config: ", init_robot_config)

    # Initialize the ROS publisher node
    pub = rospy.Publisher("/lr_torque_command/commands", JointState, queue_size=1)
    # wait for the publisher to be ready
    time.sleep(1)

    # Sample random torque commands
    random.seed(0)
    torques = [[random.uniform(-1,1) for _ in range(7)]]*50
    robot_sim = RobotSim(0.1, init_robot_config)

    states, frames = [], []
    for tau_robot in torques:
        robot_sim.apply_torque(tau_robot)
        state, frame = robot_sim.get_state()
        states.append(state)
        frames.append(frame)
    
    # create video from frames
    video_dir = Path(__file__).parent / "videos"
    os.makedirs(video_dir, exist_ok=True)
    video_outfile = video_dir / "robot_sim.mp4"
    video_fps = 10
    iio.mimsave(video_outfile, frames, fps=video_fps)
    print(f"Wrote out to {video_outfile}")

    input("Press Enter to run in real world")
    # Run in real world
    for tau_robot in torques:
        torque_command = JointState()
        torque_command.header.stamp = rospy.Time.now()
        torque_command.effort = tau_robot
        pub.publish(torque_command)
        time.sleep(0.1)

if __name__ == "__main__":
    main()