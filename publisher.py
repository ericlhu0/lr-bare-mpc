"""Experimental script for one robot moving another."""

import os
import time
from pathlib import Path

import imageio.v2 as iio
import pybullet as p
import rospy
from sensor_msgs.msg import JointState
from tqdm import tqdm

from dynamics import create_dynamics_model, create_robot
from envs import create_env
from planners import PredictiveSamplingPlanner, create_planner


def wait_for_init_config() -> list[float]:
    """Wait for initial robot config from ROS <topic>."""
    rospy.loginfo("Waiting for a message on '/franka_motion_control/joint_states'...")
    received_msg: JointState = rospy.wait_for_message(
        "/franka_motion_control/joint_states", JointState
    )
    print(received_msg.position)
    print(received_msg.position[:2] + received_msg.position[3:])
    return received_msg.position[:2] + received_msg.position[3:]


def _main(
    env_name: str,
    sim_dynamics_name: str,
    real_dynamics_name: str,
    planner_name: str,
    T: float,  # horizon
    dt: float,
    use_gui: bool,
    make_video: bool,
    video_fps_scale: float,
    seed: int = 0,
    render_interval: int = 10,
    num_rollouts: int = 10,
) -> None:

    video_dir = Path(__file__).parent / "videos"
    os.makedirs(video_dir, exist_ok=True)

    rospy.init_node("mpc_controller")
    init_robot_config: list[float] = wait_for_init_config()

    env = create_env(
        env_name, real_dynamics_name, dt, use_gui, panda_init_joint_positions=init_robot_config
    )

    # input(f"initial setup, {env.get_state()}")

    scene_config = env.get_scene_config()
    sim_physics_client_id = p.connect(p.DIRECT)
    dynamics_model = create_dynamics_model(
        sim_dynamics_name,
        sim_physics_client_id,
        scene_config,
        dt,
    )
    planner: PredictiveSamplingPlanner = create_planner(
        planner_name,
        scene_config=scene_config,
        dynamics=dynamics_model,
        T=T,
        dt=dt,
        seed=seed,
        num_rollouts=num_rollouts,
    )

    init_state = env.get_state()
    goal_state = env.get_goal()

    # input(
    #     f"initial setup finished. Press Enter to start the control loop.,  {env.get_state()}"
    # )

    planner.reset(init_state, goal_state)

    if make_video:
        imgs = [env.render()]

    # Initialize the ROS publisher node
    pub = rospy.Publisher("/lr_torque_command/commands", JointState, queue_size=1)

    i = 0
    while not rospy.is_shutdown():
        tstart = time.time()
        i += 1
        state = env.get_state()
        t1 = time.time()
        action = planner.step_fixed_horizon(state)

        # Publish the torque command
        torque_command = JointState()
        torque_command.header.stamp = rospy.Time.now()
        torque_command.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print("torque_command: ", torque_command)
        pub.publish(torque_command)

        t2 = time.time()
        env.step(action)
        tend = time.time()
        elapsed = tend - tstart
        print(f"planning time: {t2 - t1}")
        print(f"frequency:     {1/elapsed} Hz")
        print("next step")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--env", type=str, default="panda-human")
    parser.add_argument("--sim_dynamics", type=str, default="pybullet-constraint")
    parser.add_argument("--real_dynamics", type=str, default="pybullet-constraint")
    parser.add_argument("--planner", type=str, default="predictive-sampling")
    parser.add_argument("--T", type=float, default=1)  # horizon
    parser.add_argument("--dt", type=float, default=1 / 3)
    parser.add_argument("--num_rollouts", type=int, default=25)
    parser.add_argument("--use_gui", action="store_true")
    parser.add_argument("--make_video", action="store_true")
    parser.add_argument("--video_fps_scale", type=float, default=0.025)
    parser.add_argument("--seed", type=int, default=0)

    args = parser.parse_args()

    _main(
        args.env,
        args.sim_dynamics,
        args.real_dynamics,
        args.planner,
        args.T,
        args.dt,
        args.use_gui,
        args.make_video,
        args.video_fps_scale,
        seed=args.seed,
        num_rollouts=args.num_rollouts,
    )
