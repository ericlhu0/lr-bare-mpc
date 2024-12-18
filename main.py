"""Experimental script for one robot moving another."""

import os
import time
from pathlib import Path

import imageio.v2 as iio
import pybullet as p
from tqdm import tqdm

from dynamics import create_dynamics_model, create_robot
from envs import create_env
from planners import PredictiveSamplingPlanner, create_planner


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

    env = create_env(env_name, real_dynamics_name, dt, use_gui)
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
    planner.reset(init_state, goal_state)

    if make_video:
        imgs = [env.render()]

    i = 0

    while True:
        tstart = time.time()
        i += 1
        state = env.get_state()
        t1 = time.time()
        action = planner.step_fixed_horizon(state)
        t2 = time.time()
        env.step(action)
        tend = time.time()
        elapsed = tend - tstart
        print(f"planning time: {t2 - t1}")
        print(f"frequency:     {1/elapsed} Hz")

        if make_video and (i % render_interval == 0):
            imgs.append(env.render())

        print("next step")

    if make_video:
        video_outfile = (
            video_dir
            / f"{env_name}_{sim_dynamics_name}_{real_dynamics_name}_{planner_name}_{seed}.mp4"
        )
        video_fps = (1 / dt) * video_fps_scale
        iio.mimsave(video_outfile, imgs, fps=video_fps)
        print(f"Wrote out to {video_outfile}")


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
