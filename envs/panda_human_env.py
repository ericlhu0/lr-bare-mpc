"""Repositioning environment where a panda is repositioning a human arm."""

import numpy as np
from pybullet_helpers.geometry import Pose
from scipy.spatial.transform import Rotation

from structs import RepositioningSceneConfig

from .dynamics_model_env import DynamicsModelEnv


class PandaHumanRepositioningEnv(DynamicsModelEnv):
    """A panda is repositioning a human arm."""

    def _get_default_scene_config(self) -> RepositioningSceneConfig:

        active_name = "panda-limb-repo"
        passive_name = "human-arm-6dof"

        robot_init_pos = (0.8, -0.1, 0.5)
        human_init_pos = (0.15, 0.1, 1.4)

        robot_init_orn_obj = Rotation.from_euler("xyz", [0, 0, np.pi])
        robot_base_pose = Pose(robot_init_pos, robot_init_orn_obj.as_quat())
        human_init_orn_obj = Rotation.from_euler("xyz", [np.pi, 0, 0])
        human_base_pose = Pose(human_init_pos, human_init_orn_obj.as_quat())

        panda_init_joint_positions = [
            0.94578431,
            -0.89487842,
            -1.67534487,
            -0.34826698,
            1.73607292,
            0.14233887,
        ]
        human_init_joint_positions = [
            1.43252278,
            -0.81111486,
            -0.42373363,
            0.49931369,
            -1.17420521,
            0.37122887,
        ]
        panda_init_joint_velocities = [0.0] * len(panda_init_joint_positions)
        human_init_joint_velocities = [0.0] * len(human_init_joint_positions)

        human_goal_joint_positions = [
            0.5,
            -0.8,
            -0.1,
            0.0,
            0.2,
            0.0,
        ]

        torque_lower_limits = [-10.0] * 6
        torque_upper_limits = [10.0] * 6

        camera_target = human_init_pos
        camera_distance = 1.75
        camera_pitch = -50

        image_height = 512
        image_width = 512

        return RepositioningSceneConfig(
            active_name=active_name,
            passive_name=passive_name,
            active_base_pose=robot_base_pose,
            passive_base_pose=human_base_pose,
            active_init_joint_positions=panda_init_joint_positions,
            active_init_joint_velocities=panda_init_joint_velocities,
            passive_init_joint_positions=human_init_joint_positions,
            passive_init_joint_velocities=human_init_joint_velocities,
            passive_goal_joint_positions=human_goal_joint_positions,
            torque_lower_limits=torque_lower_limits,
            torque_upper_limits=torque_upper_limits,
            camera_target=camera_target,
            camera_distance=camera_distance,
            camera_pitch=camera_pitch,
            image_height=image_height,
            image_width=image_width,
        )
