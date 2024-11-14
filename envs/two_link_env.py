"""Environment where a two-link arm is repositioning another."""

import numpy as np
from pybullet_helpers.geometry import Pose

from structs import RepositioningSceneConfig

from .dynamics_model_env import DynamicsModelEnv


class TwoLinkRepositioningEnv(DynamicsModelEnv):
    """Environment where a two-link arm is repositioning another."""

    active_name = "two-link"
    passive_name = "two-link"

    pad = 0.01  # add pad to prevent contact forces
    active_base_pose = Pose((-np.sqrt(2) - pad, 0.0, 0.0))
    passive_base_pose = Pose((np.sqrt(2) + pad, 0.0, 0.0))

    active_init_joint_positions = [-np.pi / 4, np.pi / 2]
    active_init_joint_velocities = [0.0, 0.0]
    passive_init_joint_positions = [np.pi / 2 + np.pi / 4, np.pi / 2]
    passive_init_joint_velocities = [0.0, 0.0]

    # TODO actually set these
    passive_goal_joint_positions = [np.pi / 2 + np.pi / 4, np.pi / 2]

    torque_lower_limits = [-10.0] * 2
    torque_upper_limits = [10.0] * 2

    camera_target = (0.0, 0.0, 0.0)
    camera_distance = 2.0
    camera_pitch = -60

    image_height = 512
    image_width = 512

    def _get_default_scene_config(self) -> RepositioningSceneConfig:

        return RepositioningSceneConfig(
            active_name=self.active_name,
            passive_name=self.passive_name,
            active_base_pose=self.robot_base_pose,
            passive_base_pose=self.human_base_pose,
            active_init_joint_positions=self.panda_init_joint_positions,
            active_init_joint_velocities=self.panda_init_joint_velocities,
            passive_init_joint_positions=self.human_init_joint_positions,
            passive_init_joint_velocities=self.human_init_joint_velocities,
            passive_goal_joint_positions=self.human_goal_joint_positions,
            torque_lower_limits=self.torque_lower_limits,
            torque_upper_limits=self.torque_upper_limits,
            camera_target=self.camera_target,
            camera_distance=self.camera_distance,
            camera_pitch=self.camera_pitch,
            image_height=self.image_height,
            image_width=self.image_width,
        )

    def set_active_init_joint_positions(self, joint_positions: list[float]):
        self._scene_config.active_init_joint_positions = joint_positions

    def set_passive_init_joint_positions(self, joint_positions: list[float]):
        self._scene_config.passive_init_joint_positions = joint_positions
