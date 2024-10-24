"""Helper function to create dynamics models."""

import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose
from pybullet_helpers.joint import JointPositions
from pybullet_helpers.robots import create_pybullet_robot
from pybullet_helpers.robots.single_arm import SingleArmPyBulletRobot

from robots.human import HumanArm6DoF
from robots.panda import PandaPybulletRobotLimbRepo
from structs import RepositioningSceneConfig

from .base_model import RepositioningDynamicsModel
from .math_model import MathRepositioningDynamicsModel
from .pybullet_constraint_model import \
    PybulletConstraintRepositioningDynamicsModel


def create_robot(
    name: str, physics_client_id: int, base_pose: Pose, joint_positions: JointPositions
) -> SingleArmPyBulletRobot:
    """Create a robot from its name."""
    if name == "human-arm-6dof":
        return HumanArm6DoF(
            physics_client_id, base_pose=base_pose, home_joint_positions=joint_positions
        )

    if name == "panda-limb-repo":
        return PandaPybulletRobotLimbRepo(
            physics_client_id, base_pose=base_pose, home_joint_positions=joint_positions
        )

    return create_pybullet_robot(
        name,
        physics_client_id,
        base_pose=base_pose,
        home_joint_positions=joint_positions,
    )


def create_dynamics_model(
    name: str,
    physics_client_id: int,
    scene_config: RepositioningSceneConfig,
    dt: float,
) -> RepositioningDynamicsModel:
    """Helper function to create dynamics models."""

    assert np.allclose(scene_config.active_init_joint_velocities, 0.0)
    assert np.allclose(scene_config.passive_init_joint_velocities, 0.0)

    active_arm = create_robot(
        scene_config.active_name,
        physics_client_id,
        scene_config.active_base_pose,
        scene_config.active_init_joint_positions,
    )
    passive_arm = create_robot(
        scene_config.passive_name,
        physics_client_id,
        scene_config.passive_base_pose,
        scene_config.passive_init_joint_positions,
    )

    if name == "math":
        return MathRepositioningDynamicsModel(active_arm, passive_arm, dt)

    if name == "pybullet-constraint":
        return PybulletConstraintRepositioningDynamicsModel(active_arm, passive_arm, dt)

    raise NotImplementedError
