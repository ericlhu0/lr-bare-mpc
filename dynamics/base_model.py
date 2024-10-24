"""Base class for repositioning dynamics model."""

import abc

import numpy as np
import pybullet as p
from pybullet_helpers.robots.single_arm import SingleArmPyBulletRobot

from structs import JointTorques, RepositioningState


class RepositioningDynamicsModel(abc.ABC):
    """A model of forward dynamics."""

    def __init__(
        self,
        active_arm: SingleArmPyBulletRobot,
        passive_arm: SingleArmPyBulletRobot,
        dt: float,
    ) -> None:
        self.active_arm = active_arm
        self.passive_arm = passive_arm
        self.dt = dt

        # Need to disable default velocity control to use torque control.
        for robot in [active_arm, passive_arm]:
            p.setJointMotorControlArray(
                robot.robot_id,
                robot.arm_joints,
                p.VELOCITY_CONTROL,
                forces=np.zeros(len(robot.arm_joints)),
                physicsClientId=robot.physics_client_id,
            )
            for joint in robot.arm_joints:
                # Turn on torque sensor.
                p.enableJointForceTorqueSensor(
                    robot.robot_id, joint, 1, physicsClientId=robot.physics_client_id
                )

                # Remove any joint friction.
                p.changeDynamics(
                    robot.robot_id,
                    joint,
                    jointDamping=0.0,
                    anisotropicFriction=0.0,
                    maxJointVelocity=5000,
                    linearDamping=0.0,
                    angularDamping=0.0,
                    lateralFriction=0.0,
                    spinningFriction=0.0,
                    rollingFriction=0.0,
                    contactStiffness=0.0,
                    contactDamping=0.0,
                    physicsClientId=robot.physics_client_id,
                )

                # Disable any possible collisions.
                p.setCollisionFilterGroupMask(
                    robot.robot_id, joint, 0, 0, physicsClientId=robot.physics_client_id
                )

        # Let the simulation settle.
        for _ in range(1000):
            p.stepSimulation(physicsClientId=robot.physics_client_id)

    @abc.abstractmethod
    def reset(self, state: RepositioningState) -> None:
        """Reset the model state."""

    @abc.abstractmethod
    def get_state(self) -> RepositioningState:
        """Get the model state."""

    @abc.abstractmethod
    def step(self, torque: JointTorques) -> None:
        """Apply torque to the active arm and update in-place."""
