"""A model where PyBullet sets a constraint."""

import pybullet as p
from pybullet_helpers.geometry import multiply_poses

from structs import JointTorques, RepositioningState

from .base_model import RepositioningDynamicsModel


class PybulletConstraintRepositioningDynamicsModel(RepositioningDynamicsModel):

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        # Create constraint.
        tf = multiply_poses(
            self.passive_arm.get_end_effector_pose().invert(),
            self.active_arm.get_end_effector_pose(),
        )
        p.createConstraint(
            self.active_arm.robot_id,
            self.active_arm.end_effector_id,
            self.passive_arm.robot_id,
            self.passive_arm.end_effector_id,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=tf.position,
            parentFrameOrientation=[0, 0, 0, 1],
            childFrameOrientation=tf.orientation,
            physicsClientId=self.active_arm.physics_client_id,
        )

    def reset(self, state: RepositioningState) -> None:
        self.active_arm.set_joints(state.active_positions, state.active_velocities)
        self.passive_arm.set_joints(state.passive_positions, state.passive_velocities)

    def get_state(self) -> RepositioningState:
        active_positions = self.active_arm.get_joint_positions()
        active_velocities = self.active_arm.get_joint_velocities()
        passive_positions = self.passive_arm.get_joint_positions()
        passive_velocities = self.passive_arm.get_joint_velocities()
        return RepositioningState(
            active_positions, active_velocities, passive_positions, passive_velocities
        )

    def step(self, torque: JointTorques) -> None:
        # TODO: move this into pybullet helpers.
        p.setJointMotorControlArray(
            self.active_arm.robot_id,
            self.active_arm.arm_joints,
            p.TORQUE_CONTROL,
            forces=torque,
            physicsClientId=self.active_arm.physics_client_id,
        )
        t = 0.0
        pybullet_dt = 1 / 240  # default pybullet dt
        while t <= self.dt:
            p.stepSimulation(physicsClientId=self.active_arm.physics_client_id)
            t += pybullet_dt
