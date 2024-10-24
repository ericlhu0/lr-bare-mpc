"""A math repositioning dynamics model."""

import numpy as np
import pybullet as p
from numpy.typing import NDArray
from pybullet_helpers.geometry import matrix_from_quat
from pybullet_helpers.joint import JointPositions, JointVelocities
from pybullet_helpers.robots.single_arm import SingleArmPyBulletRobot

from structs import JointTorques, RepositioningState

from .base_model import RepositioningDynamicsModel


class MathRepositioningDynamicsModel(RepositioningDynamicsModel):

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._active_to_passive_ee_twist = self._get_active_to_passive_ee_twist(
            self.active_arm, self.passive_arm
        )
        active_positions = self.active_arm.get_joint_positions()
        active_velocities = self.active_arm.get_joint_velocities()
        passive_positions = self.passive_arm.get_joint_positions()
        passive_velocities = self.passive_arm.get_joint_velocities()
        self._current_state = RepositioningState(
            active_positions, active_velocities, passive_positions, passive_velocities
        )

    def reset(self, state: RepositioningState) -> None:
        self._current_state = state

    def get_state(self) -> RepositioningState:
        return self._current_state

    def step(self, torque: JointTorques) -> None:
        pos_r = self._current_state.active_positions
        pos_h = self._current_state.passive_positions
        vel_r = self._current_state.active_velocities
        vel_h = self._current_state.passive_velocities
        R = self._active_to_passive_ee_twist

        Jr = self._calculate_jacobian(self.active_arm, pos_r)
        Jh = self._calculate_jacobian(self.passive_arm, pos_h)
        Jhinv = np.linalg.pinv(Jh)

        Mr = self._calculate_mass_matrix(self.active_arm, pos_r)
        Mh = self._calculate_mass_matrix(self.passive_arm, pos_h)

        Nr = self._calculate_N_vector(self.active_arm, pos_r, vel_r)
        Nh = self._calculate_N_vector(self.passive_arm, pos_h, vel_h)

        acc_r = np.linalg.pinv((Jhinv @ R @ -Jr).T @ Mh @ (Jhinv @ R @ Jr) - Mr) @ (
            (Jhinv @ R @ Jr).T
            @ (
                Mh * (1 / self.dt) @ (Jhinv @ R @ Jr) @ vel_r
                - Mh * (1 / self.dt) @ vel_h
                + Nh
            )
            + Nr
            - np.array(torque)
        )

        new_vel_r = vel_r + acc_r * self.dt
        r_lin_vel = Jr @ new_vel_r
        h_lin_vel = R @ r_lin_vel
        new_vel_h = Jhinv @ h_lin_vel

        acc_h = (new_vel_h - vel_h) / self.dt

        vel_r_vec = vel_r + acc_r * self.dt
        vel_h_vec = vel_h + acc_h * self.dt

        pos_r_vec = pos_r + vel_r_vec * self.dt
        pos_h_vec = pos_h + vel_h_vec * self.dt

        self._current_state = RepositioningState(
            list(pos_r_vec), list(vel_r_vec), list(pos_h_vec), list(vel_h_vec)
        )

    @staticmethod
    def _calculate_jacobian(
        robot: SingleArmPyBulletRobot, joint_positions: JointPositions
    ) -> NDArray:
        jac_t, jac_r = p.calculateJacobian(
            robot.robot_id,
            robot.tool_link_id,
            [0, 0, 0],
            joint_positions,
            [0.0] * len(joint_positions),
            [0.0] * len(joint_positions),
            physicsClientId=robot.physics_client_id,
        )
        return np.concatenate((np.array(jac_t), np.array(jac_r)), axis=0)

    @staticmethod
    def _calculate_mass_matrix(
        robot: SingleArmPyBulletRobot, joint_positions: JointPositions
    ) -> NDArray:
        mass_matrix = p.calculateMassMatrix(
            robot.robot_id,
            joint_positions,
            physicsClientId=robot.physics_client_id,
        )
        return np.array(mass_matrix)

    @staticmethod
    def _calculate_N_vector(
        robot: SingleArmPyBulletRobot,
        joint_positions: JointPositions,
        joint_velocities: JointVelocities,
    ) -> NDArray:
        joint_accel = [0.0] * len(joint_positions)
        n_vector = p.calculateInverseDynamics(
            robot.robot_id,
            joint_positions,
            joint_velocities,
            joint_accel,
            physicsClientId=robot.physics_client_id,
        )
        return np.array(n_vector)

    @staticmethod
    def _get_active_to_passive_ee_twist(
        active_arm: SingleArmPyBulletRobot, passive_arm: SingleArmPyBulletRobot
    ) -> NDArray:
        active_ee_orn = active_arm._base_pose.orientation
        passive_ee_orn = passive_arm._base_pose.orientation
        active_to_passive_ee = matrix_from_quat(passive_ee_orn).T @ matrix_from_quat(
            active_ee_orn
        )
        active_to_passive_ee_twist = np.eye(6)
        active_to_passive_ee_twist[:3, :3] = active_to_passive_ee
        active_to_passive_ee_twist[3:, 3:] = active_to_passive_ee
        return active_to_passive_ee_twist
