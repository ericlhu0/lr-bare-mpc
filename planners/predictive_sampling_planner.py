"""A planner that uses predictive sampling."""

import numpy as np
from pybullet_helpers.joint import (JointPositions, get_joint_infos,
                                    get_jointwise_difference)
from pybullet_helpers.trajectory import (Trajectory, TrajectorySegment,
                                         concatenate_trajectories)

from structs import JointTorques, RepositioningGoal, RepositioningState

from .base_planner import RepositioningPlanner


class PredictiveSamplingPlanner(RepositioningPlanner):
    """A planner that uses predictive sampling."""

    def __init__(
        self,
        num_rollouts: int = 10,
        noise_scale: float = 1.0,
        num_control_points: int = 10,
        replan_dt: float = 1e-2,
        *args,
        **kwargs
    ) -> None:
        super().__init__(*args, **kwargs)
        self._num_rollouts = num_rollouts
        self._noise_scale = noise_scale
        self._num_control_points = num_control_points
        self._replan_dt = replan_dt
        self._num_active_dof = len(self._active_arm.arm_joints)
        self._passive_joint_infos = get_joint_infos(
            self._passive_arm.robot_id,
            self._passive_arm.arm_joints,
            self._passive_arm.physics_client_id,
        )
        self._current_plan: Trajectory[JointTorques] | None = None
        self._current_goal: JointPositions | None = None

    def reset(
        self,
        initial_state: RepositioningState,
        goal: RepositioningGoal,
    ) -> None:
        self._current_plan = self._get_initialization()
        self._current_goal = goal

    def step(self, state: RepositioningState) -> JointTorques:
        # Warm start by advancing the last solution by one step.
        nominal = self._current_plan.get_sub_trajectory(
            self._dt, self._current_plan.duration
        )
        # Check if it's time to replan.
        if (nominal.duration // self._replan_dt) == (
            (nominal.duration + self._dt) // self._replan_dt
        ):
            self._current_plan = nominal
        else:
            sample_list: list[Trajectory[JointTorques]] = [nominal]
            # Sample new candidates around the nominal trajectory.
            num_samples = self._num_rollouts - len(sample_list)
            new_samples = self._sample_from_nominal(nominal, num_samples)
            sample_list.extend(new_samples)
            # Pick the best one.
            self._current_plan = min(
                sample_list, key=lambda s: self._score_trajectory(s, state)
            )
        # Return just the first action.
        return self._current_plan[0.0]

    def _get_initialization(self) -> Trajectory[JointTorques]:
        control_points = self._rng.normal(
            size=(self._num_control_points, self._num_active_dof),
            scale=self._noise_scale,
        ).tolist()
        clipped_control_points = self._clip_control_points(control_points)
        control_dt = self._T / (len(control_points) - 1)
        return self._point_sequence_to_trajectory(clipped_control_points, control_dt)

    def _clip_control_points(
        self, control_points: list[JointTorques]
    ) -> list[JointTorques]:
        low = self._scene_config.torque_lower_limits
        high = self._scene_config.torque_upper_limits
        return [
            np.clip(a, low, high).astype(np.float64).tolist() for a in control_points
        ]

    def _sample_from_nominal(
        self, nominal: Trajectory[JointTorques], num_samples: int
    ) -> list[Trajectory[JointTorques]]:
        # Sample by adding Gaussian noise around the nominal trajectory.
        control_times = np.linspace(
            0,
            nominal.duration,
            num=self._num_control_points,
            endpoint=True,
        )
        nominal_control_points = np.array([nominal(t) for t in control_times])
        noise_shape = (
            num_samples,
            len(control_times),
            self._num_active_dof,
        )
        noise = self._rng.normal(loc=0, scale=self._noise_scale, size=noise_shape)
        new_control_points = nominal_control_points + noise
        # Clip to obey bounds.
        clipped_control_points = [
            self._clip_control_points(sample) for sample in new_control_points
        ]
        # Convert to trajectories.
        dt = control_times[1] - control_times[0]
        return [
            self._point_sequence_to_trajectory(actions, dt)
            for actions in clipped_control_points
        ]

    def _score_trajectory(
        self, trajectory: Trajectory[JointTorques], current_state: RepositioningState
    ) -> float:
        # Currently scoring based on distance at every state to goal.
        self._dynamics.reset(current_state)
        score = 0.0
        for t in np.arange(0.0, trajectory.duration + self._dt, self._dt):
            torque = trajectory[t]
            self._dynamics.step(torque)
            state = self._dynamics.get_state()
            score += self._get_distance_to_goal(state)
        return score

    def _get_distance_to_goal(self, state: RepositioningState) -> float:
        diff = get_jointwise_difference(
            self._passive_joint_infos, self._current_goal, state.passive_positions
        )
        return np.sum(np.square(diff))

    def _point_sequence_to_trajectory(
        self, point_sequence: list[JointTorques], dt: float
    ) -> Trajectory[JointTorques]:

        def _interpolate_fn(
            p1: JointTorques, p2: JointTorques, t: float
        ) -> JointTorques:
            dists_arr = np.subtract(p2, p1)
            return list(np.add(p1, t * dists_arr))

        def _distance_fn(p1: JointTorques, p2: JointTorques) -> float:
            return np.sum(np.subtract(p2, p1) ** 2)

        segments = []
        for t in range(len(point_sequence) - 1):
            start = point_sequence[t]
            end = point_sequence[t + 1]
            assert dt is not None and dt > 0
            seg = TrajectorySegment(start, end, dt, _interpolate_fn, _distance_fn)
            segments.append(seg)
        return concatenate_trajectories(segments)
