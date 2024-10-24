"""A planner that uses RRT to find a joint position plan."""

import time
from typing import Iterator

import numpy as np
from pybullet_helpers.geometry import interpolate_poses, multiply_poses
from pybullet_helpers.inverse_kinematics import (InverseKinematicsError,
                                                 inverse_kinematics)
from pybullet_helpers.joint import (JointPositions, get_joint_infos,
                                    get_jointwise_difference)
from pybullet_helpers.trajectory import (Trajectory, TrajectorySegment,
                                         concatenate_trajectories)
from tomsutils.motion_planning import BiRRT

from structs import JointTorques, RepositioningGoal, RepositioningState

from .base_planner import RepositioningPlanner


class RRTPositionPlanner(RepositioningPlanner):
    """A planner that uses RRT to find a joint position plan.

    The joint position plan is then followed using a PD controlle (using
    only the active arm joint positions).
    """

    def __init__(
        self,
        birrt_extend_num_interp: int = 25,
        birrt_num_attempts: int = 10,
        birrt_num_iters: int = 100,
        birrt_smooth_amt: int = 50,
        *args,
        **kwargs
    ) -> None:
        super().__init__(*args, **kwargs)
        self._birrt_extend_num_interp = birrt_extend_num_interp
        self._birrt_num_attempts = birrt_num_attempts
        self._birrt_num_iters = birrt_num_iters
        self._birrt_smooth_amt = birrt_smooth_amt
        self._num_active_dof = len(self._active_arm.arm_joints)
        self._passive_joint_infos = get_joint_infos(
            self._passive_arm.robot_id,
            self._passive_arm.arm_joints,
            self._passive_arm.physics_client_id,
        )
        self._current_motion_plan: Trajectory[JointPositions] | None = None

    def reset(
        self,
        initial_state: RepositioningState,
        goal: RepositioningGoal,
    ) -> None:
        # Run motion planning once.
        self._current_motion_plan = self._run_motion_planning(initial_state, goal)

    def step(self, state: RepositioningState) -> JointTorques:
        # Advance the motion plan by one step
        self._current_motion_plan = self._current_motion_plan.get_sub_trajectory(
            self._dt, self._current_motion_plan.duration
        )
        # Try to follow the motion plan.
        import ipdb

        ipdb.set_trace()

    def _run_motion_planning(
        self, state: RepositioningState, goal: RepositioningGoal
    ) -> Trajectory[JointPositions]:
        # Get transform between active and passive end effectors.
        active_init_ee = self._active_arm.get_end_effector_pose()
        passive_init_ee = self._passive_arm.get_end_effector_pose()
        active_ee_to_passive_ee = multiply_poses(
            active_init_ee.invert(), passive_init_ee
        )
        # Get a goal in the robot joint space.
        passive_target_joints = goal
        passive_target_ee = self._passive_arm.forward_kinematics(passive_target_joints)
        active_target_ee = multiply_poses(
            passive_target_ee, active_ee_to_passive_ee.invert()
        )
        active_target_joints = inverse_kinematics(self._active_arm, active_target_ee)

        # The state for motion planning is the active joints concatenated with
        # the passive joints. But we will sample in the active joint space and
        # then run IK for the passive arm.
        init_joints = state.active_positions + state.passive_positions
        target_joints = active_target_joints + passive_target_ee
        joint_space = self._active_arm.action_space
        joint_space.seed(self._seed)

        def _sampling_fn(_: JointPositions) -> JointPositions:
            # Retry sampling until we find a feasible state that works for both
            # the active and passive arms.
            while True:
                new_active_joints: JointPositions = list(joint_space.sample())
                # Run FK to get new active joint position.
                new_active_ee = self._active_arm.forward_kinematics(new_active_joints)
                # Run IK to get a new passive joint position.
                new_passive_ee = multiply_poses(new_active_ee, active_ee_to_passive_ee)
                try:
                    new_passive_joints = inverse_kinematics(
                        self._passive_arm, new_passive_ee
                    )
                except InverseKinematicsError:
                    continue
                # Succeeded.
                return new_active_joints + new_passive_joints

        # Interpolate in end effector space.
        def _extend_fn(
            pt1: JointPositions, pt2: JointPositions
        ) -> Iterator[JointPositions]:
            # Interpolate in end effector space.
            active_joints1 = pt1[: self._num_active_dof]
            active_joints2 = pt2[: self._num_active_dof]
            active_ee_pose1 = self._active_arm.forward_kinematics(active_joints1)
            active_ee_pose2 = self._active_arm.forward_kinematics(active_joints2)
            for active_ee_pose in interpolate_poses(
                active_ee_pose1,
                active_ee_pose2,
                num_interp=self._birrt_extend_num_interp,
                include_start=False,
            ):
                # Run inverse kinematics for both robot and human.
                active_joints = inverse_kinematics(self._active_arm, active_ee_pose)
                passive_ee_pose = multiply_poses(
                    active_ee_pose, active_ee_to_passive_ee
                )
                passive_joints = inverse_kinematics(self._passive_arm, passive_ee_pose)
                yield active_joints + passive_joints

        # Just use end effector positions for distance for now.
        def _distance_fn(pt1: JointPositions, pt2: JointPositions) -> float:
            active_joints1 = pt1[: self._num_active_dof]
            active_joints2 = pt2[: self._num_active_dof]
            from_ee = self._active_arm.forward_kinematics(active_joints1).position
            to_ee = self._active_arm.forward_kinematics(active_joints2).position
            return sum(np.subtract(from_ee, to_ee) ** 2)

        # Collision function doesn't do anything.
        _collision_fn = lambda _: False

        birrt = BiRRT(
            _sampling_fn,
            _extend_fn,
            _collision_fn,
            _distance_fn,
            self._rng,
            num_attempts=self._birrt_num_attempts,
            num_iters=self._birrt_num_iters,
            smooth_amt=self._birrt_smooth_amt,
        )

        start_time = time.perf_counter()
        plan = birrt.query(init_joints, target_joints)
        print("Motion planning duration:", time.perf_counter() - start_time)

        # Uncomment to debug.
        for s in plan:
            active_joints = s[: self._num_active_dof]
            passive_joints = s[self._num_active_dof :]
            self._active_arm.set_joints(active_joints)
            self._passive_arm.set_joints(passive_joints)
            time.sleep(0.1)

        import ipdb

        ipdb.set_trace()

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
