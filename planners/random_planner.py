"""A very naive 'planner' that just outputs random actions."""

import numpy as np

from structs import JointTorques, RepositioningGoal, RepositioningState

from .base_planner import RepositioningPlanner


class RandomRepositioningPlanner(RepositioningPlanner):
    """A very naive 'planner' that just outputs random actions."""

    def reset(
        self,
        initial_state: RepositioningState,
        goal: RepositioningGoal,
    ) -> None:
        pass

    def step(self, state: RepositioningState) -> JointTorques:
        lower = self._scene_config.torque_lower_limits
        upper = self._scene_config.torque_upper_limits
        scale = np.subtract(upper, lower)
        shift = np.array(lower)
        num_dof = len(lower)
        norm_action = self._rng.uniform(
            0,
            1,
            size=num_dof,
        )
        action = norm_action * scale + shift
        return action.tolist()
