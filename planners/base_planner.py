"""Base class for repositioning planner."""

import abc

import numpy as np

from dynamics.base_model import RepositioningDynamicsModel
from structs import (JointTorques, RepositioningGoal, RepositioningSceneConfig,
                     RepositioningState)


class RepositioningPlanner(abc.ABC):
    """Base class for repositioning planner."""

    def __init__(
        self,
        scene_config: RepositioningSceneConfig,
        dynamics: RepositioningDynamicsModel,
        T: float,
        dt: float,
        seed: int,
    ) -> None:
        self._scene_config = scene_config
        self._dynamics = dynamics
        self._T = T
        self._dt = dt
        self._seed = seed
        self._rng = np.random.default_rng(seed)
        self._active_arm = dynamics.active_arm
        self._passive_arm = dynamics.passive_arm
        self._dt = dynamics.dt

    @abc.abstractmethod
    def reset(
        self,
        initial_state: RepositioningState,
        goal: RepositioningGoal,
    ) -> None:
        """Run planning on a new problem."""

    @abc.abstractmethod
    def step(self, state: RepositioningState) -> JointTorques:
        """Get the next action to execute in the given state."""
