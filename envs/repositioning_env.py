"""Repositioning environment base class."""

import abc

from structs import (Image, JointTorques, RepositioningGoal,
                     RepositioningSceneConfig, RepositioningState)


class RepositioningEnv(abc.ABC):
    """Repositioning environment base class."""

    @abc.abstractmethod
    def get_scene_config(self) -> RepositioningSceneConfig:
        """Get the static scene configuration."""

    @abc.abstractmethod
    def get_torque_limits(self) -> tuple[JointTorques, JointTorques]:
        """Returns lower and upper limits."""

    @abc.abstractmethod
    def reset(self) -> None:
        """Reset the environment to its initial state."""

    @abc.abstractmethod
    def get_goal(self) -> RepositioningGoal:
        """Get the goal state."""

    @abc.abstractmethod
    def get_state(self) -> RepositioningState:
        """Get the current state of the environment."""

    @abc.abstractmethod
    def step(self, action: JointTorques) -> None:
        """Execute the action."""

    @abc.abstractmethod
    def render(self) -> Image:
        """Render an image of the current state."""
