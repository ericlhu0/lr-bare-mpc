"""Helper function for creating environments."""

from .panda_human_env import PandaHumanRepositioningEnv
from .repositioning_env import RepositioningEnv
from .two_link_env import TwoLinkRepositioningEnv


def create_env(name: str, *args, **kwargs) -> RepositioningEnv:
    """Helper function for creating environments."""

    if name == "panda-human":
        return PandaHumanRepositioningEnv(*args, **kwargs)

    if name == "two-link":
        return TwoLinkRepositioningEnv(*args, **kwargs)

    raise NotImplementedError
