from pathlib import Path
from typing import Optional

from pybullet_helpers.ikfast import IKFastInfo
from pybullet_helpers.joint import JointPositions
from pybullet_helpers.robots.single_arm import SingleArmPyBulletRobot


class PandaPybulletRobotLimbRepo(SingleArmPyBulletRobot):
    """Franka Emika Panda with the limb repo end effector block."""

    @classmethod
    def get_name(cls) -> str:
        return "panda-limb-repo"

    @classmethod
    def urdf_path(cls) -> Path:
        dir_path = Path(__file__).parent / "urdf"
        return dir_path / "panda_limb_repo" / "panda_limb_repo.urdf"

    @property
    def default_home_joint_positions(self) -> JointPositions:
        return [
            -0.54193711,
            -1.07197495,
            -2.81591873,
            -1.6951869,
            2.48184051,
            -1.43600207,
        ]

    @property
    def end_effector_name(self) -> str:
        return "ee_fixed_joint"

    @property
    def tool_link_name(self) -> str:
        return "ee_link"

    @classmethod
    def ikfast_info(cls) -> Optional[IKFastInfo]:
        return IKFastInfo(
            module_dir="panda_limb_repo_arm",
            module_name="ikfast_panda_limb_repo_arm",
            base_link="panda_link0",
            ee_link="ee_link",
            free_joints=[],
        )
