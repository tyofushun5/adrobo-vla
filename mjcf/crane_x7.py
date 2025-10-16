import sapien
import numpy as np
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent


@register_agent()
class CraneX7(BaseAgent):
    uid = "crane_x7"
    mjcf_path = "crane_x7.xml"
    # keyframes = dict(
    #     rest=Keyframe(
    #         qpos=np.array(
    #             [0.0, np.pi / 8, 0, -np.pi * 5 / 8, 0, np.pi * 3 / 4, np.pi / 4, 0.04, 0.04]
    #         ),
    #         pose=sapien.Pose(),
    #     )
    # )
    arm_joint_names = [
        "crane_x7_shoulder_fixed_part_pan_joint",
        "crane_x7_shoulder_revolute_part_tilt_joint",
        "crane_x7_upper_arm_revolute_part_twist_joint",
        "crane_x7_upper_arm_revolute_part_rotate_joint",
        "crane_x7_lower_arm_fixed_part_joint",
        "crane_x7_lower_arm_revolute_part_joint",
        "crane_x7_wrist_joint",
    ]
    gripper_joint_names = [
        "crane_x7_gripper_finger_a_joint",
        "crane_x7_gripper_finger_b_joint",
    ]

