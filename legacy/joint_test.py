import sapien
from mani_skill.envs.scene import ManiSkillScene
from mani_skill.utils.building import MJCFLoader
loader = MJCFLoader()
loader.set_scene(ManiSkillScene())
robot = loader.load("crane_x7.xml")
print(robot.active_joints_map.keys())
