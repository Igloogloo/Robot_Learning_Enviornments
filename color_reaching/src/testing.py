from color_reacher import ColorReacher
from continuous_cartesian import go_to_relative
import rospy
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
#rospy.init_node('testing')
#env = ColorReacher()
#arm = ArmMoveIt()
#print("AAAAA")
go_to_relative([0.01,0.01,0,0,0,0], collision_check=True)
# print(arm.check_arm_collision())