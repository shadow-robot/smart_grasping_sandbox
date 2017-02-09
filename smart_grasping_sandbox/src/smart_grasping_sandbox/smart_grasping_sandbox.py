import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler
from math import pi
from copy import deepcopy

class SmartGraspingSandbox(object):

    def __init__(self):
        self.__reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.__get_ball_pose = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.__visualisation = rospy.Publisher("~display", Marker, queue_size=1, latch=True)
        self.__arm_commander = MoveGroupCommander("arm")
        self.__hand_commander = MoveGroupCommander("hand")

    def reset_world(self):
        self.__reset_world.call()

    def get_ball_pose(self):
        return self.__get_ball_pose.call("cricket_ball", "world")
