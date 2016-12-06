import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from visualization_msgs.msg import Marker
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler
from math import pi
from copy import deepcopy

class Grasp(object):

    def __init__(self):
        self.__reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)

        self.__get_ball_pose = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.__visualisation = rospy.Publisher("~display", Marker, queue_size=1, latch=True)

        self.__arm_commander = MoveGroupCommander("arm")
        self.__hand_commander = MoveGroupCommander("hand")
        self.pick()

    def pick(self):
        self.go_to_start()

        arm_target = self.compute_arm_target()

        self.pre_grasp(arm_target)
        self.grasp(arm_target)
        self.lift(arm_target)

    def compute_arm_target(self):
        ball_pose = self.__get_ball_pose.call("cricket_ball", "world")

        # come at it from the top
        arm_target = ball_pose.pose
        arm_target.position.z += 0.5

        quaternion = quaternion_from_euler(-pi/2., 0.0, 0.0)
        arm_target.orientation.x = quaternion[0]
        arm_target.orientation.y = quaternion[1]
        arm_target.orientation.z = quaternion[2]
        arm_target.orientation.w = quaternion[3]

        target_marker = Marker()
        target_marker.action = Marker.MODIFY
        target_marker.lifetime.from_sec(10.0)
        target_marker.type = Marker.ARROW
        target_marker.header.stamp = rospy.Time.now()
        target_marker.header.frame_id = "world"
        target_marker.pose = arm_target
        target_marker.scale.z = 0.01
        target_marker.scale.x = 0.1
        target_marker.scale.y = 0.01
        target_marker.color.r = 1.0
        target_marker.color.g = 0.8
        target_marker.color.b = 0.1
        target_marker.color.a = 1.0

        self.__visualisation.publish(target_marker)

        return arm_target

    def pre_grasp(self, arm_target):
        self.__hand_commander.set_named_target("open")
        plan = self.__hand_commander.plan()
        self.__hand_commander.execute(plan, wait=True)

        for _ in range(10):
            self.__arm_commander.set_start_state_to_current_state()
            self.__arm_commander.set_pose_targets([arm_target])
            plan = self.__arm_commander.plan()
            if self.__arm_commander.execute(plan):
                return True

    def grasp(self, arm_target):
        waypoints = []
        waypoints.append(self.__arm_commander.get_current_pose(self.__arm_commander.get_end_effector_link()).pose)
        arm_above_ball = deepcopy(arm_target)
        arm_above_ball.position.z -= 0.12
        waypoints.append(arm_above_ball)

        self.__arm_commander.set_start_state_to_current_state()
        (plan, fraction) = self.__arm_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        print fraction
        if not self.__arm_commander.execute(plan):
            return False

        self.__hand_commander.set_named_target("close")
        plan = self.__hand_commander.plan()
        if not self.__hand_commander.execute(plan, wait=True):
            return False

        self.__hand_commander.attach_object("cricket_ball__link")

    def lift(self, arm_target):
        waypoints = []
        waypoints.append(self.__arm_commander.get_current_pose(self.__arm_commander.get_end_effector_link()).pose)
        arm_above_ball = deepcopy(arm_target)
        arm_above_ball.position.z += 0.1
        waypoints.append(arm_above_ball)

        self.__arm_commander.set_start_state_to_current_state()
        (plan, fraction) = self.__arm_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        print fraction
        if not self.__arm_commander.execute(plan):
            return False

    def go_to_start(self):
        self.__arm_commander.set_named_target("start")
        plan = self.__arm_commander.plan()
        if not self.__arm_commander.execute(plan, wait=True):
            return False

        self.__hand_commander.set_named_target("open")
        plan = self.__hand_commander.plan()
        self.__hand_commander.execute(plan, wait=True)

        self.__reset_world.call()


if __name__=="__main__":
    rospy.init_node("smart_grasper")
    Grasp()
    #rospy.spin()
