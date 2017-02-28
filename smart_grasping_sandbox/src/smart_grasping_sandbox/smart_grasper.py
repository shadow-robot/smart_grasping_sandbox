import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler
from math import pi
from copy import deepcopy

class SmartGrasper(object):

    def __init__(self):
        rospy.init_node("smart_grasper")

        self.__reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.__get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.arm_commander = MoveGroupCommander("arm")
        self.hand_commander = MoveGroupCommander("hand")

    def reset_world(self):
        """
        Resets the object poses in the world.
        """
        self.__reset_world.call()

    def get_ball_pose(self):
        """
        Gets the pose of the ball in the world frame.
        :return: The pose of the ball.
        """
        return self.__get_pose_srv.call("cricket_ball", "world")

    def go_to_start(self):
        """
        Resets the world and goes back to the start pose with the robot.
        """
        self.arm_commander.set_named_target("start")
        plan = self.arm_commander.plan()
        if not self.arm_commander.execute(plan, wait=True):
            return False

        self.hand_commander.set_named_target("open")
        plan = self.hand_commander.plan()
        self.hand_commander.execute(plan, wait=True)

        self.__reset_world.call()

    def pick(self):
        """
        Does its best to pick the ball.
        """
        self.go_to_start()

        arm_target = self.__compute_arm_target_for_ball()

        self.__pre_grasp(arm_target)
        self.__grasp(arm_target)
        self.__lift(arm_target)

    def __compute_arm_target_for_ball(self):
        ball_pose = self.get_ball_pose()

        # come at it from the top
        arm_target = ball_pose.pose
        arm_target.position.z += 0.5

        quaternion = quaternion_from_euler(-pi/2., 0.0, 0.0)
        arm_target.orientation.x = quaternion[0]
        arm_target.orientation.y = quaternion[1]
        arm_target.orientation.z = quaternion[2]
        arm_target.orientation.w = quaternion[3]

        return arm_target

    def __pre_grasp(self, arm_target):
        self.hand_commander.set_named_target("open")
        plan = self.hand_commander.plan()
        self.hand_commander.execute(plan, wait=True)

        for _ in range(10):
            self.arm_commander.set_start_state_to_current_state()
            self.arm_commander.set_pose_targets([arm_target])
            plan = self.arm_commander.plan()
            if self.arm_commander.execute(plan):
                return True

    def __grasp(self, arm_target):
        waypoints = []
        waypoints.append(self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose)
        arm_above_ball = deepcopy(arm_target)
        arm_above_ball.position.z -= 0.12
        waypoints.append(arm_above_ball)

        self.arm_commander.set_start_state_to_current_state()
        (plan, fraction) = self.arm_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        print fraction
        if not self.arm_commander.execute(plan):
            return False

        self.hand_commander.set_named_target("close")
        plan = self.hand_commander.plan()
        if not self.hand_commander.execute(plan, wait=True):
            return False

        self.hand_commander.attach_object("cricket_ball__link")

    def __lift(self, arm_target):
        waypoints = []
        waypoints.append(self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose)
        arm_above_ball = deepcopy(arm_target)
        arm_above_ball.position.z += 0.1
        waypoints.append(arm_above_ball)

        self.arm_commander.set_start_state_to_current_state()
        (plan, fraction) = self.arm_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        print fraction
        if not self.arm_commander.execute(plan):
            return False
