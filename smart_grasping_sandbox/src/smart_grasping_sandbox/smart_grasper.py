import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler
from math import pi
from copy import deepcopy
from tf_conversions import posemath, toMsg
import PyKDL

class SmartGrasper(object):

    def __init__(self):
        rospy.init_node("smart_grasper")

        rospy.wait_for_service("/gazebo/get_model_state", 10.0)
        rospy.wait_for_service("/gazebo/reset_world", 10.0)
        self.__reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.__get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        rospy.wait_for_service('/get_planning_scene', 10.0)
        self.__get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.__pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10, latch=True)

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
        return self.__get_pose_srv.call("cricket_ball", "world").pose

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

    def get_tip_pose(self):
        """
        Gets the current pose of the robot's tooltip in the world frame.
        @return the tip pose
        """
        return self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose

    def move_tip_absolute(self, target):
        """
        Moves the tooltip to the absolute target in the world frame

        @param target is a geometry_msgs.msg.Pose
        @return True on success
        """
        self.arm_commander.set_start_state_to_current_state()
        self.arm_commander.set_pose_targets([target])
        plan = self.arm_commander.plan()
        if not self.arm_commander.execute(plan):
            return False
        return True
        
    def move_tip(self, x=0., y=0., z=0., roll=0., pitch=0., yaw=0.):
        """
        Moves the tooltip in the world frame by the given x,y,z / roll,pitch,yaw. 

        @return True on success
        """
        transform = PyKDL.Frame(PyKDL.Rotation.RPY(pitch, roll, yaw),
                                PyKDL.Vector(-x, -y, -z))
        
        tip_pose = self.get_tip_pose()
        tip_pose_kdl = posemath.fromMsg(tip_pose)
        final_pose = toMsg(tip_pose_kdl * transform)
            
        self.arm_commander.set_start_state_to_current_state()
        self.arm_commander.set_pose_targets([final_pose])
        plan = self.arm_commander.plan()
        if not  self.arm_commander.execute(plan):
            return False
        return True

    def open_hand(self):
        """
        Opens the hand.
        @return True on success
        """
        self.hand_commander.set_named_target("open")
        plan = self.hand_commander.plan()
        if not self.hand_commander.execute(plan, wait=True):
            return False

        return True

    def close_hand(self):
        """
        Closes the hand.
        @return True on success
        """
        self.hand_commander.set_named_target("close")
        plan = self.hand_commander.plan()
        if not self.hand_commander.execute(plan, wait=True):
            return False

        return True

    def check_fingers_collisions(self, enable=True):
        """
        Disables or enables the collisions check between the fingers and the objects / table
        
        @param enable set to True to enable / False to disable
        @return True on success
        """
        objects = ["cricket_ball__link", "drill__link", "cafe_table__link"]

        while self.__pub_planning_scene.get_num_connections() < 1:
            rospy.loginfo("waiting for someone to subscribe to the /planning_scene")
            rospy.sleep(0.1)

        request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = self.__get_planning_scene(request)
        acm = response.scene.allowed_collision_matrix

        for object_name in objects:
            if object_name not in acm.entry_names:
                # add object to allowed collision matrix
                acm.entry_names += [object_name]
                for row in range(len(acm.entry_values)):
                    acm.entry_values[row].enabled += [False]

                new_row = deepcopy(acm.entry_values[0])
                acm.entry_values += {new_row}

        for index_entry_values, entry_values in enumerate(acm.entry_values):
            if "H1_F" in acm.entry_names[index_entry_values]:
                for index_value, value in enumerate(entry_values.enabled):
                    if acm.entry_names[index_value] in objects:
                        if enable:
                            acm.entry_values[index_entry_values].enabled[index_value] = False
                        else:
                            acm.entry_values[index_entry_values].enabled[index_value] = True
            elif acm.entry_names[index_entry_values] in objects:
                for index_value, value in enumerate(entry_values.enabled):
                    if "H1_F" in acm.entry_names[index_value]:
                        if enable:
                            acm.entry_values[index_entry_values].enabled[index_value] = False
                        else:
                            acm.entry_values[index_entry_values].enabled[index_value] = True
        
        planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
        self.__pub_planning_scene.publish(planning_scene_diff)
        rospy.sleep(1.0)

        return True

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
        arm_target = ball_pose
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
