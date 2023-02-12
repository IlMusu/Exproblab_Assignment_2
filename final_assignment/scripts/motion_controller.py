#!/usr/bin/env python

# Imporing ROS library for python
import rospy
import actionlib
import math

from actionlib import SimpleGoalState

from actionlib_msgs.msg import GoalStatus
from robot_state_msgs.msg import FollowPathAction, FollowPathResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class MotionController():
    '''
    Creates an action client for:
        - /move_base (MoveBaseAction)
    Creates an action server for:
        - /follow_path (FollowPathAction)
    Subscribes to topics:
        - /odom (Odometry)
    ROS Parameters :
        - /goal_threshold (float) : The distance from goal at which the robot is
            considered to be arrived at goal.
    
    This Ros Node makes the robot follow a path composed by waypoints.
    The actual movements between the waypoints is perfomed through move_base.
    Sometimes the robot gets stuck just before reaching the requested goal,
    because of this reason, this node checks the distance from the robot and
    the current goal position and interrupts move_base when the distance is
    under the user defined threshold.
    '''
    def __init__(self):
        '''
        |  In the constructor method, this class:
        |  1. Creates an action client for the "/move_base" action.
        |  2. Creates an action server for the "/follow_path" action.
        |  3. Creates a subscriber for the "/odom" topic.
        '''
        # Creating a ROS Node.
        rospy.init_node('motion_controller', log_level=rospy.INFO)
        # Initializing variables
        self._robot_position = Point()
        self._goal_threshold = rospy.get_param("~goal_threshold", 1.5)
        # Creating an ActionClient to the move_base action server.
        self._move_acl = actionlib.SimpleActionClient(
            "/move_base", MoveBaseAction
        )
        # Creating an ActionServer for receving a path.
        self._follow_asv = actionlib.SimpleActionServer(
            '/follow_path', FollowPathAction, auto_start=False,
            execute_cb=self._follow_path,
        )
        # Creating a Subscriber to the odom topic for the move_base threshold
        self._odom_sub = rospy.Subscriber(
            '/odom', Odometry, queue_size=10,
            callback = self._on_odom_callback
        )
        self._follow_asv.start()
    
    
    def _on_odom_callback(self, msg):
        '''
        Args:
            msg (Odometry) : the message containing the robot odometry.
        
        This is the callback for the /odom topic. The message contains the
        current position of the robot which is stored to be used later.
        '''
        # Storing the new robot position
        self._robot_position.x = msg.pose.pose.position.x
        self._robot_position.y = msg.pose.pose.position.y
        

    def _follow_path(self, action):
        '''
        Args:
            action (FollowPathGoal) : the requested path to follow.
        
        | This is the callback for the /follow_path action server:
        | 1. Waits until the move_base action server is available.
        | 2. For each waypoint inside the requested path:
        |  3. Creates a MoveBaseAction containing the waypoint as goal.
        |  4. Sends the goal the move_base action server.
        |  5. Waits until the robot is arried at the waypoint.
        | 6. Creates and publishes the results.
        '''
        # Waiting until move_base exists.
        self._move_acl.wait_for_server()
        # Moving to each waypoint of the path using move_base.
        has_failed = False
        for pose in action.path :
            # Creating the goal containing the target position.
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.orientation.w = 1
            goal.target_pose.pose.position.x = pose.x
            goal.target_pose.pose.position.y = pose.y
            rospy.loginfo("[MOVER] Moving to (x:"+str(pose.x)+", y:"+str(pose.y)+")!");
            
            # Sending the goal to the move_base server.
            self._move_acl.send_goal(goal)
            # Waiting until the robot is arrived at destination.
            self._wait_until_robot_at_goal()
                
            # Canceling the goal if not completed.
            if self._move_acl.simple_state != SimpleGoalState.DONE:
                self._move_acl.cancel_goal()
            
            # Check if the motion was completed successfully.
            if self._move_acl.get_state() != GoalStatus.SUCCEEDED :
                has_failed = True
                break
        
        # Publishing the final result
        result = FollowPathResult()
        result.position.x = self._robot_position.x
        result.position.y = self._robot_position.y
        if not has_failed:
            self._follow_asv.set_succeeded(result)
        else:
            self._follow_asv.set_aborted(result)


    def _wait_until_robot_at_goal(self):
        '''
        This function blocks the execution until the robot is arrived at destination
        which might happen because of two reasons: the first one is that move_base
        completes the execution of the action, the second one is because the distance
        between the robot and the goal is less than the /goal_threshold parameter.
        '''
        # Manually checking if the robot is arrived at position
        rate = rospy.Rate(1)
        distance = 10000
        while distance > self._goal_threshold:
            # Check if the position is marked as reached by the move_base
            # action server. In this case the robot would not move anymore.
            if self._move_acl.simple_state == SimpleGoalState.DONE:
                break
            rate.sleep()
            distance = self._compute_robot_distance(pose)
            rospy.loginfo("[MOVER] Current distance is "+str(distance))


    
    def _compute_robot_distance(self, goal):
        '''
        Args:
            goal (Point) : the goal position.
        
        Computes the distance between the robot and the goal.
        '''
        dx = self._robot_position.x-goal.x
        dy = self._robot_position.y-goal.y
        return math.sqrt(dx*dx + dy*dy)


if __name__ == "__main__":
    # Creating the mover and then spinning to prevent
    # python from deallocating resources.
    MotionController()
    rospy.spin()
