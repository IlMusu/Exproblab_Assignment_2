#!/usr/bin/env python

# Imporing ROS library for python
import rospy
import actionlib
import math

from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleGoalState
from robot_state_msgs.msg import FollowPathAction, FollowPathResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class MotionController():

    def __init__(self):
        # Creating a ROS Node.
        rospy.init_node('motion_controller', log_level=rospy.INFO)
        # Initializing variables
        self._robot_position = Point()
        # Creating an ActionClient to the move_base action server.
        self._move_acl = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
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
        # Storing the new robot position
        self._robot_position.x = msg.pose.pose.position.x
        self._robot_position.y = msg.pose.pose.position.y
        

    def _follow_path(self, action):
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
            
            # Manually checking if the robot is arrived at position
            rate = rospy.Rate(1)
            threshold = 10000
            while threshold > 1.0:
                # Check if the position is marked as reached by the move_base
                # action server. In this case the robot would not move anymore.
                if self._move_acl.simple_state == SimpleGoalState.DONE:
                    break
                rate.sleep()
                threshold = self._compute_robot_distance(pose)
                rospy.loginfo("[MOVER] New threshold is "+str(threshold))
                
            # Canceling the goal if not completed, that is because the treshold
            # has already been reached.
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

    
    def _compute_robot_distance(self, goal):
        dx = self._robot_position.x-goal.x
        dy = self._robot_position.y-goal.y
        return math.sqrt(dx*dx + dy*dy)


if __name__ == "__main__":
    # Creating the mover and then spinning to prevent
    # python from deallocating resources.
    MotionController()
    rospy.spin()
