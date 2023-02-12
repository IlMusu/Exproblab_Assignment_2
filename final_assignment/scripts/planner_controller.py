#!/usr/bin/env python

# Importing ROS library for python
import rospy
import actionlib

from robot_state_msgs.msg import ComputePathAction, ComputePathResult
from geometry_msgs.msg import Point

class PlannerController():

    def __init__(self):
        # Creating a ROS Node.
        rospy.init_node('planner_controller', log_level=rospy.INFO)
        # Creating an ActionServer to compute a plan.
        self._planner_asv = actionlib.SimpleActionServer(
            '/compute_path', ComputePathAction, auto_start=False,
            execute_cb=self._compute_path,
        )
        self._planner_asv.start()


    def _compute_path(self, action):
        # The plan is just the the start and goal positions.
        # This planner should be used to compute high-level paths when
        # dealing with really big environments. In this case we are in
        # a small enviroment which does not require a high-level path.
        result = ComputePathResult()
        result.path = [action.goal]
        # Logging some information
        gx = str(action.goal.x)
        gy = str(action.goal.y)
        rospy.loginfo("[PLANNER] Computed a plan to (x:"+gx+",y:"+gy+")!");
        # Responding with the new plan.
        self._planner_asv.set_succeeded(result)



if __name__ == "__main__":
    # Creating the planner and then spinning to prevent
    # python from deallocating resources.
    PlannerController()
    rospy.spin()
