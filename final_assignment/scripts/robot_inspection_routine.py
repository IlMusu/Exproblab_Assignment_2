#!/usr/bin/env python

import rospy
import actionlib

from math import copysign

from std_msgs.msg import Float64
from final_assignment.msg import RobotInspectionRoutineAction, RobotInspectionRoutineResult

class RobotInspectionRoutine():
    
    def __init__(self):
        rospy.init_node('robot_inspection_routine', log_level=rospy.INFO)
        # Initializing variables
        self._is_inspecting = False;
        
        # Publishers for the arm joits
        self._joint0_pub = rospy.Publisher(
            '/joint0_position_controller/command', Float64, queue_size=1
        )
        self._joint1_pub = rospy.Publisher(
            '/joint1_position_controller/command', Float64, queue_size=1
        )
        self._joint2_pub = rospy.Publisher(
            '/camera_position_controller/command', Float64, queue_size=1
        )
        
        # Action server for starting the inspection routine
        self._routine_asv = actionlib.SimpleActionServer(
            '/robot_inspection_routine', RobotInspectionRoutineAction, auto_start=False,
            execute_cb=self._on_inspection_request,
        )
        self._routine_asv.start()
    
    
    def _on_inspection_request(self, action):
        # Check if the robot is already inspecting
        if self._is_inspecting :
            rospy.logerr("[INSPECTION ROUTINE] Robot is already inspecting!")
            result = RobotInspectionRoutineResult()
            result.completed = False
            self._routine_asv.set_aborted(result)
            return
        # Starting a new inspection routine
        self._is_inspecting = True
        rospy.loginfo("[INSPECTION ROUTINE] Starting a new inspection routine!")
        # Initially setting the rotation all to one side
        self._rotate_arm_joint_at(self._joint1_pub, 0.0)
        self._rotate_arm_joint_smoothly(self._joint0_pub, 0.05, 0.0, -3.1)
        # Rotate camera down and 360 deg
        self._rotate_arm_joint_at(self._joint2_pub, 0.3)
        self._rotate_arm_joint_smoothly(self._joint0_pub, 0.03, -3.1, 3.1)
        # Rotate camera a little up and -360 deg
        self._rotate_arm_joint_at(self._joint2_pub, -0.8)
        self._rotate_arm_joint_smoothly(self._joint0_pub, 0.03, 3.1, -3.1)
        # Returning to the initial position
        self._rotate_arm_joint_at(self._joint2_pub, 0.0)
        self._rotate_arm_joint_smoothly(self._joint0_pub, 0.05, -3.1, 0.0)
        
        # Publishing the result
        result = RobotInspectionRoutineResult()
        result.completed = True
        self._routine_asv.set_succeeded(result)
        rospy.loginfo("[INSPECTION ROUTINE] Inspection routine completed!")
        self._is_inspecting = False

    
    def _rotate_arm_joint_smoothly(self, pub, delta, yaw, target_yaw):
        # Defining a rate for moving the arm slowly
        rate = rospy.Rate(10)
        # The signed delta
        update_delta_sign = copysign(1, target_yaw-yaw)
        update_delta_yaw = update_delta_sign*delta
        
        # Move until the target yaw is reached
        while yaw*update_delta_sign < target_yaw*update_delta_sign :
            # Publishing the new control
            self._rotate_arm_joint_at(pub, yaw)
            # Updating the yaw value
            yaw += update_delta_yaw
            # Sleeping until the next cycle
            rate.sleep()


    def _rotate_arm_joint_at(self, pub, yaw):
        control = Float64()
        control.data = yaw
        pub.publish(control)
        rospy.sleep(rospy.Duration(0.1))
        

if __name__ == "__main__" :
    RobotInspectionRoutine()
    rospy.spin()
    
