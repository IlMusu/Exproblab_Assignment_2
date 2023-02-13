#!/usr/bin/env python

# Importing ROS library for python
import rospy
import actionlib

from math import copysign

from std_msgs.msg import Float64
from final_assignment.msg import RobotInspectionRoutineAction, RobotInspectionRoutineResult

class RobotInspectionRoutine():
    '''
    Publishes to the topics:
        - /joint0_position_controller/command (Float64)
        - /joint1_position_controller/command (Float64)
        - /camera_position_controller/command (Float64)
    Creates action servers for:
        - /robot_inspection_routine (RobotInspectionRoutineAction)
    
    This ROS Node makes the arm of the robot rotate in circular patters.
    This behavior is achieved by subscribing to the topics of each joint
    of the arm and manually setting the angle values.
    '''
    def __init__(self):
        '''
        |  In the constructor method, this class:
        |  1. Creates a subscriber for the "/joint0_position_controller/command" topic.
        |  2. Creates a subscriber for the "/joint1_position_controller/command" topic.
        |  3. Creates a subscriber for the "/camera_position_controller/command" topic.
        |  4. Creates an action server for the "/robot_inspection_routine" action.
        '''
        rospy.init_node('robot_inspection_routine', log_level=rospy.INFO)
        # Initializing variables
        self._is_inspecting = False
        
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
        '''
        Args:
            action (RobotInspectionRoutineGoal) : the goal.
        
        | This is the callback for the /robot_inspection_routine action server:
        | 1. If the routine is already taking place:
        |  2. Sends the result as failed.
        | 3. Makes the first joint rotate to -pi.
        | 4. Makes the camera joint to 0.3.
        | 5. Makes the first joint rotate to pi.
        | 6. Makes the camera rotate to -0.8.
        | 7. Makes the first joint rotate to -pi.
        | 8. Resets the joint positions.
        | 9. Sends the result as success.
        '''
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
        self._rotate_arm_joint_smoothly(self._joint0_pub, 0.1, 0.0, -3.1)
        # Rotate camera down and 360 deg
        self._rotate_arm_joint_at(self._joint2_pub, 0.25)
        self._rotate_arm_joint_smoothly(self._joint0_pub, 0.1, -3.1, 3.1)
        rospy.sleep(rospy.Duration(0.5))
        # Rotate camera a little up and -360 deg
        self._rotate_arm_joint_at(self._joint2_pub, -0.7)
        self._rotate_arm_joint_smoothly(self._joint0_pub, 0.1, 3.1, -3.1)
        rospy.sleep(rospy.Duration(0.5))
        # Returning to the initial position
        self._rotate_arm_joint_at(self._joint2_pub, 0.0)
        self._rotate_arm_joint_smoothly(self._joint0_pub, 0.1, -3.1, 0.0)
        
        # Publishing the result
        result = RobotInspectionRoutineResult()
        result.completed = True
        self._routine_asv.set_succeeded(result)
        rospy.loginfo("[INSPECTION ROUTINE] Inspection routine completed!")
        self._is_inspecting = False

    
    def _rotate_arm_joint_smoothly(self, pub, delta, value, target_value):
        '''
        Args:
            pub (Publisher) : the publisher for the joint controller.
            delta (float) : the increment used to update the value at each iteration.
            value (float) : the value for the current configuration of the joint.
            target_value (float) : the value for the target configuration of the joint.

        This method makes the arm joint of the robot rotate by incrementing the value
        of the configuration with a delta at each iteration.
        '''
        # Defining a rate for moving the arm slowly
        rate = rospy.Rate(10)
        # The signed delta
        update_delta_sign = copysign(1, target_value-value)
        update_delta_value = update_delta_sign*delta
        
        # Move until the target value is reached
        while value*update_delta_sign < target_value*update_delta_sign :
            # Publishing the new control
            self._rotate_arm_joint_at(pub, value)
            # Updating the value value
            value += update_delta_value
            # Sleeping until the next cycle
            rate.sleep()


    def _rotate_arm_joint_at(self, pub, value):
        '''
        Args:
            pub (Publisher) : the publisher for the joint controller.
            value (float) : the value for the new configuration of the joint.
        
        This method makes the arm joint of the robot rotate to the
        requested configuration by publishing the value on the topic.
        '''
        control = Float64()
        control.data = value
        pub.publish(control)
        rospy.sleep(rospy.Duration(0.2))
        

if __name__ == "__main__" :
    RobotInspectionRoutine()
    rospy.spin()
    
