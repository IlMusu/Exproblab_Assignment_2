#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
import actionlib

from actionlib import SimpleGoalState
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from final_assignment.srv import RoomInformation
from final_assignment.msg import RoomConnection
from final_assignment.msg import RoomDoors
from final_assignment.msg import OntologyMap
from final_assignment.msg import RobotInspectionRoutineAction, RobotInspectionRoutineGoal


class MarkerDetector():
    '''
    Subscribes to topics:
        - /camera/image_raw (Image)
    Publishes to topics:
        - /cmd_vel (Twist)
        - /ontology_map/build_map (OntologyMap)
    Requires services on:
        - /room_info (RoomInformation)
    Creates action clients for:
        - /robot_inspection_routine (RobotInspectionRoutine)
    ROS Parameters :
        - /markers_count (int) : The number of markers to detect.
    
    This ROS node makes the robot detect the specified number of ArUco 
    markers that are located around it. To accomplish this, it sends a 
    goal to the /robot_inspection_routine action server, which causes 
    the robot arm to rotate in circular patterns. Once the inspection 
    routine is completed, it obtains information about the markers that 
    it has detected using the /room_info service. These steps are 
    repeated until all markers have been detected. Finally, it computes 
    a new OntologyMap message containing all the necessary information 
    about the ontology and publishes it.
    '''
    def __init__(self):
        '''
        |  In the constructor method, this class:
        |  1. Creates a subscriber for the "/camera/image_raw" topic.
        |  2. Creates a publisher for the "/cmd_vel" topic.
        |  3. Creates a publisher for the "/ontology_map/build_map/" topic.
        |  4. Creates a service client for the "/room_info" service.
        |  5. Creates an action client for the "/robot_inspection_routine" action.
        |  6. Finally, it starts to look for ArUco markers.
        '''
        # Initializing ROS Node
        rospy.init_node("marker_detector")
        # Initializing variables
        self._aruco_params =  aruco.DetectorParameters_create()
        self._cv_bridge = CvBridge()
        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self._detected_markers = []
        self._validated_markers = []
        self._onto_map_msg = OntologyMap()
        self._markers_count = rospy.get_param("~markers_count")
        # Creating a Subscriber for the image topic
        self._image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, queue_size=100, 
            callback = self._image_callback
        )
        # Creating a Publisher for the robot velocity
        self._vel_pub = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=1000, latch=True
        )
        # Creating a Publisher for the ontology map
        self._onto_map_pub = rospy.Publisher(
            "/ontology_map/build_map", OntologyMap, queue_size=100, latch=True
        )
        # Creating a ServiceClient for the rooms information
        self._room_info_srv = rospy.ServiceProxy(
            "/room_info", RoomInformation
        )
        # Creating an ActionClient for moving the robot arm
        self._arm_acl = actionlib.SimpleActionClient(
            "/robot_inspection_routine", RobotInspectionRoutineAction
        )
        # Starting to look for aruco markers
        self._look_for_aruco_markers()


    def _look_for_aruco_markers(self):
        '''
        This method is used for detecting ArUco markers. It starts by setting a 
        low velocity for the robot so that it remains stationary while searching 
        for markers. After the /robot_inspection_routine action server is detected
        and until all the markers are correctly detected, a goal is sent to it, 
        and the completion is awaited. When all the markers are correctly detected, 
        a new message is published on the /ontology_map/build_map channel with 
        information about the ontology.
        '''
        # Setting the robot velocity very small in order to keep it still
        vel_msg = Twist()
        vel_msg.linear.x = 0.001
        self._vel_pub.publish(vel_msg)
        # Waiting for the server to be available
        rospy.loginfo("[MARKER DETECTOR] Waiting for InspectionRoutine Server...")
        self._arm_acl.wait_for_server()
        rospy.loginfo("[MARKER DETECTOR] InspectionRoutine Server found!")
        # Finding all the markers in the room
        while len(self._validated_markers) < self._markers_count :
            # Creating a goal for the movement of the arm
            inspection_goal = RobotInspectionRoutineGoal()
            self._arm_acl.send_goal(inspection_goal)
            # Waiting for the arm movement to complete
            self._arm_acl.wait_for_result()
            # Check if the arm movement was complete
            if not self._arm_acl.get_result().completed :
                rospy.logwarn("[MARKER DETECTOR] Inspection routine was not executed!")
                rospy.logwarn("[MARKER DETECTOR] Waiting for a bit and then retrying!")
                rospy.sleep(10)
                continue
            # Retrieving all the rooms information
            self._retrieve_rooms_information()
            rospy.loginfo("[MARKER DETECTOR] Validated "+str(len(self._validated_markers))+" markers!")
        # Stopping the subscriber for the camera image
        self._image_sub.unregister()
        # Publishing the ontology map
        self._onto_map_pub.publish(self._onto_map_msg)


    def _image_callback(self, msg):
        '''
        This is the callback for the /camera/image_raw topic.
        When a new image message is available, this method encodes it using cv_bridge.
        Then, dectects any eventual the ArUco markers in the image using the
        apposite function. The dected markers id are not ignored if there are not
        already validated or are not already found.
        '''
        # Encoding the image using cv
        image = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        # Getting the markers from aruco
        corners, ids, _ = aruco.detectMarkers(image, self._aruco_dict, parameters=self._aruco_params)
        if len(corners) == 0 :
            return
        # Adding all the new detected ids to the set
        for _id in ids.flatten() :
            # Check if the marker id was already validated
            if _id in self._validated_markers :
                continue
            # Check if the marker was already detected
            if _id in self._detected_markers :
                continue
            # A new marker has been found
            self._detected_markers.append(_id)
            rospy.loginfo("[MARKER DETECTOR] New marker with id "+str(_id)+" detected!")
    

    def _retrieve_rooms_information(self):
        '''
        This method is called when new ArUco markers ids are detected and
        the related information needs to be obtained. In order to do this,
        it calls the /room_info service for each detected id.
        If the id, was valid, adds the information to the OntologyMap
        message, adds the id to the list of "validated markers".
        Finally, it clears the list of detected markers.
        '''
        # Waiting for the service to be available
        rospy.loginfo("[MARKER DETECTOR] Waiting for MarkerServer Service...")
        self._room_info_srv.wait_for_service()
        rospy.loginfo("[MARKER DETECTOR] MarkerServer Service found!")
        # Creating a new room information message
        room_info_msg = RoomInformation()
        for _id in self._detected_markers :
            # Requesting information to the room service
            rospy.loginfo("[MARKER DETECTOR] Requesting info for ID "+str(_id))
            response = self._room_info_srv(_id)
            # Check if the id was valid
            if response.room == "no room associated with this marker id":
                rospy.loginfo("[MARKER DETECTOR] Detected marker with invalid ID "+str(_id)+", ignoring")
                continue
            # Storing the room name
            self._onto_map_msg.rooms.append(response.room)
            # Storing the room positions
            point = Point()
            point.x = response.x
            point.y = response.y
            self._onto_map_msg.positions.append(point)
            # Storing the room connections
            room_doors = RoomDoors()
            for connection in response.connections :
                room_doors.doors.append(connection.through_door)
            self._onto_map_msg.doors.append(room_doors)
            # The marker has been parsed as valid
            self._validated_markers.append(_id)
        # The detected markers have been handled
        self._detected_markers.clear()


if __name__ == "__main__":
    MarkerDetector()
    rospy.spin()

