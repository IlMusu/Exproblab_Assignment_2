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

    def __init__(self):
        # Initializing ROS Node
        rospy.init_node("marker_detector")
        # Initializing variables
        self._aruco_params =  aruco.DetectorParameters_create()
        self._cv_bridge = CvBridge()
        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self._detected_markers = []
        self._validated_markers = []
        self._onto_map_msg = OntologyMap()
        self._markers_count = rospy.get_param("~markers_count", 7)
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
        self._look_for_aruco_markers()


    def _look_for_aruco_markers(self):
        # Waiting for the server to be available
        rospy.loginfo("[MARKER DETECTOR] Waiting for InspectionRoutine Server...")
        self._arm_acl.wait_for_server()
        rospy.loginfo("[MARKER DETECTOR] InspectionRoutine Server found!")
        # Setting the robot velocity very small in order to keep it still
        vel_msg = Twist()
        vel_msg.linear.x = 0.001
        self._vel_pub.publish(vel_msg)
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

