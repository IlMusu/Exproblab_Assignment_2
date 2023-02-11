#include <ros/ros.h>
#include <aruco/aruco.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <final_assignment/RoomInformation.h>
#include <final_assignment/RoomConnection.h>
#include <final_assignment/RoomDoors.h>
#include <final_assignment/OntologyMap.h>
#include <geometry_msgs/Point.h>

aruco::CameraParameters defaultCameraParams;
std::set<int> detectedMarkers;

ros::ServiceClient roomInfoPub;
ros::Subscriber imageSub;
ros::Publisher velPub;
ros::Publisher j0Pub;
ros::Publisher j1Pub;
ros::Publisher j2Pub;
ros::Publisher mapPub;

final_assignment::OntologyMap ontoMapMsg;

void rotateArmAround();
bool retrieveRoomsInformation();
void imageCallback(const sensor_msgs::Image::ConstPtr& msg);


int main(int argc, char** argv)
{
    // Initializing the ROS Node
    ros::init(argc, argv, "aruco_detector");  
    ros::NodeHandle n;
    
    // Initializing variables
    defaultCameraParams = aruco::CameraParameters();
  
    // Creating a Subscriber for the image topic
    imageSub = n.subscribe("/camera1/camera1/image_raw", 1000, imageCallback);
    // Creating Publishers for the robot arm
    j0Pub = n.advertise<std_msgs::Float64>("/joint0_position_controller/command", 10);
    j1Pub = n.advertise<std_msgs::Float64>("/joint1_position_controller/command", 10);
    j2Pub = n.advertise<std_msgs::Float64>("/camera_position_controller/command", 10);
    mapPub = n.advertise<final_assignment::OntologyMap>("/ontology_map/build_map", 10);
    // Creating a Publisher for the robot velocity
    velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
    // Creating a ServiceClient for the rooms information
    roomInfoPub = n.serviceClient<final_assignment::RoomInformation>("/room_info");
    
    // Rotate the arm around for detecting all the markers
    rotateArmAround();
    // Unsubscribing from the image topic since it is not necessary anymore
    imageSub.shutdown();
    
    geometry_msgs::Twist velMsg;
    velPub.publish(velMsg);
    
    // TODO Remove this shit
    detectedMarkers.insert(11);
    detectedMarkers.insert(12);
    detectedMarkers.insert(13);
    detectedMarkers.insert(14);
    detectedMarkers.insert(15);
    detectedMarkers.insert(16);
    detectedMarkers.insert(17);
    
    // Retrieving all the rooms information
    retrieveRoomsInformation();
    // Publishin the ontology map
    mapPub.publish(ontoMapMsg);
    
    // Spinning to prevent stopping the execution
    ros::spin();
}


void rotateArmAround()
{
    if(true)
        return;
        
    // Setting the velocity of the robot as 0
    geometry_msgs::Twist velMsg;
    velPub.publish(velMsg);
    
    // The arm is initially down
    std_msgs::Float64 msgF;
    msgF.data = -1.0;
    j1Pub.publish(msgF);
    
    // Rotating the arm in small steps
    for(float i=-3.0; i<=3.0; i+=0.2)
    {
        msgF.data = i;
        j0Pub.publish(msgF);
        std::cout << i << "\n";
        // Waiting for a bit before stating to rotate again
        ros::Duration(1).sleep();
    }
}


void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Intializing some variables
    aruco::MarkerDetector detector;
    std::vector<aruco::Marker> markers;
    cv_bridge::CvImagePtr cvPtr;
    
    // Creating a CV copy of the image from the camera
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // Detecting ARUCO markers in the image
    detector.detect(cvPtr->image, markers, defaultCameraParams, 0.05, false);

    std::cout << "IDs detected: ";
    for (std::size_t i = 0; i < markers.size(); ++i)
    {
        int _id = markers.at(i).id;
        std::cout << _id << " ";
    }
    std::cout << std::endl;
}


bool retrieveRoomsInformation()
{
    final_assignment::RoomInformation info;
    for(int markerID : detectedMarkers)
    {
        std::cout << "Requesting ID: " << markerID << "\n";
        // Requesting information to the room info service
        info.request.id = markerID;
        if(!roomInfoPub.call(info))
            return false;
        // Storing the room name
        ontoMapMsg.rooms.push_back(info.response.room);
        // Storing the room positions
        geometry_msgs::Point point;
        point.x = info.response.x;
        point.y = info.response.y;
        ontoMapMsg.positions.push_back(point);
        // Storing the room connections
        final_assignment::RoomDoors roomDoors;
        for(final_assignment::RoomConnection conn : info.response.connections)
            roomDoors.doors.push_back(conn.through_door);
        ontoMapMsg.doors.push_back(roomDoors);
    }
    
    return true;
}



