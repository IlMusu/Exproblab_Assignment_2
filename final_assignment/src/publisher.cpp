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
final_assignment::OntologyMap ontoMapMsg;

ros::ServiceClient roomInfoSrv;
ros::Subscriber imageSub;
ros::Publisher ontoMapPub;
ros::Publisher velPub;
ros::Publisher joint1Pub;
ros::Publisher joint2Pub;
ros::Publisher joint3Pub;

void resetArmPosition();
bool retrieveRoomsInformation();
void rotateArmAroundInSteps();
void imageCallback(const sensor_msgs::Image::ConstPtr& msg);


template <typename T> int sign(T val) 
{
    return (T(0) < val) - (val < T(0));
}
    
    
int main(int argc, char **argv)
{
    // Initializing the ROS Node
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle n;

    // Initializing variables
    defaultCameraParams = aruco::CameraParameters();
    
    // Creating a Subscriber for the image topic
    imageSub = n.subscribe("/camera/image_raw", 100, imageCallback);
    // Creating a Publisher for the robot velocity
    velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000, true);
    // Creating Publishers for the robot arm
    joint1Pub = n.advertise<std_msgs::Float64>("/joint1_position_controller/command", 1000, true);
    joint2Pub = n.advertise<std_msgs::Float64>("/joint2_position_controller/command", 1000, true);
    joint3Pub = n.advertise<std_msgs::Float64>("/joint3_position_controller/command", 1000, true);
    // Creating a Publisher for the ontology map
    ontoMapPub = n.advertise<final_assignment::OntologyMap>("/ontology_map/build_map", 100, true);
    // Creating a ServiceClient for the rooms information
    roomInfoSrv = n.serviceClient<final_assignment::RoomInformation>("/room_info");
    
    // Resetting the arm position
    resetArmPosition();
    
    // Rotating the arm around to search for aruco markers
    while(detectedMarkers.size() < 7)
    {
        rotateArmAroundInSteps();
        ROS_INFO("Found %li markers!", detectedMarkers.size());
    }
    
    // Resetting the arm position after rotating
    resetArmPosition();
    // Retrieving all the rooms information
    retrieveRoomsInformation();
    // Publishin the ontology map
    ontoMapPub.publish(ontoMapMsg);
    
    // Spinning to prevent stopping the execution
    ros::spin();

    return 0;
}


void resetArmPosition()
{
    // Creating the message for comunicating with the whole arm
    std_msgs::Float64 armMsg;
    armMsg.data = 0.0;
    // Resetting the arm position
    joint1Pub.publish(armMsg);
    joint2Pub.publish(armMsg);
    joint3Pub.publish(armMsg);
}


void rotateArmAroundInSteps()
{
    // Setting the robot velocity very small in order to keep it still
    geometry_msgs::Twist velMsg;
    velMsg.linear.x = 0.001;
    velPub.publish(velMsg);
    
    // Logging
    ROS_INFO("Starting to rotate the arm!");
    
    // Creating the message for comunicating with the whole arm
    std_msgs::Float64 armMsg;
    
    // Handling the yaw rotation in a smarter yaw
    float yaw = -3.10F;
    float targetYaw = 3.10F;
    float deltaYaw = 0.1F;
    
    // Actually starting to rotate the arm
    ros::Rate updateRate(5);    
    for(float pitch = 0.4F; pitch >= -0.6F; pitch -= 0.5F)
    {
        // Rotating the camera pitch
        armMsg.data = pitch;
        joint3Pub.publish(armMsg);
        ROS_INFO("Setting new pitch: %f", pitch);
        
        float deltaYawSign = sign(deltaYaw);
        while(yaw*deltaYawSign < targetYaw*deltaYawSign)
        {
            // Rotating the camera yaw
            armMsg.data = yaw;
            joint1Pub.publish(armMsg);
            ROS_INFO("Setting new yaw: %f .", yaw);
            
            ros::spinOnce();
            updateRate.sleep();
            
            yaw += deltaYaw;
        }
        
        // Inverting the target yaw and delta
        targetYaw *= -1;
        deltaYaw *= -1;
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
    
    // Check if a new aruco ID was detected
    std::pair<std::set<int>::iterator,bool> ret;
    for (std::size_t i = 0; i < markers.size(); ++i)
    {
        int id = markers.at(i).id;
        ret = detectedMarkers.insert(id);
        if(!ret.second)
            continue;
        // A new marker has been found
        ROS_INFO("New marker with id '%i' detected!", id);
    }
}


bool retrieveRoomsInformation()
{
    final_assignment::RoomInformation info;
    for(int markerID : detectedMarkers)
    {
        ROS_INFO("Requesting info for ID %i", markerID);
        // Requesting information to the room info service
        info.request.id = markerID;
        if(!roomInfoSrv.call(info))
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

