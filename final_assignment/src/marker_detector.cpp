#include <ros/ros.h>
#include <aruco/aruco.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <final_assignment/RoomInformation.h>
#include <final_assignment/RoomConnection.h>
#include <final_assignment/RoomDoors.h>
#include <final_assignment/OntologyMap.h>
#include <final_assignment/RobotInspectionRoutineAction.h>

aruco::CameraParameters defaultCameraParams;
std::set<int> detectedMarkers;
std::set<int> validatedMarkers;
final_assignment::OntologyMap ontoMapMsg;

ros::ServiceClient roomInfoSrv;
ros::Subscriber imageSub;
ros::Publisher ontoMapPub;
ros::Publisher velPub;

bool retrieveRoomsInformation();
void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    
    
int main(int argc, char **argv)
{
    // Initializing the ROS Node
    ros::init(argc, argv, "marker_detector");
    ros::NodeHandle n;

    // Initializing variables
    defaultCameraParams = aruco::CameraParameters();
    
    // Creating a Subscriber for the image topic
    imageSub = n.subscribe("/camera/image_raw", 100, imageCallback);
    // Creating a Publisher for the robot velocity
    velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000, true);
    // Creating a Publisher for the ontology map
    ontoMapPub = n.advertise<final_assignment::OntologyMap>("/ontology_map/build_map", 100, true);
    // Creating a ServiceClient for the rooms information
    roomInfoSrv = n.serviceClient<final_assignment::RoomInformation>("/room_info");
    
    // Creatin a ActionClient for moving the robot arm aroung
    // True causes the client to spin its own thread
    actionlib::SimpleActionClient<final_assignment::RobotInspectionRoutineAction> armAcl("robot_inspection_routine", true);
    
    ROS_INFO("[MARKER DETECTOR] Waiting for InspectionRoutine Server...");
    armAcl.waitForServer();
    ROS_INFO("[MARKER DETECTOR] InspectionRoutine Server found!");
    ROS_INFO("[MARKER DETECTOR] Waiting for MarkerServer Service...");
    roomInfoSrv.waitForExistence();
    ROS_INFO("[MARKER DETECTOR] MarkerServer Service found!");
    
    // Setting the robot velocity very small in order to keep it still
    geometry_msgs::Twist velMsg;
    velMsg.linear.x = 0.001;
    velPub.publish(velMsg);
     
    // Finding all the markers in the room
    do
    {
        // Creating the goal for the movement of the arm
        final_assignment::RobotInspectionRoutineGoal goal;
        armAcl.sendGoal(goal);
    
        // Waiting for the arm movement to complete
        // In the meanwhile, handle callbacks
        ros::Rate rate(10); // 10 hz
        while(!armAcl.getState().isDone())
        {
            ros::spinOnce();
            rate.sleep();
        }
        
        // If the InspectionRoutine was not executed successfully,
        // wait for a bit and then retry
        if(!armAcl.getResult()->completed)
        {
            ROS_INFO("[MARKER DETECTOR] Inspection routine was not executed!");
            ROS_INFO("[MARKER DETECTOR] Waiting for a bit and then retrying!");
            ros::Duration(10).sleep();
            continue;
        }
        
        // Retrieving all the rooms information
        retrieveRoomsInformation();
        ROS_INFO("[MARKER DETECTOR] Validated '%li' markers!", validatedMarkers.size());
        
    }
    while(validatedMarkers.size() < 7);
    
    // Stopping the now unnecessary subscriber for the camera image
    imageSub.shutdown();
    
    // Publishin the ontology map
    ontoMapPub.publish(ontoMapMsg);
    
    // Spinning to prevent stopping the execution
    ros::spin();

    return 0;
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
    
    // Check if a new marker id was detected
    std::pair<std::set<int>::iterator,bool> ret;
    for (std::size_t i = 0; i < markers.size(); ++i)
    {        
        int id = markers.at(i).id;
        // Check if the marker was already validated
        if(validatedMarkers.find(id) != validatedMarkers.end())
            continue;
        // Check if marker was already detected
        if(detectedMarkers.find(id) != detectedMarkers.end())
            continue;
        
        // A new marker has been found
        detectedMarkers.insert(id);
        ROS_INFO("[MARKER DETECTOR] New marker with id '%i' detected!", id);
    }
}


bool retrieveRoomsInformation()
{
    final_assignment::RoomInformation info;
    for(int markerID : detectedMarkers)
    {
        ROS_INFO("[MARKER DETECTOR] Requesting info for ID %i", markerID);
        // Requesting information to the room info service
        info.request.id = markerID;
        if(!roomInfoSrv.call(info))
            return false;
            
        // Check if the detected marker is valid
        if(info.response.room == "no room associated with this marker id")
        {
            ROS_INFO("[MARKER DETECTOR] Detected invalid ID %i, ignoring", markerID);
            continue;
        }
        
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
        
        // The marker has been parsed as valid
        validatedMarkers.insert(markerID);
    }
    
    // The detected markers have been handled
    detectedMarkers.clear();
    
    return true;
}

