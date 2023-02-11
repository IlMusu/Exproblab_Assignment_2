#!/usr/bin/env python

# Importing AST library for safely evaluating strings
import ast
# Events for synchronizing threads
from threading import Event

# Importing ARMOR libraries to easily interact with ARMOR
from armor_api.armor_client import ArmorClient
from armor_api.armor_utils_client import ArmorUtilsClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_manipulation_client import ArmorManipulationClient

# Importing ROS library for python
import rospy
from geometry_msgs.msg import Point
from robot_state_msgs.srv import RoomPosition, RoomPositionResponse
from robot_state_msgs.srv import ReferenceName, ReferenceNameResponse
from final_assignment.msg import OntologyMap

class OntologyMapBuilder(object):
    '''
    Provides services on :
        - /ontology_map/reference_name (ReferenceName)
        - /ontology_map/room_position (RoomPosition)
        - /ontology_map/map 
    ROS Parameters :
        - /ontology_reference (string) : The reference name of the ontology.
        - /ontology_path (string) : The global path of the default ontology.
        - /ontology_uri (string) : The uri of the ontology.
        - /map (list) : The list of pairs (room, [doors]) for building the map.
        - /room_positions (list) : The list of pairs (room, position) for the rooms positions.
        - /robot_room (string) : The initial room at which the robot is located.
    
    This ROS node loads a default ontology into the ARMOR server and then creates
    the map specified by the user. The description of the map is provided through 
    parameters which are automatically decoded. Once the map if fully created,
    this node provides a service for providing the position of room given the name.
    '''
    def __init__(self) :
        '''
        |  In the constructor method, this class:
        |  1. Creates a service for the "ontology_map/reference_name" service.
        |  2. Creates a service for the "ontology_map/room_position" service.
        |  3. Initializes objects to communicate with ARMOR.
        |  4. Builds the map by calling the related method.
        '''
        # Initializing the ROS node
        rospy.init_node('ontology_map_builder', log_level=rospy.INFO)
        # Creating a Service for the map reference name
        self._map_ready_srv = rospy.Service(
            '/ontology_map/reference_name', ReferenceName, 
            handler = self._onto_reference_service
        )
        # Creating a Service to provide the room positions
        self._rooms_pos_srv = rospy.Service(
            '/ontology_map/room_position', RoomPosition, 
            handler = self._room_position_service
        )
        # Creating a Topic for building the map
        self._build_map_sub = rospy.Subscriber(
            '/ontology_map/build_map', OntologyMap, queue_size=10,
            callback = self._build_ontology_map_from_msg
        )
        # Initializing object to communicate with ARMOR
        self._onto_ref_name = rospy.get_param('~ontology_reference_name', 'ontology_reference')
        self._armor_client = ArmorClient('ontology_map_builder', self._onto_ref_name)
        self._onto_utils = ArmorUtilsClient(self._armor_client)
        self._onto_query = ArmorQueryClient(self._armor_client)
        self._onto_manip = ArmorManipulationClient(self._armor_client)
        # Setting the map as not already built
        self._building_complete = False
        self._building_complete_event = Event()
        # Check if the map needs to be build from params
        self._build_ontology_map_from_params()

        
    def _build_ontology_map_from_params(self):
        # Check if the map must be built from params
        rooms = ast.literal_eval(rospy.get_param('~rooms', 'None'))
        rooms_doors = ast.literal_eval(rospy.get_param('~rooms_doors', 'None'))
        rooms_positions = ast.literal_eval(rospy.get_param('~rooms_positions', 'None'))
        # Logging informations about the parametes
        if rooms == None or rooms_doors == None or rooms_positions == None :
            rospy.loginfo("Map was not found in parameters, waiting for message!")
            return
        rospy.loginfo("Found map from parameters!")
        # The rooms is already a list of strings
        # The rooms_doors is already a list of list of strings
        # The rooms_positions need to be parsed to a list of Point
        rooms_positions = map(lambda pos: Point(pos[0], pos[1], 0), rooms_positions)
        # Building the map
        self._build_ontology_map(rooms, rooms_doors, rooms_positions)
    
    
    def _build_ontology_map_from_msg(self, msg):
        # The rooms is already a list of strings
        # The rooms_positions is already a list of list of Point
        # The rooms_doors need to be parsed to a list of list of strings
        rooms_doors = map(lambda doors: doors.doors, msg.doors)
        # Building the map
        self._build_ontology_map(msg.rooms, rooms_doors, msg.positions)
    
    
    def _build_ontology_map(self, rooms, rooms_doors, rooms_positions):
        # Check if the map is already build
        if self._building_complete :
            rospy.logerr("Map is already built!")
            return
        # Getting the map parameters from the Parameter Server
        onto_path = rospy.get_param('~ontology_path')
        onto_uri = rospy.get_param('~ontology_uri')
        robot_room = rospy.get_param('~robot_room', 'E')
        # Loading the default ontology into ARMOR
        rospy.loginfo("The base ontology is located at: "+onto_path)
        rospy.loginfo("Loading base ontology with uri: "+onto_uri)
        self._onto_utils.load_ref_from_file(onto_path, onto_uri, True)
        # Starting to create map
        rospy.loginfo("Starting to build map.")
        for [room, doors] in zip(rooms, rooms_doors) :
            # Adding rooms with all the related doors
            for door in doors :
                print(room, type(room), door, type(door))
                self._onto_manip.add_objectprop_to_ind('hasDoor', room, door)
                rospy.loginfo("Added door "+door+" to room "+room+".")
            # Setting room properties to initial values
            self._onto_manip.add_dataprop_to_ind('visitedAt', room, 'Long', '0')
        # Disjoint between all the rooms
        self._onto_utils.sync_buffered_reasoner()
        self._onto_manip.disj_inds_of_class('ROOM')
        self._onto_manip.disj_inds_of_class('DOOR')
        # Positioning robot to specified room
        self._onto_manip.add_objectprop_to_ind('isIn', 'Robot1', robot_room)
        rospy.loginfo("Positioned robot in room "+robot_room+".")
        rospy.loginfo("Finished building map.")
        # Organizing room positions
        self._rooms_positions = {}
        for [room, pos] in zip(rooms, rooms_positions):
            self._rooms_positions[room] = pos
        rospy.loginfo("Finished organizing rooms positions.")
        # The building is now complete
        self._building_complete = True
        self._building_complete_event.set()
        
    
    def _onto_reference_service(self, request):
        '''
        Args:
            request (ReferenceNameRequest) : The request from the client.
        Returns:
            (ReferenceNameResponse) : The response to the request.

        This is the callback for the "/ontology_map/reference_name" service.
        It makes the client wait until the the map is complete, then responds
        with the reference name of the ontology.
        '''
        response = ReferenceNameResponse()
        if not self._building_complete :
            self._building_complete_event.wait()
        response.name = self._onto_ref_name
        return response


    def _room_position_service(self, request):
        '''
        Args:
            request (RoomPositionRequest) : The request from the client.
        Returns:
            (RoomPositionResponse) : The response to the request.

        This is the callback for the "/ontology_map/room_position" service.
        It checks that the requested room_name is valid and then fills the
        response with the position for the requested room.
        '''
        # Creating the response object
        response = RoomPositionResponse()
        # Filling the fields of the response
        response.valid = request.room_name in self._rooms_positions
        if response.valid :
            response.position = self._rooms_positions[request.room_name]
        # Returning the response to the client
        return response

        

if __name__ == '__main__' :
    # Creating and starting the map builder
    builder = OntologyMapBuilder()
    # Spinning to prevent quitting
    rospy.spin()

