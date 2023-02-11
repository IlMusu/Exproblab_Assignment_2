## MESSAGE

# The list of rooms names inside the map
string[] rooms

# This list contains, at index i, the position of room rooms[i]
geometry_msgs/Point[] positions

# This list contains, at index i, the list of doors names
# that are coonected to room rooms[i]
RoomDoors[] doors
