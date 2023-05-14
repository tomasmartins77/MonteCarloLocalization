import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

def map_callback(msg):
    # Convert the 1D array to a 2D numpy array
    width = msg.info.width
    height = msg.info.height
    map_array = np.array(msg.data).reshape((height, width))

    # Boolean indexing to select all non-negative values
    mask = map_array != -1

    # Use np.delete() function to remove all -1 values
    map_array = np.delete(map_array, np.where(~mask)[0])
    print(map_array)



# Initialize the ROS node
rospy.init_node('map_subscriber')

# Subscribe to the "/map" topic
rospy.Subscriber("/map", OccupancyGrid, map_callback)

# Spin the node to receive messages
rospy.spin()