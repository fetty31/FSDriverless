#!/usr/bin/env python3

"""
start.py: Track visualization publisher.
This script is in charge of reading and publishing the chosen FS track, as well as updating the cones positions if moved
"""

__author__      = "Oriol Martínez @fetty31"
__maintainer__  = "Oriol Martínez @fetty31"
__email__       = "oriol.martinez.fite@gmail.com"
__credits__     = "BCN eMotorsport"

import rospy, yaml, time
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray

markerArray = MarkerArray()

def loadTrack(daes: bool):
    global cones
    try:
        path = rospy.get_param("/tracker_start/path", "") # absolute file path
        with open(path, "r") as file:
            cones = yaml.load(file, Loader=yaml.FullLoader)
        fillMarker(daes) # fill marker array
        return True
    except FileNotFoundError:
        rospy.logerr("TRACKER: Track file not found")
        print("Check whether the file exists")
        return False
    except:
        rospy.logerr("TRACKER: Something went wrong while reading track file")
        print("Check whether the file exists and it has the correct format")
        return False

def callback(data):
    if data.layout.dim[0].label == "reset":
        loadTrack(rospy.get_param("/tracker_start/cone_daes", False))
        return
    for cone in markerArray.markers:
        for i in range(len(data.data)):
            if cone.id == data.data[i]:
                cone.pose.position.x = data.data[i+1]
                cone.pose.position.y = data.data[i+2]
                i += 3

def standardMarker(daes: bool, color: int):

    # Shared args
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.lifetime = rospy.Duration(0)

    marker.header.frame_id = "global"
    marker.action = marker.ADD

    marker.pose.orientation.w = 1.0
    marker.color.a = 1.0

    marker.type = marker.MESH_RESOURCE
    marker.scale.x = 2.0
    marker.scale.y = 2.0
    marker.scale.z = 2.0

    if daes: # we have as_visualization pkg, so we can load dae files 
        marker.type = marker.MESH_RESOURCE
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        if color == 0:
            marker.mesh_resource = "package://visualizer/mesh/cone_yellow.dae"
        elif color == 1:
            marker.mesh_resource = "package://visualizer/mesh/cone_blue.dae"
        elif color == 2:
            marker.mesh_resource = "package://visualizer/mesh/cone_orange.dae"
        elif color == 3:
            marker.mesh_resource = "package://visualizer/mesh/cone_orange_big.dae"

    else:
        marker.type = marker.CYLINDER
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 1.0

    return marker
        

def fillMarker(daes: bool):

    idx = 0 # index for each cone

    markerArray.markers.clear() # clear array

    # Blue Cones (== 1) 
    for cone in cones['cones_left']:
        marker = standardMarker(daes, 1) # shared args for all markers
        marker.ns = "cone_blue"
        marker.pose.position.x = cone[0]
        marker.pose.position.y = cone[1]
        marker.pose.position.z = 0.325
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.id = idx
        idx += 1
        markerArray.markers.append(marker)
    
    # Yellow Cones (== 0)
    for cone in cones['cones_right']:
        marker = standardMarker(daes, 0) # shared args for all markers
        marker.ns = "cone_yellow"
        marker.pose.position.x = cone[0]
        marker.pose.position.y = cone[1]
        marker.pose.position.z = 0.325
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.id = idx
        idx += 1
        markerArray.markers.append(marker)
    
    # Orange Cones (== 2)
    for cone in cones['cones_orange']:
        marker = standardMarker(daes, 2) # shared args for all markers     
        marker.ns = "cone_orange"
        marker.pose.position.x = cone[0]
        marker.pose.position.y = cone[1]
        marker.pose.position.z = 0.325
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.id = idx
        idx += 1
        markerArray.markers.append(marker)
    
    # Big Orange Cones (== 3)
    for cone in cones['cones_orange_big']:
        marker = standardMarker(daes, 3) # shared args for all markers
        marker.ns = "cone_orange_big"
        marker.pose.position.x = cone[0]
        marker.pose.position.y = cone[1]
        marker.pose.position.z = 0.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.id = idx
        idx += 1
        markerArray.markers.append(marker)
    
def node(topic: str):

    # Init ROS node
    rospy.init_node("tracker_start", anonymous=False)

    # Define publisher
    pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
    sub = rospy.Subscriber("/carmaker/moved", Float32MultiArray, callback)

    cone_daes = rospy.get_param("/tracker_start/cone_daes", False) # whether we use cones dae files or not

    # Loop
    start = loadTrack(cone_daes)
    rate = rospy.Rate(rospy.get_param("/tracker_start/freq", 1.0))
    while not rospy.is_shutdown():
        try:
            if start:
                pub.publish(markerArray) # publish cone markers
            else:
                start = loadTrack(cone_daes)
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: # rate.sleep() raises this error when restarting simulation (rospy es una pala)
            pass

if __name__ == '__main__':
    try:
        node("/carmaker/track")
    except rospy.ROSInterruptException:
        pass

