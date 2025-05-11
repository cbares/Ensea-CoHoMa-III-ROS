#!/usr/bin/env python
import socket
import json

import rospy
from sensor_msgs.msg import NavSatFix
    NAME = "2_TER_PETIT-POUCET"
    UDP_IP = "192.168.2.213" #debug
    UDP_PORT = 32001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(data)    
    dt = {}
    dt["uav id"] = NAME	
    dt["uav longitude"] = data.longitude
    dt["uav latitude"] = data.latitude
    dt["uav altitude"] = data.altitude
    dt["uav speed"] = 0
    dt["uav heading"] = 0
    jdata = json.dumps(dt)
    Message = bytes(jdata)
    
	
    sock.sendto(Message, (UDP_IP, UDP_PORT))

    rospy.loginfo(Message)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/fix", NavSatFix, callback)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

