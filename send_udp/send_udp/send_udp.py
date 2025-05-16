#!/usr/bin/env python
import os
import socket
import json
import requests

import rospy
from sensor_msgs.msg import NavSatFix


# Load environment variables from the current environment only
NAME = os.environ.get("NAME", "3_TER_ARIANE")
UDP_IP = os.environ.get("UDP_IP", "192.168.2.2")
UDP_PORT = int(os.environ.get("UDP_PORT", 32001))
IHM_IP = os.environ.get("IHM_IP", "192.168.2.2")
IHM_PORT = os.environ.get("IHM_PORT", "8008")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def setup_data_udp(data):
    dt = {}
    dt["uav id"] = NAME	
    dt["uav longitude"] = data.longitude
    dt["uav latitude"] = data.latitude
    dt["uav altitude"] = data.altitude
    dt["uav altitude"] = 0
    dt["uav height"] = 0 
    dt["uav speed"] = 0
    dt["uav heading"] = 0
    jdata = json.dumps(dt)
    
    return jdata

def setup_data_rest(data):
    dt = {}
    dt["name"] = NAME
    dt["longitude"] = data.longitude
    dt["latitude"] = data.latitude
    dt["altitude"] = 0
    #dt["speed"] = 0
    #dt[" heading"] = 0
    jdata = json.dumps(dt)
    
    return jdata

def callback_udp(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(data)
    jdata = setup_data_udp(data)
    Message = bytes(jdata)

    sock.sendto(Message, (UDP_IP, UDP_PORT))

    rospy.loginfo(Message)

def callback_rest_api(data):
    jdata = setup_data_rest(data)
    Message = bytes(jdata)

    requests.post(f"http://{IHM_IP}:{IHM_PORT}/api/satellite/update", json=Message)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/fix", NavSatFix, callback_udp)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

