#!/usr/bin/env python
import os
import socket
import json

#import requests

import rclpy
from rclpy.node import Node
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

# def setup_data_rest(data):
#     dt = {}
#     dt["name"] = NAME
#     dt["longitude"] = data.longitude
#     dt["latitude"] = data.latitude
#     dt["altitude"] = 0
#     #dt["speed"] = 0
#     #dt[" heading"] = 0
#     jdata = json.dumps(dt)
    
#     return jdata

def callback_udp(data):

    jdata = setup_data_udp(data)
    Message = bytes(jdata.encode('utf-8')) 
    sock.sendto(Message, (UDP_IP, UDP_PORT))

    rclpy.logging.get_logger('send_udp').info(Message)

# def callback_rest_api(data):
#     jdata = setup_data_rest(data)
#     Message = bytes(jdata)

#     requests.post(f"http://{IHM_IP}:{IHM_PORT}/api/satellite/update", json=Message)




class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            'fix',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        jdata = setup_data_udp(msg)
        Message = bytes(jdata)
        print(Message)
        sock.sendto(Message, (UDP_IP, UDP_PORT))

        self.get_logger().info(Message)


def main(args=None):
    rclpy.init(args=args)

    my_subscriber = MySubscriber()

    rclpy.spin(my_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

