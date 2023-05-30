#!/usr/bin/env python3

import json
import yaml

import roslibpy

import rospy
from geometry_msgs.msg import Twist

IP = "10.42.0.1"
PORT = 9091

BASE_TOPIC = "/navigation_velocity_smoother/raw_cmd_vel_"

def main():

    rospy.init_node("cmd_vel_proxy", anonymous=True)
    ws_client = roslibpy.Ros(host=IP, port=PORT)
    ws_client.run()

    publisher = roslibpy.Topic(ws_client, BASE_TOPIC, 'geometry_msgs/Twist')

    def cmd_vel(data):
        y = yaml.safe_load(str(data))
        publisher.publish(roslibpy.Message(y))
        print(y)

    rospy.Subscriber("base/cmd_vel", Twist, cmd_vel)
    rospy.spin()

if __name__ == "__main__":
    main()
