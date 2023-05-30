#!/usr/bin/env python3

import json
import socket

import roslibpy
import roslibpy.actionlib

import rospy
import rospkg

from happynav.srv import MoveTo

IP = "10.42.0.1"
PORT = 9091

def amy_cmd_proxy(cmd):
    with socket.create_connection((IP, 12306)) as sock:
        sock.send(cmd.encode("ASCII"))
        return sock.recv(4096).decode("ASCII")

def main():

    r = amy_cmd_proxy("#NAVS")
    print(f"Open navigation response: {r}", flush=True)

    rospack = rospkg.RosPack()
    d = rospack.get_path("happynav")
    posraw = {}
    POSITIONS = {}
    with open(f"{d}/src/locations.json") as f:
      posraw = json.load(f)
    for val in posraw:
        POSITIONS[val["name"]] = val["location"]

    rospy.init_node("mover", anonymous=True)
    ws_client = roslibpy.Ros(host=IP, port=PORT)
    ws_client.run()

    action_client = roslibpy.actionlib.ActionClient(
        ws_client, "/move_base", "move_base_msgs/MoveBaseAction"
    )



    #pub = rospy.Publisher("base/move_to_result", String, queue_size=10)

    def move_to(args):
        dest = args.dest
        print(args.dest, flush=True)
        if not dest in POSITIONS:
            return "location_not_found"
        msg = {
            "target_pose": {
                "header": {"frame_id": "/map"},
                "pose": POSITIONS[dest],
            }
        }
        goal = roslibpy.actionlib.Goal(
            action_client,
            roslibpy.Message(msg),
        )
        #goal.on("result", lambda f: pub.publish(str(f)))
        goal.send()
        resp = goal.wait()
        return "arrived"

    rospy.Service("base/move_to", MoveTo, move_to)

    #rospy.Subscriber("base/move_to", String, move_to)

    rospy.spin()

if __name__ == "__main__":
    main()
