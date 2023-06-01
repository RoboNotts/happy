#!/usr/bin/env python3

import math

import rospy
from time import sleep
from geometry_msgs.msg import Twist

from roslibpy.tf import TFClient
import roslibpy


from drake.msg import DrakeResults

IP = "10.42.0.1"
PORT = 9091

def z_from_quaternion(x, y, z, w):
    t1 = +2.0 * (w * z + x * y)
    t2 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t1, t2)

def main():

    cmd_vel = rospy.Publisher("/base/cmd_vel", Twist, queue_size=10)
    twist = Twist()
    twist.angular.z = (math.pi / 3) * (480/640) #Third of a pi

    rospy.init_node("locate_person", anonymous=True)

    ws_client = roslibpy.Ros(host=IP, port=PORT)
    ws_client.run()

    tfc = TFClient(ws_client, fixed_frame="/map", angular_threshold=0.01, translation_threshold=0.01)

    rate = rospy.Rate(10)

    current_tf = {"current": None}
    def r(x):
        if x != None:
            current_tf["current"] = x
    tfc.subscribe("/base_link", r)
    start_tf = None
    while not rospy.is_shutdown():
        if current_tf["current"] != None:
            start_tf = current_tf["current"]
            break
        rate.sleep()
    start_rot = z_from_quaternion(
                                    start_tf["rotation"]["x"],
                                    start_tf["rotation"]["y"],
                                    start_tf["rotation"]["z"],
                                    start_tf["rotation"]["w"])

    def find_person(args):

        vs = {
            "passed": False,
            "last_rot": start_rot,
            "person": None,
            "final_rot": None,
            "sub": None,
        }

        def on_person_image(msg):
            ctf = current_tf["current"]["rotation"]
            cur_rot = z_from_quaternion(ctf["x"], ctf["y"], ctf["z"], ctf["w"])
            for detection in msg.results:
                if detection.object_class != 0:
                    continue
                if detection.ycentroid < 300:
                    continue
                if not 80 < detection.xcentroid < 560:
                    continue
                if vs["person"] == None:
                    vs["person"] = detection
                    vs["final_rot"] = cur_rot
                elif detection.ycentroid > vs["person"].ycentroid and detection.zcentroid < 5000:
                    vs["person"] = detection
                    vs["final_rot"] = cur_rot


            if not vs["passed"] and cur_rot < vs["last_rot"]:
                vs["passed"] = True
            elif vs["passed"] and cur_rot > start_rot:
                print("Rotation complete")
                vs["sub"].unregister()

                final_loc = None
                final_rot = vs["final_rot"]
                print(f"Selected rotation: {final_rot}")

                if final_rot <= 1.216 and final_rot > -0.7:
                    final_loc = "reading_0"
                elif final_rot <= -0.7 and final_rot >= -0.21:
                    final_loc = "living_room"
                elif final_rot <= -0.21 and final_rot >= -2.869:
                    final_loc = "dining_table_0"
                else:
                    final_loc = "hall"

                print(f"Selected location: {final_loc}")

                return
            cmd_vel.publish(twist)
            sleep(0.8)


        vs["sub"] = rospy.Subscriber("/drake/results", DrakeResults, on_person_image)

        """
        final_rot = z_from_quaternion(rotation["x"], rotation["y"], rotation["z"], rotation["w"])
        final_loc = None
        print(f"Selected rotation: {final_rot}")
        if final_rot <= 1.216 and final_rot > -0.7:
            final_loc = "reading_0"
        elif final_rot <= -0.7 and final_rot >= -0.21:
            final_loc = "living_room"
        elif final_rot <= -0.21 and final_rot >= -2.869:
            final_loc = "dining_table_0"
        else:
            final_loc = "hall"

        print(person)
        print(f"Selected location: {final_loc}")
        sub.unregister()
        return final_loc
        """

    find_person(None)

    rospy.spin()

if __name__ == "__main__":
    main()
