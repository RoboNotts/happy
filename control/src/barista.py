#HackNotts Barista main control node

import rospy
from bocelli.srv import Request, Listen, Speak, ListenResponse, SpeakResponse, RequestResponse
from drake.msg import DrakeResults, DrakeResult
from art_detection.msg import ArtResult, ArtResults, CameraPoint
from slam.srv import MoveTo, Locate
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from time import sleep
from random import choice
from tf.transformations import euler_from_quaternion

# Kaldi will interface with ROS
class Kaldi:
    
    
    pass

# General Procedure
# Starting at BARISTA location with empty COFFEEQ
# 0. Confirm starting location is BARISTA
# # a. Look for barista AR tag. If not visible, announce this.
# 1. Wait for COFFEEQ to be non-empty
# 2. Wait for barista to place coffee in pick-up area
# # a. Use CV to spot AR tag in the BARISTA location
# # # i. Barista AR tag is undetectable for > 5 seconds
# 3. Pick up coffee
# # a. Announce that a pick up is to be attempted
# # b. Give time for barista to issue cancel command
# # c. If no cancel command is provided, attempt pick-up
# 4. Deliver Coffee
# # a. Pop top value of COFFEEQ to determine customer name and location
# # b. Use navigation stack to travel to the correct location
# # c. Once at correct location, look for corresponding AR tag
# # # I. If not found, announce that it cannot see the delivery location, and to make sure it is clear
# # # II. If still not found after two reminders, send distress signal
# # d. Announce to customer that coffee is to be delivered, and to keep away
# 5. Drop Coffee at location
# # a. Place coffee down
# # b. Thank customer
# 6. Return to Barista location, and repeat