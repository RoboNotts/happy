import rospy
from bocelli.srv import Request, Listen, Speak, ListenResponse, SpeakResponse, RequestResponse
from drake.msg import DrakeResults, DrakeResult
from slam.srv import MoveTo
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from time import sleep
from tf.transformations import euler_from_quaternion

LOCATIONS = [
    "reading room",
    "living room",
    "dining table",
    "hall"
]

LOCATION_WAYPOINTS = [
    "reading_0",
    "living_room",
    "dining_table_0",
    "hall"
]

OBJECTS = {
    "chips_can": ["crisps", "pringles"],
    "tomato_soup_can": ["soup"],
    "softball" : ["softball"],
    "potted_meat_can": ["spam", "meat"],
    "windex_bottle" : ["windex", "cleaner"],
    "spatula": ["spatula"],
    "pitcher_base": ["jug", "pitcher"],
}


# Mozart will interface with ROS
class Mozart:

    def __init__(self):
        ## Internal Overarching State
        # HUNTING   - Finding POI
        # SPEAKING  - Extracting Command 
        # SEARCHING - Looking for object
        # RETURN    - Returning to person

        self.state = "INIT"

        ## Internal Variables
        # Pose
        self.x = 0
        self.y = 0
        self.theta = 0

        # Detections
        self.people = []


        ## Setup Subscribers
        self.subscribers = {
            "people_resuts":rospy.Subscriber("/drake/results", DrakeResults, self._onPersonImage),
            "pose":rospy.Subscriber("/odom", Odometry, self._onOdom)
        }

        ## Setup Publishers
        self.publishers = {
            "manual_move":rospy.Publisher("/base/cmd_vel", Twist),
        }

    # This function runs periodically
    # State Machine!
    def act(self):
        if self.state == "INIT":

            #Start
            self.speak_client("Hello! I'm happy, Let's help some people!")
            
            
            # Find person

            # Go to person

            #self.waypoint_client("living_room")
            #self.speak_client("I'm looking for you!")

            

            # Talk to person
            self.speak_client("Hello, how can I help you today? Please speak clearly")

            while True:
                waypoint = ""
                obj = ""
                args = ["",""]

                result = self.listen_client().result
                userIn = str(self.dialogFlow_client(result).result).lower()
                print(f"m:{result}")
                print(f"df:{userIn}")
                result = str(result).lower()
                # First try dialogflow inferrence
                try: 
                    command = userIn.split(":")[0]
                    if command == "FETCH":
                        args = userIn.split(":")[1].split(",")
                        print(f"c:{command}")
                        print(f"a:{args}")
                        waypoint = LOCATION_WAYPOINTS[LOCATIONS.index(args[1])]
                        obj = args[0]
                    else:
                        raise ValueError

                except: # Otherwise try manual inferrence
                    for i,v in enumerate(LOCATIONS):
                        if v in result:
                            waypoint = LOCATION_WAYPOINTS[i]
                            args[1] = v
                    for o in OBJECTS:
                        for v in o:
                            if v in result:
                                obj = o
                                args[0] = v

                if obj == "" or waypoint == "":
                    self.speak_client("Repeat that for me please...")
                    
                else:
                    break

            self.speak_client(f"Alright I will {command} the {args[0]} from the {args[1]}")
            print(obj, waypoint)

            
            
            

    ## Updates the current position of the robot
    def _onOdom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # from pose get quarterion and do euler's transofrm to get 3D vector to extract yaw 
        sep = msg.pose.pose.orientation
        orientation_list = [sep.x, sep.y, sep.z, sep.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        self.theta = yaw


    ## Updates the Currently visible Person(s)
    def _onPersonImage(self, msg):
        people = []
        for detection in msg.results:
            # Detection is a person
            if detection.object_class == 0:
                people.append(detection)

        # Sorted by distance
        self.people = sorted(people, key=lambda x: x.zcentroid)

    # Client for waypointing
    def waypoint_client(self, waypoint):
        print("Hmmm")
        rospy.wait_for_service("base/move_to")
        print("Got it!")
        try:
            diaf = rospy.ServiceProxy('base/move_to', MoveTo)
            response = diaf(waypoint)
            return response
        except rospy.ServiceException as e:
            print(f"Service Call Failed: {e}")

    # Client for dialogFlow
    def dialogFlow_client(self, text):
        rospy.wait_for_service('df_request')
        try:
            diaf = rospy.ServiceProxy('df_request', Request)
            response = diaf(text)
            return response
        except rospy.ServiceException as e:
            print(f"Service Call Failed: {e}")
        
    # Client for speaking
    def speak_client(self, text):
        rospy.wait_for_service('speak')
        try:
            speak = rospy.ServiceProxy('speak', Speak)
            response = speak(text)
        except rospy.ServiceException as e:
            print(f"Service Call Failed: {e}")
    
    # Client for listening
    def listen_client(self):
        rospy.wait_for_service('listen')
        try:
            listen = rospy.ServiceProxy('listen', Listen)
            response = listen(5)
            return response
        except rospy.ServiceException as e:
            print(f"Service Call Failed: {e}")
        

if __name__ == "__main__":
    rospy.init_node("mozart")
    m = Mozart()
    r = rospy.Rate(5)
    m.act()
    #while not rospy.is_shutdown():
    #    m.act() # DO THINGS!!!
    #    r.sleep()
    

# General Procedure
# 1. Recieve Start Command
# 2. Locate Subject
# # a. Traverse to 'central' waypoint
# # b. Identify Person using Drake
# # c. Estimate position of person
# # d. Move towards person(s)
# 3. Ask Subject for Command
# # a. Interrupt subject in socially acceptable way
# # b. Ask subject for a command
# # c. Parse command using DialogFlow
# # d. If command cannt be parsed, go to 3b.
# 4. Locate OOI
# # a. Go to first waypoint of location
# # b. Execute 'Search' Procedure
# # # i.  Rotate while assesing model?
# # # ii. Infer 3D object pose
# # c. If object not found repeat from 4a with next waypoint
# # d. If object not found still, reutrn to subject and announce failure
# 5. Retrieve OOI
# # a. Use 3D-Object coordinates to manipulate object
# # b. Hold Object
# 6. Return to POI
# # a. Use position found in 2c to return to POI
# # b. Announce to POI that handover is about to occur
# # c. Undergo Handover
# 7. Return to Initial position