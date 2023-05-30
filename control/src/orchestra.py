import rospy
from bocelli.srv import Request, Listen, Speak
from drake.msg import DrakeResults, DrakeResult

# Mozart will interface with ROS
class Mozart:

    def __init__(self):
        ## Internal Variables
        self.people = []


        ## Setup Subscribers
        self.subscribers = {
            "people_resuts":rospy.Subscriber("/drake/results", DrakeResults, self._onPersonImage)
        }

    ## Updates the Currently visible Person(s)
    def _onPersonImage(self, msg):
        people = []
        for detection in msg.results:
            # Detection is a person
            if detection.object_class == 0:
                people.append(detection)

        self.people = sorted(people, key=lambda x: x.zcentroid)

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

    @staticmethod
    def main(*args, **kwargs):
        rospy.init_node('mozart')
        m = Mozart()

        m.speak_client("This is an orchestrator test")

        rospy.spin()
        
if __name__ == "__main__":
    Mozart.main()
    

    print("why")
    

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