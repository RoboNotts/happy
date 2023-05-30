import rospy

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