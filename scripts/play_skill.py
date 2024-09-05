#!/usr/bin/env python3
"""
Playback of trajectories and storing them into a databaseself.
"""
from panda_ros.pose_transform_functions import pose_2_transformation
from lfd import LfD
import rospy

from platonics_vision.srv import IterativeRegistrationLocalizer, IterativeRegistrationLocalizerRequest, IterativeRegistrationLocalizerResponse
if __name__ == '__main__':
    rospy.init_node("play_skill_node")
    name_skills = rospy.get_param('/execute_node/name_skill')
    print("Executing skill: ", name_skills)
    # print("Localize box: ", localize_box)
    lfd = LfD()
    localize_box = True
    if localize_box:
        rospy.wait_for_service('iterative_sift_localizer')
        active_localizer = rospy.ServiceProxy('iterative_sift_localizer', IterativeRegistrationLocalizer)
        request = IterativeRegistrationLocalizerRequest()
        request.steps.data = 2
        resp: IterativeRegistrationLocalizerResponse = active_localizer(request)
        lfd.localization_transform = pose_2_transformation(resp.pose)

    lfd.load(name_skills)
    lfd.execute() #

    lfd.buttons.desk._listening = False
    # save = None

    # while not (save in [0,1]):
    #     print("SAVE CURRENT RECORDING? 0 = NO, 1 = YES")
    #     try:
    #         save = int(input('\n'))
    #     except:
    #         print("INVALID INPUT")
    # if save:
    #     lfd.save(name_skills)

