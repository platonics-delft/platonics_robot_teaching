#!/usr/bin/env python3
"""
Playback of trajectories and storing them into a databaseself.
"""
from lfd import LfD
from std_srvs.srv import Trigger
import sys
import os
import rospy
import numpy as np
from platonics_vision.srv import IterativeRegistrationLocalizer, IterativeRegistrationLocalizerRequest, IterativeRegistrationLocalizerResponse

from panda_ros.pose_transform_functions import pose_2_transformation
if __name__ == '__main__':
    rospy.init_node("play_all_skills_node")
    localize_box = rospy.get_param('/execute_node/localize_box')
    print("Localize box: ", localize_box)
    lfd = LfD()

    if localize_box:
        rospy.wait_for_service('iterative_sift_localizer')
        active_localizer = rospy.ServiceProxy('iterative_sift_localizer', IterativeRegistrationLocalizer)
        request = IterativeRegistrationLocalizerRequest()
        request.steps.data = 2
        resp: IterativeRegistrationLocalizerResponse = active_localizer(request)
        lfd.localization_transform = pose_2_transformation(resp.pose)
    try:

        lfd.load("button_blue")
        lfd.execute()
        
        lfd.load("peg_pick")
        lfd.execute()

        lfd.load("peg_door")
        lfd.execute()

        lfd.load("peg_place")
        lfd.execute()

        lfd.camera_correction = np.array([0,0,0]) # reset camera corrections
        lfd.load("probe_pick")
        lfd.execute()
        lfd.camera_correction = np.array([0,0,0]) # reset camera corrections
        lfd.load("probe_probe")
        lfd.execute()
        
        lfd.camera_correction = np.array([0,0,0]) # reset camera corrections
        lfd.load("probe_place")
        lfd.execute(retry_insertion_flag=1)

        lfd.camera_correction = np.array([0,0,0]) # reset camera corrections
        lfd.load("wrap")
        lfd.execute()

    except rospy.ROSInterruptException:
        pass



