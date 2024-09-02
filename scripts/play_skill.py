#!/usr/bin/env python3
"""
Playback of trajectories and storing them into a databaseself.
"""
from lfd import LfD
import rospy
if __name__ == '__main__':
    name_skills = rospy.get_param('/execute_node/name_skill')
    print("Executing skill: ", name_skills)
    # print("Localize box: ", localize_box)
    lfd = LfD()

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

