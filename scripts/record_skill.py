#!/usr/bin/env python3
"""
Recording trajectories and storing them into a databaseself.
"""
from lfd import LfD
import rospy

if __name__ == '__main__':
    try:
        name_skill = rospy.get_param('/recording_node/name_skill')
        print("Recording skill: ", name_skill)
        lfd = LfD()
        lfd.kinesthetic_teaching()
        lfd.save(name_skill)
        lfd.buttons.desk._listening = False
    except rospy.ROSInterruptException:
        pass

