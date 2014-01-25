#!/usr/bin/env python
"""
Plot points from a trackem_ros stream.

SCL; 19 August 2012.
"""

import roslib; roslib.load_manifest("slamsynth_ros")
import rospy
from trackem_ros.msg import *

import numpy as np
from display import Display


if __name__ == "__main__":
    rospy.init_node("miniviz")
    d = Display(mapsize=[0,3.048,0,3.048]) # in meters
    d.marker = 'o'  # Backwards compatibility until bentobox.cds is updated

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = rospy.wait_for_message("trackem/calpoints", MTCalPoints)
        points = np.array([[p.x, p.y] for p in data.points])
        d.clear()
        d.scatter(points.T[0], points.T[1])
        d.blit()
        rate.sleep()
