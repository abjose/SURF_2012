#!/usr/bin/env python
"""
SCL; 19 August 2012.
"""

import roslib; roslib.load_manifest("slamsynth_ros")
import rospy
from trackem_ros.msg import *

import numpy as np
import numpy.linalg as la


class Robot:
    def __init__(self):
        self.x, self.y, self.theta = None, None, None

    def est_orientation(self, points):
        """Rough estimation of orientation from three points

        Given three points in the plane, find the two that are
        mutually closest, take their mean as the base point, and then
        find angle to the third point (farthest from the cluster).
        """
        # Compute by hand, generalize later if needed.
        min_dist = la.norm(points[0]-points[1])
        min_idx = [0,1]
        if la.norm(points[0]-points[2]) < min_dist:
            min_dist = la.norm(points[0]-points[2])
            min_idx = [0,2]
        if la.norm(points[1]-points[2]) < min_dist:
            min_dist = la.norm(points[1]-points[2])
            min_idx = [1,2]
        base = np.mean(points[min_idx,:],axis=0)
        for other_idx in range(3):
            if other_idx not in min_idx:
                break
        return np.arctan2(points[other_idx][1]-base[1], points[other_idx][0]-base[0])

    def update_pose(self, data):
        points = np.array([[p.x, p.y] for p in data.points])
        if points.shape[0] > 0:
            self.x, self.y = points[0]
            self.theta = self.est_orientation(points)

    def get_pose(self, blocking=False):
        """Return pose, optionally blocking until one is available.
        """
        if self.x is None or self.y is None or self.theta is None:
            if not blocking:
                raise ValueError("requested pose is undefined.")
            else:
                while self.x is None or self.y is None or self.theta is None:
                    pass
        return (self.x, self.y, self.theta)


if __name__ == "__main__":
    #waypoints = np.array([[1.,1.],[1.5,2.5]])
    robot = Robot()

    rospy.init_node("demo")
    tsub = rospy.Subscriber("trackem/calpoints", MTCalPoints,
                            callback=robot.update_pose)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print robot.get_pose(blocking=True)
        rate.sleep()
