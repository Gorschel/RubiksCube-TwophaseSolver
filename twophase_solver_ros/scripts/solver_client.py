#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
from twophase_solver_ros.srv import Solver

def cube_solver_client(cube):
    rospy.wait_for_service('cube_solver')
    try:
        cube_solver = rospy.ServiceProxy('cube_solver', Solver)
        #BV: maybe call python script with params
        resp1 = cube_solver(cube)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [cube definition string]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
	cube = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    print "Requesting solution for cube definition string: %s"%(cube)
    resp1 = cube_solver_client(cube)
    print "solution is: %s (%s moves)" %(resp1.solution, resp1.movecount)
