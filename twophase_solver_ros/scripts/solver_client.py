#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
from twophase_solver_ros.srv import Solver

# import bv_client

# ! # vlt bv-fkt hier callen. dafür keinen parameter übergeben und den CubeDefStr als variable passen
# ! # oder bv-fkt hier definieren

def cube_solver_client(cube):
    rospy.wait_for_service('cube_solver')
    try:
        cube_solver = rospy.ServiceProxy('cube_solver', Solver)
        resp = cube_solver(cube)
        return resp
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
    resp = cube_solver_client(cube)
    if resp.movecount == 0:
        print "\n%s" %(resp.solution)
        pass
    else:
        print "\nsolution is: %s (%s moves)" %(resp.solution, resp.movecount)
        pass
