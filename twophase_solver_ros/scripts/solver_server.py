#!/usr/bin/env python
# coding: utf-8


import sys # debug
import rospy
from solver import solve
from twophase_solver_ros.srv import Solver, SolverResponse
from std_msgs.msg import String


def handle_solve(req):
    #twophase algorithm here
    # maybe catch wrong input
    solStr = solver.solve(req)
    print "found solution: %s for cube %s"%(solStr, req.defstr)
    length = len(solStr)	
    thresh = len(solStr)-2
    solution = solStr[thresh:length]
    movecount = solStr[:thresh]
    return SolverResponse(movecount,solution)

def cube_solve_server():
    rospy.init_node('cube_solver_server')
    s = rospy.Service('cube_solver', Solver, handle_solve)
    print "Ready to solve a cube."
    rospy.spin()

if __name__ == "__main__":
    cube_solve_server()
    print(sys.version)
