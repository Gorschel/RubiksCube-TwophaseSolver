#!/usr/bin/env python
# coding: utf-8

import rospy

from solver import solve # actual twophase solver algorithm
from twophase_solver_ros.srv import Solver, SolverResponse
from std_msgs.msg import String

def check_solution(solStr):
    """check for errors and split movecount from solution string"""
    if solStr == 'Error: Some edges are undefined.' :
        solution = solStr
        movecount = 0
        print solStr
        pass
    elif solStr == 'Error: Total edge flip is wrong.' :
        solution = solStr
        movecount = 0
        print solStr
        pass
    elif solStr == 'Error: Some corners are undefined.' :
        solution = solStr
        movecount = 0
        print solStr
        pass
    else:
        print "solution: %s" %(solStr)
        if solStr[len(solStr)-2:] == 'f)':
            movecount = int(solStr[len(solStr)-4:len(solStr)-2])
            solution = solStr[:len(solStr)-6]
            pass
        else:
            movecount = 0
            solution = solStr
            pass
        pass 
    return solution, movecount

def handle_solve(req):
    print "\nproblem: %s" %(req.defstr)
    solStr = solve(str(req.defstr)) # twophase algorithm
    solution, movecount = check_solution(solStr) # check for errors

    return SolverResponse(movecount, solution)

def cube_solver_server():
    rospy.init_node('cube_solver_server')
    s = rospy.Service('cube_solver', Solver, handle_solve)
    print "Ready to solve a cube."
    rospy.spin()

if __name__ == "__main__":
    cube_solver_server()