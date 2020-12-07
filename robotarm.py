#/usr/bin/python3

import numpy as np
import math

#length of the arm sections
l1 = 20
l2 = 17
l3 = 10


#four angles of servos determine armposition, remaining two are to rotate clamp + open/close clamp
def armposition(phi, theta1, theta2, theta3):
    x = math.cos(phi) * (l1 * math.sin(theta1) + l2 * math.sin(theta1+theta2) + l3 * math.sin(theta1+theta2+theta3))
    y = math.sin(phi) * (l1 * math.sin(theta1) + l2 * math.sin(theta1+theta2) + l3 * math.sin(theta1+theta2+theta3))
    z = l1 * math.cos(theta1) + l2 * math.cos(theta1+theta2) + l3 * math.cos(theta1+theta2+theta3)

    return (x, y, z)

def roboarm(x):
    return armposition(x[0], x[1], x[2], x[3])

#from sage:
def gradient(phi, theta1, theta2, theta3, x0, y0, z0):

    dphi = 2*((l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))*math.sin(phi) - y0)*(l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))*math.cos(phi) - 2*((l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))*math.cos(phi) - x0)*(l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))*math.sin(phi) 
        
    dtheta1 = 2*((l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))*math.cos(phi) - x0)*(l3*math.cos(theta1 + theta2 + theta3) + l2*math.cos(theta1 + theta2) + l1*math.cos(theta1))*math.cos(phi) + 2*(l3*math.cos(theta1 + theta2 + theta3) + l2*math.cos(theta1 + theta2) + l1*math.cos(theta1))*((l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))*math.sin(phi) - y0)*math.sin(phi) - 2*(l3*math.cos(theta1 + theta2 + theta3) + l2*math.cos(theta1 + theta2) + l1*math.cos(theta1) - z0)*(l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))

    dtheta2 = 2*((l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))*math.cos(phi) - x0)*(l3*math.cos(theta1 + theta2 + theta3) + l2*math.cos(theta1 + theta2))*math.cos(phi) + 2*(l3*math.cos(theta1 + theta2 + theta3) + l2*math.cos(theta1 + theta2))*((l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))*math.sin(phi) - y0)*math.sin(phi) - 2*(l3*math.cos(theta1 + theta2 + theta3) + l2*math.cos(theta1 + theta2) + l1*math.cos(theta1) - z0)*(l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2)) 

    dtheta3 = 2*((l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))*math.cos(phi) - x0)*l3*math.cos(phi)*math.cos(theta1 + theta2 + theta3) + 2*((l3*math.sin(theta1 + theta2 + theta3) + l2*math.sin(theta1 + theta2) + l1*math.sin(theta1))*math.sin(phi) - y0)*l3*math.cos(theta1 + theta2 + theta3)*math.sin(phi) - 2*(l3*math.cos(theta1 + theta2 + theta3) + l2*math.cos(theta1 + theta2) + l1*math.cos(theta1) - z0)*l3*math.sin(theta1 + theta2 + theta3)

    return (dphi, dtheta1, dtheta2, dtheta3)


def grad(x, target):
    return gradient(x[0],x[1],x[2], x[3], target[0], target[1], target[2])

def sqrdist(x, y):
    return (x[0]-y[0]) ** 2 + (x[1]-y[1]) **2 + (x[2]-y[2]) **2

#along z-axis
arm_upright = (0,0, 40)

#tilted a bit from z-axis
test1 = armposition(0.1,-0.1,0.1,0.1)
print(test1, "-  dist: ", sqrdist(arm_upright, test1))

#this is along the x-axis:
test2 = armposition(0,math.pi/2.,0.,0)
print(test2)

#how set up gradient descent? - cost function to minimize is sqrdist to given point, so that's the function to build gradient of.

#used sagemath to calculate gradient
#now try it out:

#difference to target position
def cost(x, y):
    return sqrdist( (x[0],x[1],x[2]), (y[0],y[1],y[2]))


def solve_for_position(target, start, step_size, precision):

    sol = start
    for i in range(1,100000):
        #print(i, sol, sqrdist(roboarm(sol), (x0,y0,z0)))
        ds = step_size * np.asarray(grad(sol, target))
        #print("step: ", ds)
        sol -= ds
        dist = sqrdist(roboarm(sol), target)
        if (dist < precision):
            print("converged, solution: ", sol, " dist: ", dist, " steps needed", i)
            return sol

    print("did not converge, current state: ", sol, " dist: ", sqrdist(roboarm(sol), target))
    return sol


sol = (0.1, -0.1, 0.1, 0.1)
step_size = 0.0001 #"learning rate"

print("arm: ", roboarm(sol))
print("dist: ", cost(roboarm(sol), arm_upright), sqrdist(roboarm(sol), arm_upright))

sol = solve_for_position(arm_upright, sol, step_size, 0.0001)

print("now worse start position")
sol = (0.1, 1, 2, 1)
solve_for_position(arm_upright, sol, step_size, 0.0001)

sol = (0.7, 0.6, 2.3, 1)
solve_for_position((5.3,4.3,6), sol, step_size, 0.0001)


