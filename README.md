# robot-arm

## Aim
Control a robot arm with your Raspberry Pi (Zero)

This repo will document how to control a robot arm. The arm in question has 6 servos or degrees of freedom (DOF). We will use simple python to position the servos.

But before doing so we have to:
* introduce a mathematical model for the arm with 4 angles (2 servos are used to rotate and open/close the clamp, so they don't determine the position of the clamp)
* map the angles of that model to servo signals
* for a position in 3d the arm can reach: find corresponding angles that point the roboter arm to that position in space 
* most servos are restricted and cannot rotate 360 degrees: thus on has to check the angles of the solutions stay within these bounds to guarantee a safe operation.

## Organization

Currently robotarm.py already is capable of finding numerical solutions to that mapping problem by using gradient descent.
