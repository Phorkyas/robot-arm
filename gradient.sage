
var('phi, theta1, theta2, theta3')

var('l1, l2, l3')

x(phi, theta1, theta2, theta3) = cos(phi)*(l1*sin(theta1)+l2*sin(theta1+theta2)+l3*sin(theta1+theta2+theta3))
y(phi, theta1, theta2, theta3) = sin(phi)*(l1*sin(theta1)+l2*sin(theta1+theta2)+l3*sin(theta1+theta2+theta3))
z(phi, theta1, theta2, theta3) = l1 * cos(theta1) + l2*cos(theta1+theta2) + l3*cos(theta1+theta2+theta3)

arm = vector([x, y, z])

var('x0, y0, z0')
dist = arm - vector([x0,y0,z0])

dist(phi=0,theta1=0,theta2=0,theta3=0)

cost = dist * dist

cost.diff(phi)

cost.gradient()


