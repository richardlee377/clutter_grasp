import math 
from math import (sin, cos, atan, atan2)

# the value we know
# dpx, dpy -- desire object position
# ipx, ipy -- intial position of the gripper
# opx, opy -- position of the obstacle (middle point)
# x, y. theta -- the position of the gripper
# r -- the length of diagonal of the obstacale 
# l -- the length of the gripper arm

def sweep(dpx, dpy, ipx, ipy, opx, opy, r, l):
# This fuction will return the angle that the gripper need to rotate at first, 
# and the three position  that the end point of the gripper need to move. We name
# it (px1, py1), (px2, py2), (px3, py3)

	theta = atan2(dpy - ipy, dpx - ipx)
	theta_obs = atan2(opy - ipy, opx - ipx)
	d_ob = math.sqrt((opy - ipy)**2 + (opx - ipx)**2)  - l/2#distance to the obstacle
	if theta <= theta_obs:
		(px1, py1) = swp_move('right', theta, r, ipx, ipy)
		(px2, py2) = swp_move('straightforward', theta, d_ob, px1, py1)
		d_ob = math.sqrt((opy - py2)**2 + (opx - px2)**2)
		(px3, py3) = swp_move('left', theta, d_ob, px2, py2)

	else:
		(px1, py1) = swp_move('left', theta, r, ipx, ipy)
		(px2, py2) = swp_move('straightforward', theta, d_ob, px1, py1)
		d_ob = math.sqrt((opy - py2)**2 + (opx - px2)**2)
		(px3, py3) = swp_move('right', theta, d_ob, px2, py2)

	return (px1, py1), (px2, py2), (px3, py3), theta

def swp_move(dir, theta, d, ipx, ipy):
# d is the distance that the gripper need to move
	
	if dir == 'right':
		theta = theta - math.pi/2
		ipx = ipx + d*cos(theta) 
		ipy = ipy + d*sin(theta) 
	elif dir == 'left':
		theta = theta - math.pi/2
		ipx = ipx - d*cos(theta)
		ipy = ipy - d*sin(theta)
	else:
		ipx = ipx + d*cos(theta)
		ipy = ipy + d*sin(theta)

	print 'theta', theta
	print 'px', d*cos(theta) 
	print 'py', d*sin(theta)
	return (ipx, ipy)

# r = diag()
r = math.sqrt((2*1.5)**2 + (2*3)**2)
p1, p2, p3, theta = sweep(15, 12, 18, 5, 17, 19, 2, 3)
print p1
print p2
print p3
print theta