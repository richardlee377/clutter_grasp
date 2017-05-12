import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)

import numpy as np
import scipy.linalg as sp
# import random as random

import Box2D  # The main library
# Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, rayCastInput, rayCastOutput, vec2)

colors = [(255, 255, 255, 255), (255, 50, 50, 255), (50, 50, 255, 255)]

# print colors[2]

d = np.loadtxt('examples/vel_prof1.txt', delimiter=";")
# print d[0][0]

vd = vec2(d[1][0], d[1][1])
print vd

t = range(0, 181)
print t

from cvxopt import matrix, solvers
Q = 2*matrix([ [2, .5], [.5, 1] ])
p = matrix([1.0, 1.0])
G = matrix([[-1.0,0.0],[0.0,-1.0]])
h = matrix([0.0,0.0])
A = matrix([1.0, 1.0], (1,2))
b = matrix(1.0)
sol=solvers.qp(Q, p, G, h, A, b)



import matplotlib.pyplot as plt
# plt.plot([1,2,3,4])
# plt.ylabel('some numbers')
# plt.show()

t = range(0, 181)
v_init = np.loadtxt('examples/vel_prof1.txt', delimiter=";")
# print v_init[:,0]

# red dashes, blue squares and green triangles
# plt.plot(t, t, 'r--', t, t**2, 'bs', t, t**3, 'g^')
plt.figure(1)
plt.plot(t, v_init[:,0], 'b')
plt.xlabel('time [s]')
plt.ylabel('x-dir velocity [m/s]')
# plt.show()

p_init = np.loadtxt('examples/pos_prof1.txt', delimiter=";")
# print p_init[:,0]

plt.figure(2)
plt.plot(t, p_init[:,0], 'b')
plt.xlabel('time [s]')
plt.ylabel('x-dir position [m]')
# plt.show()

m = 1
# a = np.matrix([m, -m])
# b = [np.matrix([-m, 2*m, -m])]*50
# c = np.matrix([-m, m])
# # print c
# # help(scipy)
# tmp = np.matrix(sp.block_diag(*b)) 
# print np.matrix(sp.block_diag(a,tmp,c)
# N = 181
# tmp = 2*m*np.identity(N-2)
# tmp = np.matrix(sp.block_diag(m,tmp,m))
# # print tmp[0,0]
# for i in range(N):
# 	for j in range(N):
# 		if j==(i+1) or j==(i-1):
# 			tmp[i,j] = -m
# print tmp

# print np.abs(3-5)

#--------------------------------------------------------------------------------------#
C1 = 1
C2 = 10
mfng = 1
TARGET_FPS = 60
N = 181
A = -1*np.identity(N) + 1*np.eye(N, k=1)
Bk = (2*C1*mfng)*np.identity(N) + (-C1*mfng)*np.eye(N, k=1) + (-C1*mfng)*np.eye(N, k=-1)
Bk[0,0] = C1*mfng
Bk[N-1,N-1] = C1*mfng
# print A.shape
# print Bk

Atmp = np.zeros((1,N-1))
A  = np.concatenate((np.matrix(1),Atmp),axis = 1)
b = np.matrix(0)
print b

# --- constants ---
# Box2D deals with meters, but we want to display pixels,
# so define a conversion factor:
PPM = 20.0  # pixels per meter
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

# --- pygame setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

# --- pybox2d world setup ---
# Create the world
world = world(gravity=(0, 0), doSleep=True)

# Add a static body to hold the ground shape
ground_body = world.CreateStaticBody(
    position=(0, 1),
    shapes=polygonShape(box=(50, 1)),
)

# Create dynamic bodies
des_body = world.CreateDynamicBody(position=(15, 12), angle=0, 
    linearDamping = 0.5*4*9.8, angularDamping = 0.3*1/12*9.8)

obs1_body = world.CreateDynamicBody(position=(18, 12), angle=0,
    linearDamping = 0.5*16*9.8, angularDamping = 0.3*4/12*9.8)
obs2_body = world.CreateDynamicBody(position=(11,11), angle=0,
    linearDamping = 0.5*36*9.8, angularDamping = 0.3*4/12*9.8)

# Create fingers as kinematic bodies (infinite masses and directly controls velocity)
width = 2.5
fng1 = world.CreateKinematicBody(position=(20, 5), angle = 0)
fng2 = world.CreateKinematicBody(position=(20+width,5), angle = 0)
# fng1 = world.CreateKinematicBody(position=(14, 5), angle = 0)
# fng2 = world.CreateKinematicBody(position=(14+width,5), angle = 0)

# And add box fixtures onto it (with a nonzero density, so it will move)
des_box  = des_body.CreatePolygonFixture(box=(1, 1), density=1, friction=0.3, restitution = 0.8)
obs1_box = obs1_body.CreatePolygonFixture(box=(2, 2), density=1, friction=0.3, restitution = 0.8)
obs2_box = obs2_body.CreatePolygonFixture(box=(3, 3), density=1, friction=0.3, restitution = 0.8)

# Add sensors for the contact points
# print vec2(-1,0)
cnt1 = des_body.CreatePolygonFixture(box=(0.05, 0.05, vec2(-1,0), 0), density=0, isSensor = True)
cnt2 = des_body.CreatePolygonFixture(box=(0.05, 0.05, vec2( 1,0), 0), density=0, isSensor = True)
printflag = True

# Model fingers as small circular cross sections
# circle = circleShape(radius=0.1)
fng1_cir = fng1.CreatePolygonFixture(box = (0.1, 0.1), density = 5, friction = 0.3)
fng2_cir = fng2.CreatePolygonFixture(box = (0.1, 0.1), density = 5, friction = 0.3)

# Mass and Moment of Inertia data
# print "des_body: " + str(des_body.mass) + " kg , " + str(des_body.inertia) + " kg*m^2"
# print "obs_body1: " + str(obs_body1.mass) + " kg , " + str(obs_body1.inertia) + " kg*m^2"
# print fng1.linearVelocity

colors = [(255, 255, 255, 255), (255, 50, 50, 255), (124,252,0), (124,252,0),
     (50, 50, 255, 255), (50, 50, 255, 255), (255, 255, 255, 255), (255, 255, 255, 255)]
bodies = [ground_body, des_body, obs1_body, obs2_body, fng1, fng2]

for k in range(N):
    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            running = False

    screen.fill((0, 0, 0, 0))
    # Draw the world
    i = 0
    cnt_reward = 0
    # Cast a ray from the center of the fingers to the desired object
    x_ray = fng1.worldCenter[0] + width/2
    y_ray = fng1.worldCenter[1]
    x_db  = des_body.worldCenter[0]
    y_db  = des_body.worldCenter[1]
    input = rayCastInput(p1=(x_ray,y_ray), p2=(x_db,y_db), maxFraction=1)
    output = rayCastOutput()
    hits = 0

    for body in bodies:  # or: world.bodies
        # The body gives us the position and angle of its shapes
        for fixture in body.fixtures:
            # The fixture holds information like density and friction,
            # and also the shape.
            shape = fixture.shape

            # Naively assume that this is a polygon shape. (not good normally!)
            # We take the body's transform and multiply it with each
            # vertex, and then convert from meters to pixels with the scale
            # factor.
            vertices = [(body.transform * v) * PPM for v in shape.vertices]

            # But wait! It's upside-down! Pygame and Box2D orient their
            # axes in different ways. Box2D is just like how you learned
            # in high school, with positive x and y directions going
            # right and up. Pygame, on the other hand, increases in the
            # right and downward directions. This means we must flip
            # the y components.
            vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]

            if body != des_body:
                hit = shape.RayCast(output, input, body.transform, 0)
                if hit:
                    hits = 1 + hits
            pygame.draw.polygon(screen, colors[i], vertices)
            i = i + 1
    print hits
    # Make Box2D simulate the physics of our world for one step.
    # Instruct the world to perform a single step of simulation. It is
    # generally best to keep the time step and iterations fixed.
    # See the manual (Section "Simulating the World") for further discussion
    # on these parameters and their implications.
    world.Step(TIME_STEP, 10, 10)
    # Flip the screen and try to keep at the target FPS
    if True:
        pygame.display.flip()
        clock.tick(TARGET_FPS)
    else:
        pass