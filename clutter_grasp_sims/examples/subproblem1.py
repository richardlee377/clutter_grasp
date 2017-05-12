#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
import Box2D  # The main library
# Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, vec2)

import cvxopt
from cvxopt import matrix, solvers

import numpy as np
import copy
import matplotlib.pyplot as plt

def KE(body):
    vl = body.linearVelocity
    va = body.angularVelocity
    KE = 0.5*body.mass*np.linalg.norm(vl)**2  + 0.5*body.inertia*np.linalg.norm(va)**2
    return KE

def KE_fng(body):
    vl = body.linearVelocity
    va = body.angularVelocity
    KE = 0.5*1*np.linalg.norm(vl)**2  + 0.5*1*np.linalg.norm(va)**2
    return KE

def simulate(cmd, trj):
    import pygame
    from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)

    import Box2D  # The main library
    # Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
    from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, vec2)

    import numpy as np
    import copy

    # --- constants ---
    # Box2D deals with meters, but we want to display pixels,
    # so define a conversion factor:
    PPM = 20.0  # pixels per meter
    TARGET_FPS = 60
    TIME_STEP = 1.0 / TARGET_FPS
    SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

    # --- cost function constants ---
    C_fng = 1
    C_des = 1
    C_obs = 1
    C_cnt = 1

    # --- pygame setup ---
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
    pygame.display.set_caption('Simple pygame example')
    clock = pygame.time.Clock()

    # --- pybox2d world setup ---
    # Create the world
    world = world(gravity=(0, 0), doSleep=True)

    # And a static body to hold the ground shape
    ground_body = world.CreateStaticBody(
        position=(0, 1),
        shapes=polygonShape(box=(50, 1)),
    )

    # Create dynamic bodies
    des_body = world.CreateDynamicBody(position=(15, 12), angle=0, 
        linearDamping = 0.5*1*9.8, angularDamping = 0.3*1/12*9.8)

    obs_body = world.CreateDynamicBody(position=(18, 12), angle=0,
        linearDamping = 0.5*4*9.8, angularDamping = 0.3*4/12*9.8)

    # Create fingers as kinematic bodies (infinite masses and directly controls velocity)
    width = 2.5
    fng1 = world.CreateKinematicBody(position=(16, 5), angle = 0)
    fng2 = world.CreateKinematicBody(position=(16+width,5), angle = 0)

    # And add box fixtures onto it (with a nonzero density, so it will move)
    des_box = des_body.CreatePolygonFixture(box=(1, 1), density=1, friction=0.3, restitution = 0.8)
    obs_box = obs_body.CreatePolygonFixture(box=(2, 2), density=1, friction=0.3, restitution = 0.8)

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
    # print "obs_body: " + str(obs_body.mass) + " kg , " + str(obs_body.inertia) + " kg*m^2"
    # print fng1.linearVelocity

    colors = [(255, 255, 255, 255), (255, 50, 50, 255), (124,252,0), (124,252,0),
         (50, 50, 255, 255), (255, 255, 255, 255), (255, 255, 255, 255)]

    bodies = [ground_body, des_body, obs_body, fng1, fng2]

    # LOAD NAIVE VELOCITY PROFILE: generated in matlab
    if cmd == "naive":
        v_prof = np.loadtxt('examples/vel_prof1.txt', delimiter=";")
        v_x = v_prof[:,0]
        v_y = v_prof[:,1]
        # print v_prof
        # v_prof = np.array(v_prof, dtype='f')
    else:
        v_x  = np.gradient(trj[:,0])/TARGET_FPS
        v_y  = np.gradient(trj[:,1])/TARGET_FPS

    # INITIALIZE THE COST
    f_x = 0
    pos_des_prev = copy.copy(des_body.worldCenter)
    pos_obs_prev =  copy.copy(obs_body.worldCenter)
    LQ = 1
    # print "initial pos: (" + str(pos_des_prev[0]) + ", " + str(pos_des_prev[1]) + ")" 

    # --- main game loop ---
    # while True:
    for k in range(0, 181):
        # Check the event queue
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                # The user closed the window or pressed escape
                running = False

        screen.fill((0, 0, 0, 0))
        # Draw the world
        i = 0
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

                pygame.draw.polygon(screen, colors[i], vertices)
                i = i + 1
                # print i
            vd = vec2(v_x[k], v_y[k])
            fng1.linearVelocity = vd
            fng2.linearVelocity = vd
            # print fng1.linearVelocity
            # print fng2.linearVelocity

        # Collect data from these bodies
        pos_des = des_body.worldCenter
        pos_obs = obs_body.worldCenter
        # Calculate the difference
        d_des = np.linalg.norm(pos_des - pos_des_prev)
        d_obs = np.linalg.norm(pos_obs - pos_obs_prev)
        # print d_des

        KE_des = KE(des_body)
        KE_obs = KE(obs_body)
        # print KE_des
        # print KE_obs

        # Check contacts
        cnt_reward = 0
        for c in des_body.contacts:
            # if printflag:
            #     print c.contact.touching
            #     printflag = False
            if c.contact.touching :
                # print "sensor triggered"
                cnt_reward = 0
            else:
                # print "contacts points are free"
                cnt_reward = 1*LQ     

        # Determine the kinetic energy of the fingers
        KE_fng1 = KE_fng(fng1)
        KE_fng2 = KE_fng(fng2)
        # print KE_fng1 + KE_fng2

        # Integrate the Cost function
        f_x = C_fng*KE_fng1 + C_fng*KE_fng2 + C_des*KE_des + C_obs*KE_obs - C_cnt*cnt_reward + f_x
        # print f_x

        # Update the previous position
        pos_des_prev = copy.copy(pos_des)
        pos_obs_prev = copy.copy(pos_obs)

        # Make Box2D simulate the physics of our world for one step.
        # Instruct the world to perform a single step of simulation. It is
        # generally best to keep the time step and iterations fixed.
        # See the manual (Section "Simulating the World") for further discussion
        # on these parameters and their implications.
        world.Step(TIME_STEP, 10, 10)
        # Flip the screen and try to keep at the target FPS
        if cmd == "naive" or cmd == "show":
            pygame.display.flip()
            clock.tick(TARGET_FPS)
        else:
            pass

    pygame.quit()
    print('Simulation Done!')

    # RETURN THINGS
    # x_k = v_prof[:,0]
    # y_k = v_prof[:,1]
    # return x_k # the velocity trajectory x_k
    # return y_k # the velocity trajectory y_k
    return (v_x, v_y, f_x) # the velocity profile, the total cost


# def abs_m(x):
#     x_ret  = np.zeros(x.size)
#     for i in range(x.size):
#         if x[i] > 0:
#             x_ret[i] = x[i]
#         else:
#             x_ret[i] = 0 
#     return x_ret

def abs_p(x):
    if x > 0:
        return x
    else:
        return 0.0
     
###########################The l1QP Algorithm####################################

# INITIALIZE WITH THE NAIVE SOLUTION
TARGET_FPS = 60
_, _, f_x = simulate("naive", 0)
xk = np.loadtxt('examples/pos_prof1.txt', delimiter=";")
# print xk