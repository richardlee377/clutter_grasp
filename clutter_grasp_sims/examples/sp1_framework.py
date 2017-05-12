import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
import Box2D  # The main library
# Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody, worldManifold, rayCastInput, rayCastOutput, vec2)

import numpy as np
import copy
import matplotlib.pyplot as plt
import math
from math import (sin, cos, atan, atan2)

def diag(l,w):
    return math.sqrt(l**2 + w**2)

def dist(x,y):
    return math.sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2)

def PG(ph, a, d, theta0):
    # Calculate Vertices of Negative Are
    theta = theta0
    print theta
    neg_area = world.CreateStaticBody(position=(ph[0]+0.5*d*cos(theta), ph[1]+0.5*d*sin(theta)), angle=theta)
    neg_area.active = False
    na_fixt = neg_area.CreatePolygonFixture(box=(d, a), density=1, friction=1)
    # shape = na_fixt.shape
    # vertices = [(neg_area.transform * v) * PPM for v in shape.vertices]
    # color = [(124,252,0,0)]
    color = 1

    # Calculate Push motion
    v      = d/(N-Nt)
    trj_x  = v*cos(theta)*np.matrix(np.ones((N-Nt,1)))
    trj_x  = TARGET_FPS*np.vstack((np.zeros((Nt,1)),trj_x))
    trj_y  = v*sin(theta)*np.matrix(np.ones((N-Nt,1)))
    trj_y  = TARGET_FPS*np.vstack((np.zeros((Nt,1)),trj_y))
    trj_th = float(theta0-init_pos[2])/(Nt)*np.matrix(np.ones((Nt,1)))
    trj_th = TARGET_FPS*np.vstack((trj_th,np.zeros((N-Nt,1))))
    psi_naive = np.hstack((trj_x,trj_y,trj_th))
    return neg_area, color, psi_naive

def move(pi, pf, T):
    # Calculate trajectory
    T = int(T)
    trj_x = TARGET_FPS*(pf[0]-pi[0])/T*np.matrix(np.ones((T,1)))
    trj_y = TARGET_FPS*(pf[1]-pi[1])/T*np.matrix(np.ones((T,1)))
    trj_th= TARGET_FPS*(pf[2]-pi[2])/T*np.matrix(np.ones((T,1)))
    psi = np.hstack((trj_x,trj_y,trj_th))
    
    # Calculate negative area
    th = atan2(pf[1]-pi[1], pf[0]-pi[0])
    d_tot = dist(pf[0:2],pi[0:2])
    xc = pi[0] + d_tot*cos(th)
    yc = pi[1] + d_tot*sin(th)
    neg_area = world.CreateStaticBody(position=(xc, yc), angle=th)
    neg_area.active = False
    na_fixt = neg_area.CreatePolygonFixture(box=(d_tot, a), density=1, friction=1)
    # color = (124,252,0,0)
    color = 1
    return neg_area, color, psi    

def SW(dp, ip, op, r, l, theta0):
# This fuction will return the angle that the gripper need to rotate at first, 
# and the three position  that the end point of the gripper need to move. We name
# it (px1, py1), (px2, py2), (px3, py3)
    dpx = dp[0]
    dpy = dp[1]
    ipx = ip[0]
    ipy = ip[1]
    opx = op[0]
    opy = op[1]

    theta = atan2(dpy - ipy, dpx - ipx)
    # print "theta: ",theta
    theta_obs = atan2(opy - ipy, opx - ipx)
    d_ob = dist([opx, opy], [ipx,ipy]) - l/2#distance tocd  the obstacle
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

    print px1, py1
    print px2, py2
    print px3, py3

    # part 1
    t1 = math.floor(0.333*N)
    t2 = math.floor(0.333*N)
    t3 = N-t1-t2
    na1, c1, trj1 = move([ipx, ipy, theta0], [px1, py1, theta0], t1)
    # part 2
    na2, c2, trj2 = move([px1, py1, theta0], [px2, py2, theta],  t2)
    # part 3
    na3, c3, trj3 = move([px2, py2,  theta], [px3, py3, theta],  t3)

    na = [na1, na2, na3]
    c = c1 + c2 + c3
    trj = np.vstack((trj1,trj2, trj3))
    # print trj.shape
    return na, c, trj

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

    # print 'theta', theta
    # print 'px', d*cos(theta) 
    # print 'py', d*sin(theta)
    return (ipx, ipy)



# --------------------------SIMULATE--------------------------
def simulate(cmd, trj, na, color):
    # Create dynamic bodies
    des_body = world.CreateDynamicBody(position=(db_init[0], db_init[1]),  angle=db_init[2], 
        linearDamping = 0.5*1*9.8, angularDamping = 0.3*1/12*9.8)
    obs1_body = world.CreateDynamicBody(position=(o1_init[0], o1_init[1]), angle=o1_init[2],
        linearDamping = 0.5*18*9.8, angularDamping = 0.3*4/12*9.8)
    obs2_body = world.CreateDynamicBody(position=(o2_init[0], o2_init[1]), angle=o2_init[2],
        linearDamping = 0.5*24*9.8, angularDamping = 0.3*4/12*9.8)

    # Create Gripper
    gripper = world.CreateKinematicBody(position=(18, 5), angle = 0)

    # Add polygon fixtures for objects
    des_fixt = des_body.CreatePolygonFixture(box=(1, 1), density=1, friction=0.3, restitution = 0.8)
    obs1_box = obs1_body.CreatePolygonFixture(box=(1.5, 3), density=1, friction=0.3, restitution = 0.8)
    obs2_box = obs2_body.CreatePolygonFixture(box=(3, 2), density=1, friction=0.3, restitution = 0.8)

    # Add sensors for the contact points
    # print vec2(-1,0)
    cnt1 = des_body.CreatePolygonFixture(box=(0.05, 0.05, vec2(-1,0), 0), density=0, isSensor = True)
    cnt2 = des_body.CreatePolygonFixture(box=(0.05, 0.05, vec2( 1,0), 0), density=0, isSensor = True)

    # gripperbase = polygonShape(vertices=[(-w/2,0), (-w/2,l), (-ow/2,l), 
    #   (-ow/2,l+lf), (-iw/2,l+lf), (-iw/2,l+lf-lt), (iw/2,l+lf-lt), (iw/2,l+lf), (ow/2,l+lf),
    #   (ow/2,l), (w/2,l), (w/2,0)])
    gripperbase = gripper.CreatePolygonFixture(box=(w/2,l/2), density = 1, friction = 0.3)
    gripperpalm = gripper.CreatePolygonFixture(box=(ow/2, lt/2, vec2(0,l/2+lt/2), 0), density = 1, friction = 0.3)
    gripperfngL = gripper.CreatePolygonFixture(box=(wfng/2, lfng/2, vec2(-ow/2+wfng/2,l/2+lt+lfng/2), 0), density = 1, friction = 0.3)
    gripperfngR = gripper.CreatePolygonFixture(box=(wfng/2, lfng/2, vec2( ow/2-wfng/2,l/2+lt+lfng/2), 0), density = 1, friction = 0.3)

    # Mass and Moment of Inertia data
    # print "des_body: " + str(des_body.mass) + " kg , " + str(des_body.inertia) + " kg*m^2"
    # print "obs_body: " + str(obs_body.mass) + " kg , " + str(obs_body.inertia) + " kg*m^2"

    # white = (255, 255, 255, 255)
    # print des_body.fixtures
    colors = [(255, 50, 50, 255), (124,252,0,0), (124,252,0,0), 
        (50, 50, 255, 255), (50, 50, 255, 255), (255, 255, 255, 255),
        (255, 255, 255, 255), (255, 255, 255, 255), (255, 255, 255, 255)]
    bodies = [des_body, obs1_body, obs2_body, gripper]

    if cmd != "fwd":
        for i in range(color):
            colors.insert(0,(124,252,0,0))
        if color == 1:
             bodies.insert(0,na)
        else:
            for b in na:
                bodies.insert(0,b)

    print colors
    print bodies
    LQ = 1

    # --- main game loop ---
    for k in range(N):
        # Check the event queue
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                # The user closed the window or pressed escape
                running = False

        screen.fill((0, 0, 0, 0))
        # Draw the world
        i = 0
        hits  = 0
        cnt_reward = 0
        # Cast a ray from the center of the fingers to the desired object
        psi_ray = gripper.GetWorldPoint(gripper.localCenter + [0, l/2+lt])
        x_ray = psi_ray[0]
        y_ray = psi_ray[1]
        x_db  = des_body.worldCenter[0]
        y_db  = des_body.worldCenter[1]

        input = rayCastInput(p1=(x_ray,y_ray), p2=(x_db,y_db), maxFraction=1)
        output = rayCastOutput()

        # Check contact points on the gripper
        for ce in gripper.contacts:
            # print 'contact: ',ce.contact
            if ce.contact.touching:
                cntbody = ce.contact.fixtureA.body
                if moveList.count(cntbody) == 0:
                    moveList.append(cntbody)
                elif moveList_prev.count(cntbody)!=0:
                    if avoidList.count(cntbody)==0:
                        avoidList.append(cntbody)
        # raw_input() 

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
                # print "vertices",vertices

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

        gripper.linearVelocity = (trj[k,0], trj[k,1])
        gripper.angularVelocity = trj[k,2]
        if hits > 0:
            cnt_reward = 0
        else:
            cnt_reward = 1*LQ
        # print cnt_reward
        # Make Box2D simulate the physics of our world for one step.
        # Instruct the world to perform a single step of simulation. It is
        # generally best to keep the time step and iterations fixed.
        # See the manual (Section "Simulating the World") for further discussion
        # on these parameters and their implications.
        world.Step(TIME_STEP, 10, 10)
        pygame.display.flip()
        clock.tick(TARGET_FPS)
    db  = [des_body.worldCenter[0],  des_body.worldCenter[1],  des_body.angle]
    o1 = [obs1_body.worldCenter[0], obs1_body.worldCenter[1], obs1_body.angle]
    o2 = [obs2_body.worldCenter[0], obs2_body.worldCenter[1], obs2_body.angle]

    for body in bodies:
        world.DestroyBody(body)
    return db, o1, o2
#---------------------------------THE GAME----------------------------------------
# --- constants ---
# Box2D deals with meters, but we want to display pixels,
# so define a conversion factor:
PPM = 20.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

# Polygon fixture for gripper
l  = 3
w  = 1.5
lt = 0.1
ow = 2.5
wfng = 0.25
lfng = 0.5

# initial body locations
db_init = [15, 12, 0] 
o1_init = [17, 9, 30]
o2_init = [11,11, 0]

# --- pygame setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

# --- pybox2d world setup ---
# Create the world
world = world(gravity=(0, 0), doSleep=True)

# BODY MASSES (Unique to each body- serves as a key)
# Value is x0, y0, 2*l, 2*w
des_mass = 4
obs1_mass = 18
obs2_mass = 24
init_cond = {des_mass: (15, 12, 2, 2), obs1_mass: (17, 9, 3, 6), obs2_mass: (11, 11, 6, 4)}

# SETUP THE LISTS:
moveList      = []
moveList_prev = []
avoidList     = []

N = 181
Nt = 30
a = 2.5/2

# FIRST STEP: try to directly reach the final position
init_pos  =  [18, 5, math.pi/2]
final_pos =  [15, 12-l/2-lt]
theta0 = atan2((final_pos[1]-init_pos[1]),(final_pos[0] - init_pos[0]))
d = math.sqrt(float(final_pos[0] - init_pos[0])**2 + float(final_pos[1] - init_pos[1])**2)
# print theta0 - math.pi/2
neg_area, color, trj_naive = PG(init_pos, a, d, theta0)
# print psi_naive
_,_,_ = simulate("naive",trj_naive, neg_area, color)
# for body in moveList:
#     print "MoveListbody: ",body
# print 'moveList: ', len(moveList)
# print 'avoidList: ', len(avoidList)
moveList_prev = moveList

# Go through the library of actions
# FIRST TRY THE SWEEP FOR THE FIRST OBSTACLE IN MOVELIST
des_pos = init_cond[4]
for key in init_cond:
    moveFirst = moveList_prev[0]
    if moveFirst.mass == key:
        obs_pos = init_cond[key]
# print obs_pos

r = diag(obs_pos[2],obs_pos[3])

print des_pos[0:2]
print init_pos[0:2]
print obs_pos[0:2]
na, c, trj = SW(des_pos[0:2], init_pos[0:2], obs_pos[0:2], r, l, math.pi/2)
# neg_area, color, psi_naive = SW(des_pos[0], des_pos[1], init_pos[0], init_pos[1], obs_pos[0], obs_pos[1], r, l)
_,_,_ = simulate("naive",trj, na, c)


# FORWARD SIMULATE
db_init,o1_init,o2_init = simulate("fwd",trj, 0, 0)
simulate("fwd",trj_naive, 0, 0)


# pygame.quit()
print('Simulation Done!')