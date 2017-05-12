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
    KEx = 0.5*body.mass*(vl[0]**2)  + 0.5*body.inertia*np.linalg.norm(va)**2
    KEy = 0.5*body.mass*(vl[1]**2)  + 0.5*body.inertia*np.linalg.norm(va)**2
    return KEx, KEy

def KE_fng(body, mfng):
    vl = body.linearVelocity
    # KEx = 0.5*mfng*(vl[0])**2
    # KEy = 0.5*mfng*(vl[1])**2
    KEx = 0.5*mfng*(vl[0]/TARGET_FPS)**2
    KEy = 0.5*mfng*(vl[1]/TARGET_FPS)**2
    return KEx, KEy

def simulate(cmd, trj):
    import Box2D  # The main library
    # Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
    from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody, vec2)

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

    # LOAD DESIRED VELOCITY PROFILE
    if cmd == 'naive':
        v_prof = np.loadtxt('examples/vel_prof1.txt', delimiter=';')
        v_x = v_prof[:,0]
        v_y = v_prof[:,1]
        psi_prof = np.loadtxt('examples/pos_prof1.txt', delimiter=';')
        xfng = np.reshape(np.matrix(psi_prof[:,0]),(N,1))
        yfng = np.reshape(np.matrix(psi_prof[:,1]),(N,1))
        # print xfng
        # print v_y/TARGET_FPS

    else:
        v_prof = np.gradient(trj, axis=0)*TARGET_FPS
        v_x = v_prof[:,0]
        v_y = v_prof[:,1]
        xfng = trj[:,0]
        yfng = trj[:,1]
        # print 'something else', v_x

    # GATHER ACTUAL FNG POSITIONS
    # xfng = np.zeros((N, 1))
    # yfng = np.zeros((N, 1))    

    # INTIALIZE THE COST FUNCTION
    fx = 0
    fy = 0
    LQ = 1
    xdes = np.zeros((N,1))
    ydes = np.zeros((N,1))

    # --- main game loop ---
    # while True:
    for k in range(N):
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
            vd = vec2((float)(v_x[k]), (float)(v_y[k]))
            fng1.linearVelocity = vd
            fng2.linearVelocity = vd
            # print fng1.linearVelocity
            # print fng2.linearVelocity    

        # Collect data from these bodies
        KEx_des, KEy_des = KE(des_body)
        KEx_obs, KEy_obs = KE(obs_body)
        psi_des = des_body.GetWorldPoint(des_body.localCenter + [-width/2,0])
        xdes[k] = psi_des[0]
        ydes[k] = psi_des[1]
        # xdes[k] = 13.75 # (CONSTANT)
        # ydes[k] = 12 # (CONSTANT)

        # Collect data from the fingers
        KEx_fng1, KEy_fng1 = KE_fng(fng1, mfng)
        KEx_fng2, KEy_fng2 = KE_fng(fng2, mfng)
        # xfng[k] = fng1.worldCenter[0]
        # yfng[k] = fng1.worldCenter[1]

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

        # Integrate the Cost function
        # print cnt_reward
        fx = C1*KEx_fng1+ C2*mfng*np.abs(xfng[k]-xdes[k])**2 + fx
        fy = C1*KEy_fng1+ C2*mfng*np.abs(yfng[k]-ydes[k])**2 + fy
        # print "KEx: " + str(KEx_fng1) + ", KEy: " + str(KEy_fng1)

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
    # print 'xdes', xdes
    # print 'xfng', xfng
    return xfng, yfng, xdes, ydes, fx, fy


def ConvexifyandSolve(x,y,xdes,ydes,Wk,A):
    fx_grad = Bk*np.matrix(x) + 2*C2*(np.matrix(x) - np.matrix(xdes))
    # print 'fng energy', Bk*np.matrix(x)
    # print 'distance', 2*C2*np.abs(np.matrix(x) - np.matrix(xdes))
    fy_grad = Bk*np.matrix(y) + 2*C2*(np.matrix(y) - np.matrix(ydes))
    # print fx_grad
    # print fy_grad
    
    vx = np.gradient(x, axis=0)
    vy = np.gradient(y, axis=0)

    # Convex Optimization
    Q  = matrix(Wk)
    px = matrix(fx_grad)
    py = matrix(fy_grad)
    G  = matrix(np.vstack((A,-A,I,-I)))
    hx = matrix(np.vstack((v_max - vx, v_max + vx, s*np.ones((2*N,1)))))
    hy = matrix(np.vstack((v_max - vy, v_max + vy, s*np.ones((2*N,1)))))
  
    a = matrix(np.identity(N)[0],(1,N))
    # print 'a',a
    b = matrix(np.zeros(1))

    solx = solvers.qp(Q, px, G, hx, a, b)
    soly = solvers.qp(Q, py, G, hy, a, b)

    x_tilda = np.matrix(solx['x']) + x
    y_tilda = np.matrix(soly['x']) + y
    # print 'x_tilda', x_tilda
    # print 'y_tilda', y_tilda

    return x_tilda, y_tilda, fx_grad, fy_grad

def MeritFunctions(fx, fy, fx_tilda, fy_tilda, fx_grad, fy_grad, x, y, x_tilda, y_tilda):
    phi = fx + fy
    phi_tilda = fx_tilda + fy_tilda
    px = np.matrix(x_tilda-x)
    pxT = np.transpose(px)
    py = np.matrix(y_tilda-y)
    pyT = np.transpose(py)
    phi_tilda_hat = fx + fy + pxT*fx_grad + pyT*fy_grad + 0.5*pxT*Wk*px + 0.5*pyT*Wk*py

    TrueImprove  = phi - phi_tilda
    ModelImprove = phi - phi_tilda_hat
    # print "phi" + str(phi) + ", phi_tilda: " + str(phi_tilda) + ", phi_tilda_hat: " + str(phi_tilda_hat)
    # print "True Improve" + str(TrueImprove) + ", Model Improve: " + str(ModelImprove[0,0])
    return TrueImprove, ModelImprove[0,0]

def plot_pos(x0, y0, xf, yf):
    # import matplotlib.pyplot as plt
    t = np.transpose(np.matrix(range(0, N)))*1/TARGET_FPS
    plt.figure(1)
    plt.subplot(211)
    plt.plot(t, x0, 'b--', t, xf,'b')
    plt.title("Position")
    plt.xlabel('time [s]')
    plt.ylabel('x-dir position [m]')
    plt.legend(['initial','final']) 

    plt.subplot(212)
    plt.plot(t, y0, 'r--',t, yf,'r')
    plt.xlabel('time [s]')
    plt.ylabel('y-dir position [m]')
    plt.legend(['initial','final'])

def plot_vel(x0, y0, xf, yf):
    # import matplotlib.pyplot as plt
    t = np.transpose(np.matrix(range(0, N)))*1/TARGET_FPS
    vx0 = np.gradient(x0, axis=0)*TARGET_FPS
    vxf = np.gradient(xf, axis=0)*TARGET_FPS
    vy0 = np.gradient(y0, axis=0)*TARGET_FPS
    vyf = np.gradient(yf, axis=0)*TARGET_FPS

    plt.figure(2)
    plt.subplot(211)
    plt.plot(t, vx0, 'b--', t, vxf,'b')
    plt.title("Velocity")
    plt.xlabel('time [s]')
    plt.ylabel('x-dir vel [m/s]')
    plt.legend(['initial','final']) 

    plt.subplot(212)
    plt.plot(t, vy0, 'r--',t, vyf,'r')
    plt.xlabel('time [s]')
    plt.ylabel('y-dir vel [m/s]')
    plt.legend(['initial','final'])
###########################The l1QP Algorithm####################################
C1 = 1
C2 = 10
mfng = 1
TARGET_FPS = 60
N = 181
A = -1*np.identity(N) + 1*np.eye(N, k=1)
I = np.identity(N)
# A = -np.vstack((np.identity(N)[0],A))
# A = -A
Bk = (2*C1*mfng)*np.identity(N) + (-C1*mfng)*np.eye(N, k=1) + (-C1*mfng)*np.eye(N, k=-1)
Bk[0,0] = C1*mfng
Bk[N-1,N-1] = C1*mfng
# print A.shape
# print Bk.shape

# INITIALIZE WITH THE NAIVE SOLUTION
xfng0, yfng0, xdes, ydes, fx, fy = simulate("naive",0)
# print 'fx0', fx
# print 'fy0', fy
# print yfng0

# psi = np.concatenate((xfng0, yfng0), axis=1)
# xfng1, yfng1, xdes, ydes, fx, fy = simulate("something else", psi)
# print 'fx1', fx
# print 'fy1', fy
# print yfng1-yfng0

# Parameters
mu0 = 10 # initial penalty coeff 
s0  = 1.0  # intial trust region size
Wk = Bk + 2*C2*mfng*np.identity(N)
# Wk = Bk
c   = 0.3 # fraction of TrueImprove/ModelImprove
k   = 1.2 # penalty scaling factor
tau_plus  = 1.10 # trust region expansion
tau_minus = 0.50 # trust region shrinkage
f_tol     = 2000  # acceptable cost from cost function
x_tol     = 0.02  # smallest trust region size
c_tol     = 0.1   # constraint satisfaction threshold

# Start
x = xfng0
y = yfng0
mu = mu0
s = s0
v_max = 0.150*np.ones((N,1))

# v_max = np.vstack((np.zeros(1),v_max))


for PenaltyIteration in range(0,5):
    # Convexify Iteration
    for ConvexifyIteration in range(0,10):
        # Convexify this problem and come up with a candidate soln

        x_tilda, y_tilda, fx_grad, fy_grad = ConvexifyandSolve(x,y,xdes,ydes,Wk,A)
        # print x_tilda
        psi_tilda = np.concatenate((x_tilda, y_tilda), axis=1)

        for TrustRegionTeration in range(0,10):
            xfng,yfng,xdes_tilda, ydes_tilda, fx_tilda, fy_tilda = simulate("something else", psi_tilda)
            print 'fx_tilda', fx_tilda
            print 'fx',fx
            print 'fy_tilda', fy_tilda
            print 'fy',fy

            # print x_tilda-xfng
            # print y_tilda-yfng
            # Check for Improvement
            TrueImprove, ModelImprove = MeritFunctions(fx, fy, fx_tilda, fy_tilda, fx_grad, fy_grad, x, y, x_tilda, y_tilda)
            print 'TrueImprove', TrueImprove
            print 'ModelImprove', ModelImprove
            # raw_input()

            if ModelImprove == 0:
                if TrueImprove > 0:
                    s = tau_plus*s
                    x = x_tilda
                    y = y_tilda
                    fx = fx_tilda
                    fy = fy_tilda
                    xdes = xdes_tilda
                    ydes = ydes_tilda
                    print "expand s"
                    break
                else:
                    s = tau_minus*s
                    print "shrink s"
            else:
                if TrueImprove/ModelImprove > c and TrueImprove>0 and ModelImprove>0:
                    s = tau_plus*s
                    x = x_tilda
                    y = y_tilda
                    fx = fx_tilda
                    fy = fy_tilda
                    xdes = xdes_tilda
                    ydes = ydes_tilda
                    print "expand s"
                    break
                else:
                    s = tau_minus*s
                    print "shrink s"
            # What happens if s gets to small? (change the coefficient mu)
            if s < x_tol:
                print "s has become too small"
                break
            # check if the solution, f_x_tilda is satifactory
        if fx+fy < f_tol:
            print "f is satisfy, the value is ", fx+fy
            break
    # Check that constraints are satified
    constraint_costs = 0
    # print constraint_costs
    if constraint_costs < c_tol:     
        break
    else:
        mu = k*mu
        s = s0

# print 'x',x
# print 'y',y
psi = np.concatenate((x, y), axis=1)
print 'begin simulation'
_,_,_,_, fx, fy = simulate("show", psi)

# PLOT
plot_pos(xfng0, yfng0, x, y)
plot_vel(xfng0, yfng0, x, y)
plt.show()