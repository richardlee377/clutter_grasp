import Box2D  # The main library
from Box2D.b2 import (world, edgeShape, staticBody, dynamicBody, rayCastInput, rayCastOutput, vec2)
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
import math
from math import (sin, cos)
import numpy as np

def PG(ph, a, d):
    theta = ph[2]
    phi   = math.pi/2 - theta 
    neg_area = world.CreateStaticBody(position=(ph[0]+0.5*d*cos(theta), ph[1]+0.5*d*sin(theta)), angle=theta)
    na_fixt = neg_area.CreatePolygonFixture(box=(d, a), density=1, friction=1)
    shape = na_fixt.shape
    vertices = [(neg_area.transform * v) * PPM for v in shape.vertices]
    return vertices
# 
# --- pygame setup ---
PPM = 20.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

# --- pygame setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

# --- pybox2d world setup ---
# Create the world
world = world(gravity=(0, 0), doSleep=True)


# test PG
for d in np.arange(0,10,1):
    for a in range(9):
        for th in range(18):
            ph = [15, 5, math.pi/18*th]
            vertices = PG(ph, a/8*5,d)
            vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]
            print vertices

            pygame.draw.polygon(screen, (255, 50, 50, 255), vertices)

            world.Step(TIME_STEP, 10, 10)
            pygame.display.flip()
            clock.tick(TARGET_FPS)

pygame.quit()
print('Simulation Done!')