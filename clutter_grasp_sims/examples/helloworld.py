from Box2D import (b2PolygonShape, b2World)

world = b2World(gravity=(0,-10), doSleep=True)

# Define the ground body.
groundBodyDef = b2BodyDef()
groundBodyDef.position = (0, -10)
# Make a body fitting this definition in the world.
groundBody = world.CreateBody(groundBodyDef)
groundBox = b2PolygonShape(box=(50,10))
# And create a fixture definition to hold the shape
groundBoxFixture = b2FixtureDef(shape=groundBox)

# Add the ground shape to the ground body.
groundBody.CreateFixture(groundBoxFixture)

body = world.CreateDynamicBody(position=(0,4))
box = body.CreatePolygonFixture(box=(1,1), density=1, friction=0.3)

# Prepare for simulation. Typically we use a time step of 1/60 of a
# second (60Hz) and 6 velocity/2 position iterations. This provides a 
# high quality simulation in most game scenarios.
timeStep = 1.0 / 60

vel_iters, pos_iters = 6, 2

