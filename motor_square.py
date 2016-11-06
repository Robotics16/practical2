import brickpi
import time
import motor_util
import sys
import random
import math

interface=brickpi.Interface()
#interface.startLogging('logfile.txt')
interface.initialize()

# Setup motors
motors = [0,3]
speed = 5
interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
motorParams = interface.MotorAngleControllerParameters()
motor_util.set_pid(motorParams)
interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

# Setup initial state of particles
numberOfParticles = 100
particles = [(300, 300, 0) for i in range(numberOfParticles)]

def drawParticles(particles):
    print "drawParticles:" + str(particles)


def updateParticlesForward(particles):
    """Update all particles with 10cm forward motion, 
    based on current theta for each."""
    particles = map(updateOneParticleForward, particles)
    return particles


def updateParticlesRotateRight(particles):
    """Update all particles with 90 degrees rotation to the right"""
    particles = map(updateOneParticleRotateRight, particles)
    return particles

def updateOneParticleForward(particle):
    """Update particle triple with 10cm forward motion,
    use gaussian distribution with mu=0 and estimated sigma"""
    (old_x, old_y, old_theta) = particle
    d = 10 # forward motion 10 cms

    sigma_offset = 1   # estimated sigma 2cm2
    sigma_rotation = 0.2
    mu = 0

    e = random.gauss(mu, sigma_offset) # error term for coordinate offset
    f = random.gauss(mu, sigma_rotation) # error term for rotation
    
    particle = (old_x + (d + e) * math.cos(old_theta),
                old_y + (d + e) * math.sin(old_theta),
                old_theta + f)
    
    return particle

def updateOneParticleRotateRight(particle):
    """Update particle triple with 10cm forward motion,
    use gaussian distribution with mu=0 and estimated sigma"""
    (old_x, old_y, old_theta) = particle
    
    sigma_rotation = 0.2
    mu = 0
    g =0 #random.gauss(mu, sigma_rotation) # error term for pure rotation
    
    angle = -math.pi / 2
    
    particle = (old_x, old_y, old_theta + angle + g)
    return particle


# Go in squares
for i in range(4):
    for j in range(4):
        #motor_util.move10cm(interface, motors)
        particles = updateParticlesForward(particles)
        drawParticles(particles)
        time.sleep(0.25)
    #motor_util.rotateRight90deg(interface, motors)
    particles = updateParticlesRotateRight(particles)
    drawParticles(particles)
    time.sleep(0.25)

print "Destination reached!"

#interface.stopLogging('logfile.txt')
interface.terminate()
