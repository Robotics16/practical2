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
startPos = 300
particles = [((startPos, startPos, 0), 1 / float(numberOfParticles)) for i in range(numberOfParticles)]

def drawParticles(particles):
    print "drawParticles:" + str([p[0] for p in particles])

def updateParticlesForward(particles, d):
    """Update all particles with 10cm forward motion, 
    based on current theta for each."""
	
    for i in range(0, len(particles)):
	updatedParticle = updateOneParticleForward(particles[i], d)
	particles[i] = updatedParticle

    return particles

def updateParticlesRotate(particles, angle):
    """Update all particles with a certain angle"""

    for i in range(0, len(particles)):
    	updatedParticle = updateOneParticleRotate(particles[i], angle)
    	particles[i] = updatedParticle

    return particles

def updateOneParticleForward(particle, d):
    """Update particle triple with 10cm forward motion,
    use gaussian distribution with mu=0 and estimated sigma"""
    ((old_x, old_y, old_theta), w) = particle

    sigma_offset = 1   # estimated sigma 2cm2 TODO
    sigma_rotation = 0.2 # TODO
    mu = 0

    e = random.gauss(mu, sigma_offset) # error term for coordinate offset
    f = random.gauss(mu, sigma_rotation) # error term for rotation
    
    particle = ((old_x + (d + e) * math.cos(old_theta),
                old_y + (d + e) * math.sin(old_theta),
                old_theta + f), w) # TODO keep angle between -pi and pi
    
    return particle

def updateOneParticleRotate(particle, angle):
    """Update particle triple with 10cm forward motion,
    use gaussian distribution with mu=0 and estimated sigma"""
    ((old_x, old_y, old_theta), w) = particle
                
    sigma_rotation = 0.2 # TODO 
    mu = 0
    g = random.gauss(mu, sigma_rotation) # error term for pure rotation
    
    particle = ((old_x, old_y, old_theta + angle + g), w) # TODO keep angle between -pi and pi
    return particle 


def getCurrentLocation(particles):
    """Given all particles returns an estimate of the 
    current position (x, y, theta)"""
    estimates = [(x * weight, y * weight, theta * weight) for ((x, y, theta), weight) in particles]
    x_estimate     = sum([e[0] for e in estimates])
    y_estimate     = sum([e[1] for e in estimates])
    theta_estimate = sum([e[2] for e in estimates])
    
    return (x_estimate, y_estimate, theta_estimate)

def navigateToWaypoint(w_x, w_y, particles):
    """Using the current location returned by getCurrentLocation()
    navigates the robot to (w_x, w_y) (coordinates in the Wold coordinate system)"""  
    (x, y, theta) = getCurrentLocation(particles)
    
    # Get vector between current and next position
    (d_x, d_y) = (w_x - x, w_y - y)
    
    # Turn on the spot in the direction of the waypoint
    alpha = math.atan2(d_y, d_x) # absolute orientation needed (using atan2 so that the result is between -pi and pi)
    beta  = (alpha - theta) # angle to turn
    if (abs(beta) > math.pi) :
    	if (beta > 0) :
            beta = beta - 2 * math.pi
        else :
            beta = beta + 2 * math.pi
       
    motor_util.rotate(-beta, interface, motors)
    particles = updateParticlesRotate(particles, beta)
    print "Waypoint navigate: Rotate: " + str(beta)
    
    # Move straight forward to waypoint
    distance_to_move = math.sqrt(d_x ** 2 + d_y ** 2) # distance to move using the Pythagorean theorem
    print "Waypoint navigate: go: " + str(distance_to_move)
    motor_util.forward(distance_to_move, interface, motors)
    particles = updateParticlesForward(particles, distance_to_move)

    (x, y, theta) = getCurrentLocation(particles)
    print "Current loc: " + str((x-startPos, y-startPos, theta))


"""# Go in squares
for i in range(4):
    for j in range(4):
        motor_util.forward(10, interface, motors)
        particles = updateParticlesForward(particles)
        getCurrentLocation(particles)
        drawParticles(particles)
        time.sleep(0.25)
    motor_util.rotateRight90deg(interface, motors)
    particles = updateParticlesRotateLeft(particles)
    getCurrentLocation(particles)
    drawParticles(particles)
    time.sleep(0.25)"""

while(True):
	w_x = float(input("Enter your desired Wx position: "))
	w_y = float(input("Enter your desired Wy position: "))
	navigateToWaypoint(w_x+startPos, w_y+startPos, particles) 

print "Destination reached!"

#interface.stopLogging('logfile.txt')
interface.terminate()
