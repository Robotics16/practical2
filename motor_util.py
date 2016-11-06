import brickpi
import time 

rotate90Rad = 6.45

def rotate(angle, interface, motors):
  interface.increaseMotorAngleReferences(motors,[-angle,angle])

  while not interface.motorAngleReferencesReached(motors) :
          motorAngles = interface.getMotorAngles(motors)
          if motorAngles :
                #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
                time.sleep(0.1)

  print "Destination reached!"

def move(angle, interface, motors):
  interface.increaseMotorAngleReferences(motors,[angle,angle])

  while not interface.motorAngleReferencesReached(motors) :
          motorAngles = interface.getMotorAngles(motors)
          if motorAngles :
                #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
                time.sleep(0.1)

  print "Destination reached!"

def set_pid(motorParams):
	motorParams.maxRotationAcceleration = 6.0
	motorParams.maxRotationSpeed = 12.0
	motorParams.feedForwardGain = 255/20.0
	motorParams.minPWM = 30.0
	motorParams.pidParameters.minOutput = -255
	motorParams.pidParameters.maxOutput = 255
	motorParams.pidParameters.k_p = 600.0
	motorParams.pidParameters.k_i = 1400
	motorParams.pidParameters.k_d = 20.625

def rotateLeft90deg(interface, motors):
	rotate(rotate90Rad, interface, motors)

def rotateRight90deg(interface, motors):
	rotate(-rotate90Rad, interface, motors)

def move40cm(interface, motors):
	move(14.5, interface, motors)

def move10cm(interface, motors):
	move(3.625, interface, motors)

def moveback40cm(interface, motors):
	move(-14.5, interface, motors)

def move5cm(interface, motors):
	move(1.5, interface, motors)  # TODO FIX ANGLE VAL

def moveback5cm(interface, motors):
	move(-1.5, interface, motors)  # TODO FIX ANGLE VAL


