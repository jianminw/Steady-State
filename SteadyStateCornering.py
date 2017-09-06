# -*- coding: utf-8 -*-
"""
Created on Thu Apr  6 18:24:02 2017

@author: Jianming
"""

from TireForces import getForces
import math
# import matplotlib.pyplot as plt

# radius in meters
# vStep in km/hr
# wheelBase, trackWidth in meters
# mass in kg
# height of center of mass in meters
def maxVelocity(radius, vStep = 0.1, mass = 181 + 70,
                wheelBase = 1.53, trackWidth = 1.15, height = 0.25):
    g = 9.8
    pressure, camber = 70, 0
    velocity = 0
    thetaL, thetaR = getAngles(radius, wheelBase, trackWidth)
    
    neededSideForce = 0
    maxSideForce = 1
    while (neededSideForce <= maxSideForce):
        velocity += vStep
        acceleration = (velocity / 3.6)**2 / radius
        neededSideForce = acceleration * mass
        loadTransfer = neededSideForce * height / trackWidth
        downForce = getDownForce(velocity)
        leftTireLoad = mass * g / 2 * 0.47 - loadTransfer / 2 + downForce / 4
        rightTireLoad = mass * g / 2 * 0.47 + loadTransfer / 2 + downForce / 4
        # print(rightTireLoad)
        leftTireSideForce = getForces(pressure, camber, leftTireLoad, 
                                      thetaL, 0)
        rightTireSideForce = getForces(pressure, camber, rightTireLoad, 
                                       thetaR, 0)
        maxSideForce = abs (leftTireSideForce[1] + rightTireSideForce[1])
        # print(rightTireSideForce)
        # factor of two thirds from the contents guide in round 5 tire data
        # page 8, test comments
        maxSideForce *= 0.85
    return velocity - vStep

# velocity in km / h
# liftCoeff unitless
# frontArea in m^2
def getDownForce(velocity, frontArea = 1.1, liftCoeff = 3.34):
    rho = 1.225 # kg / m^3
    return rho * liftCoeff * frontArea * (velocity / 3.6)**2 / 2

def radToDeg(r):
    return r / math.pi * 180
    
def getAngles(radius, wheelBase, trackWidth):
    theta1 = math.atan(wheelBase / (radius - trackWidth / 2) )
    theta2 = math.atan(wheelBase / (radius + trackWidth / 2) )
    return radToDeg(theta1), radToDeg(theta2)

radiusOfCoG = 7.5 + 1.38 / 2
skidPadVelocity = maxVelocity(radiusOfCoG)
lapLength = radiusOfCoG * 2 * 3.1415926535
lapTime = lapLength / (skidPadVelocity / 3.6)
print(lapTime)