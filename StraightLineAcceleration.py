# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 18:45:25 2017

@author: Jianming
"""

# acceleration event
# 75 meters
# with and without aero
# assume constant torque of 240 Newton meters
# return time taken and final speeds

from TireForces import maxFX
import matplotlib.pyplot as plt

def acceleration(deltaT = 0.001, mass = 292.2 + 70, wheelBase = 1.65, 
                height = 0.246):
    g = 9.8
    pressure, camber = 70, 0
    wheelRadius = 0.0254 * 9 # meter
    velocity = 0 # meters per second
    acceleration = 0 # meter per second squared
    
    position = -0.3
    maxPosition = 75
    time = 0
    
    maxRPM = 6000
    gearRatio = 4.6
    effectiveRadius = wheelRadius / gearRatio
    effectiveCircum = 2 * 3.1415 * effectiveRadius
    maxSpeed = maxRPM * effectiveCircum / 60
    
    RRCoeff = 0.0025

    startTime = 0
    
    times = []
    powers = []
    velocities = []
    rpms = []
    
    while position < maxPosition:   
        force = acceleration * mass
        loadTransfer = force * height / wheelBase
        downForce = getDownForce(velocity)
        tireLoad = mass * g / 2 * 0.56 + loadTransfer / 2 + downForce / 4
        #print("tire load: ", tireLoad)
        maxForce = maxFX(pressure, camber, tireLoad, 0) * 2 * (2/3)
        rpm = velocity * 60 / effectiveCircum
        print("rpm: ", rpm)
        torque = getTorque(rpm)
        Fa = (torque / effectiveRadius)
        if (rpm > 2000):
            Fa = min(Fa, getPower(rpm) / velocity)
        #print("Engine force: ", Fa)
        #print("Max Force: ", maxForce)
        newForce = min(maxForce, Fa)
        newForce -= getDrag(velocity)
        newForce -= RRCoeff * (mass * g + downForce)
        if (rpm > maxRPM):
            newForce = 0 # assume that car can hit top speed and hold it. 
        #print("Net Force: ", newForce)
        acceleration = newForce / mass
        velocity += acceleration * deltaT
        #if (velocity > maxSpeed):
        #    raise Exception("Going too fast")
        position += velocity * deltaT
        print("Position: ", position)
        #print("Velocity: ", velocity)
        time += deltaT
        print("Time: ", time)
        if (position < 0):
            startTime = time
        times.append(time)
        powers.append(Fa * velocity / 1000)
        velocities.append(velocity)
        rpms.append(rpm)
    #plt.plot(rpms, powers)
    #plt.show()
    return time - startTime
        
def getDownForce(velocity, frontArea = 0.9, liftCoeff = 3):
    rho = 1.225 # kg / m^3
    return rho * liftCoeff * frontArea * (velocity)**2 / 2

def getDrag(velocity, frontArea = 0.9, dragCoeff = 1.5):
    rho = 1.225 # kg / m^3
    return rho * dragCoeff * frontArea * (velocity)**2 / 2

def getTorque(rpm):
    return 240

def getPower(rpm):
    return 100 * 1000

print(acceleration())