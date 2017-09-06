# -*- coding: utf-8 -*-
"""
Created on Sun Mar  5 11:01:25 2017

@author: Jianming Wang
"""

import scipy.io
import math
import matplotlib.pyplot as plt
import time

# Loading the data calculated in MatLab
SA_CoEff_File = scipy.io.loadmat("Pacejka_Interpolation_SA.mat")
SA_CoEff = SA_CoEff_File["coEff"]
SR_CoEff_File = scipy.io.loadmat("Pacejka_Interpolation_SR.mat")
SR_CoEff = SR_CoEff_File["coEff"]

def degToRad(x):
    return x * math.pi / 180

def newtonToPound(N):
    return N / 4.4475

def poundToNewton(lb):
    return lb * 4.4475

def kPaToPsi(kPa):
    return kPa * 0.145038

def matrixMultiplication(A, B):
    n1 = len(A)
    m1 = len(A[0])
    n2 = len(B)
    m2 = len(B[0])
    assert(m1 == n2)
    result = [([0]*m2) for i in range(n1)]
    for i in range(n1):
        for j in range(m2):
            for k in range(m1):
                result[i][j] += A[i][k] * B[k][j]
    return result

# pressure in psi, camber in degrees, force in pounds
def pacejkaCoEffsSA(pressure, camber, force):
    force /= 50
    p = [[0] * 100]
    for i in range(4):
        for j in range(5):
            for k in range(5):
                temp = pressure**i * camber**j * force**k
                index = i * 25 + j * 5 + k
                p[0][index] = temp
    return matrixMultiplication(p, SA_CoEff)[0]

# pressure in psi, camber in degrees, force in pounds, slipAngle in degrees
def pacejkaCoEffsSR(pressure, camber, force, slipAngle):
    force /= 50
    p = [[0] * 144]
    for i in range(4):
        for j in range(3):
            for k in range(4):
                for l in range(3):
                    temp = pressure**i * camber**j * force**k * slipAngle**l
                    index = i*3*4*3 + j*3*4 + k*3 + l
                    p[0][index] = temp
    return matrixMultiplication(p, SR_CoEff)[0]

def pacejkaFunction(c):
    def f(x):
        # artifact from using the same pacejka approximation for both
        # slip angle and slip ratio
        x = degToRad(x)
        temp = c[0]*x - math.atan(c[0]*x)
        return c[2]*math.sin(c[1]*math.atan(c[0]*x - c[3]*(temp)))
    return f

def pacejkaPlotSA(pressure, camber, force):
    coEff = pacejkaCoEffsSA(pressure, camber, force)
    function = pacejkaFunction(coEff)
    steps = 10000
    SA = [(-15 + 30 * i / steps) for i in range(steps)]
    FY = [(function(sa)) for sa in SA]
    plt.plot(SA, FY)
    plt.show()
    
def pacejkaPlotSR(pressure, camber, force, slipAngle):
    coEff = pacejkaCoEffsSR(pressure, camber, force, slipAngle)
    function = pacejkaFunction(coEff)
    steps = 10000
    SR = [(-0.5 + 1 * i / steps) for i in range(steps)]
    FX = [(function(sr)) for sr in SR]
    plt.plot(SR, FX)
    plt.show()
    
# "mask" use of imperial units
# conversions from SI
# pressure in kPA, camber in degrees, force in Newtons,
# slipAngle in degrees, and slipRatio dimensionless
# output all three forces in Newtons
# This should be the only function that is called outside of this file
def getForces(pressure, camber, force, slipAngle, slipRatio):
    pressure = kPaToPsi(pressure)
    force = newtonToPound(force)
    maxForce = poundToNewton(300)
    if force > maxForce:
        force = maxForce
    SA_coEff = pacejkaCoEffsSA(pressure, camber, force)
    SR_coEff = pacejkaCoEffsSR(pressure, camber, force, slipAngle)
    SA_function = pacejkaFunction(SA_coEff)
    SR_function = pacejkaFunction(SR_coEff)
    FX = SR_function(slipRatio)
    FY = SA_function(slipAngle)
    FZ = poundToNewton(force)
    return (FX, FY, FZ)

def maxFX(pressure, camber, force, slipAngle):
    pressure = kPaToPsi(pressure)
    force = newtonToPound(force)
    maxForce = poundToNewton(250)
    if force > maxForce:
        force = maxForce
    SR_coEff = pacejkaCoEffsSR(pressure, camber, force, slipAngle)
    return SR_coEff[2]

def timeGetForces(steps):
    print("Timing getForces()...")
    prange = [(8 + 6*i/steps) for i in range(steps)]
    crange = [(3*i/steps) for i in range(steps)]
    frange = [(50 + 200*i/steps) for i in range(steps)]
    sarange = [(-0.25 + 0.5 * i / steps) for i in range(steps)]
    srrange = [(-0.5 + 1 * i / steps) for i in range(steps)]
    T0 = time.time()
    print("Timer started: ", T0, "s")
    for p in prange:
        for c in crange:
            for f in frange:
                for sa in sarange:
                    for sr in srrange:
                        getForces(p, c, f, sa, sr)
    T1 = time.time()
    print("Timer ended: ", T1, "s")
    print("Time elapsed: ", (T1-T0)*1000 , "ms")
    print("Average time per call: ", (T1-T0)*1000/steps**5, "ms")
    
if __name__ == "__main__":
    pacejkaPlotSA(12, 0, newtonToPound(984))