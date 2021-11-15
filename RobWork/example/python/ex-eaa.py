from sdurw_math import *
import math

if __name__ == '__main__':
    eaa = EAA(math.sqrt(2)/2*math.pi,math.sqrt(2)/2*math.pi,0)
    print("EAA: " + str(eaa))
    print(" angle: " + str(eaa.angle()))
    print(" axis: " + str(eaa.axis()))
    rotationFromEAA = eaa.toRotation3D()
    print("Rotation from EAA: " + str(rotationFromEAA))

    rot = Rotation3D(-1,0,0,0,0,1,0,1,0)
    print("Rotation: " + str(rot))
    eaaFromRotation = EAA(rot)
    print("EAA from Rotation: " + str(eaaFromRotation))
    print(" angle: " + str(eaaFromRotation.angle()))
    print(" axis: " + str(eaaFromRotation.axis()))