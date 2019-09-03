from rw import *
import math

if __name__ == '__main__':
    quat = Quaterniond(math.sqrt(2)/2,math.sqrt(2)/2,0,0)
    print("Quaternion: " + str(quat))
    rotationFromQuat = quat.toRotation3D()
    print("Rotation from Quaternion: " + str(rotationFromQuat));

    rot = Rotation3d(-1,0,0,0,0,1,0,1,0);
    print("Rotation: " + str(rot));
    quatFromRotation = Quaterniond(rot);
    print("Quaternion from Rotation: " + str(quatFromRotation));