require("sdurw")
using("sdurw")
require("sdurw_math")
using("sdurw_math")

quat = Quaterniond(math.sqrt(2)/2,math.sqrt(2)/2,0,0);
print("Quaternion: " .. tostring(quat))
rotationFromQuat = quat:toRotation3D();
print("Rotation from Quaternion: " .. tostring(rotationFromQuat));

rot = Rotation3Dd(-1,0,0,0,0,1,0,1,0);
print("Rotation: " .. tostring(rot));
quatFromRotation = Quaterniond(rot);
print("Quaternion from Rotation: " .. tostring(quatFromRotation))