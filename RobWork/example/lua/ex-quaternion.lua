require("sdurw_core")
require("sdurw")
using("sdurw_core")
require("sdurw_math")
using("sdurw_math")

quat = Quaternion(math.sqrt(2)/2,math.sqrt(2)/2,0,0);
print("Quaternion: " .. tostring(quat))
rotationFromQuat = quat:toRotation3D();
print("Rotation from Quaternion: " .. tostring(rotationFromQuat));

rot = Rotation3D(-1,0,0,0,0,1,0,1,0);
print("Rotation: " .. tostring(rot));
quatFromRotation = Quaternion(rot);
print("Quaternion from Rotation: " .. tostring(quatFromRotation))
