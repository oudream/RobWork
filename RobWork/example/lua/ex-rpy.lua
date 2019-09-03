require("rw")
using("rw")

rpy = RPYd(Pi,Pi/2,0);
print("RPY: " .. tostring(rpy));
rotationFromRPY = rpy:toRotation3D();
print("Rotation from RPY: " .. tostring(rotationFromRPY));

rot = Rotation3d(-1,0,0,0,0,1,0,1,0);
print("Rotation: " .. tostring(rot));
rpyFromRotation = RPYd(rot);
print("RPY from Rotation: " .. tostring(rpyFromRotation));