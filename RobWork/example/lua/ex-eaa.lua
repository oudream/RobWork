require("sdurw_core")
require("sdurw")
using("sdurw")
require("sdurw_math")
using("sdurw_math")

eaa = EAA(math.sqrt(2)/2*Pi,math.sqrt(2)/2*Pi,0);
print("EAA: " .. tostring(eaa))
print(" angle: " .. eaa:angle())
print(" axis: " .. tostring(eaa:axis()));
rotationFromEAA = eaa:toRotation3D();
print("Rotation from EAA: " .. tostring(rotationFromEAA));

rot = Rotation3D(-1,0,0,0,0,1,0,1,0);
print("Rotation: " .. tostring(rot));
eaaFromRotation = EAA(rot);
print("EAA from Rotation: " .. tostring(eaaFromRotation))
print(" angle: " .. eaaFromRotation:angle())
print(" axis: " .. tostring(eaaFromRotation:axis()));

