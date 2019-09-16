require("rw")
using("rw")

rotd = Rotation3d(1,0,0,0,0,-1,0,1,0);
rotf = Rotation3f(1,0,0,0,0,-1,0,1,0);

print("Rotation double:");
print(tostring(rotd));
print("Rotation float:");
print(tostring(rotf));
print("Rotation inverse:");
print(tostring(inverse(rotd)));
print("Identity:");
print(tostring(rotd*inverse(rotd)));