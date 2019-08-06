require("rw")

function openpackage (ns)
  for n,v in pairs(ns) do
    if _G[n] ~= nil then
      print("name clash: " .. n .. " is already defined")
    else
      _G[n] = v
    end
  end
end

openpackage(rw)

local WC_FILE = "/path/to/RobWorkData/devices/serialdev/UR10e_2018/UR10e.xml"

local wc = WorkCellLoaderFactory.load(WC_FILE)
if wc:isNull() then
    error("WorkCell could not be loaded")
end
local device = wc:findSerialDevice("UR10e")
if device:isNull() then
    error("UR10e device could not be found.")
end

local state = wc:getDefaultState()
local solver = ClosedFormIKSolverUR(device:asSerialDeviceCPtr(), state)

local Tdesired = Transform3d(Vector3d(0.2, -0.2, 0.5), EAAd(0, Pi, 0):toRotation3D())
local solutions = solver:solve(Tdesired, state)

print("Inverse Kinematics for " .. device:getName() .. ".")
print(" Base frame: " .. device:getBase():getName())
print(" End/TCP frame: " .. solver:getTCP():getName())
print(" Target Transform: " .. tostring(Tdesired))
print("Found " .. solutions:size() .. " solutions.")
for i= 0,solutions:size()-1,1
do
    print(" " .. tostring(solutions[i]))
end
