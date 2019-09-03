require("rw")
require("rw_pathplanners")
require("rw_proximitystrategies")
using("rw")
using("rw_pathplanners")
using("rw_proximitystrategies")

if #arg ~= 1 then
    print("Usage: lua ex-motionplanning.lua <path/to/RobWorkData>")
    return 1
end

local WC_FILE = "/scenes/SinglePA10Demo/SinglePA10DemoGantry.wc.xml"

local wc = WorkCellLoaderFactory.load(arg[1] .. WC_FILE)
if wc:isNull() then
    error("WorkCell could not be loaded")
end
local gantry = wc:findDevice("Gantry")
local pa10 = wc:findDevice("PA10")
if gantry:isNull() then
    error("Gantry device could not be found.")
end
if pa10:isNull() then
    error("PA10 device could not be found.")
end

local state = wc:getDefaultState()
local device = ownedPtr(CompositeDevice(gantry:getBase(), wc:getDevices(),
                        pa10:getEnd(), "Composite", state))

local cdstrategy = ProximityStrategyFactory.makeCollisionStrategy("PQP")
if cdstrategy:isNull() then
    error("PQP Collision Strategy could not be found.")
end
local collisionDetector = ownedPtr(CollisionDetector(wc, cdstrategy))
local con = PlannerConstraint.make(collisionDetector, device:asDeviceCPtr(), state)
local planner = RRTPlanner.makeQToQPlanner(con, device:asDevicePtr())

local beg = Q(9, -0.67, -0.79, 2.8, -0.02, -1.01, -0.26, -0.77, -1.01, 0)
local fin = Q(9, 0.57, 0.79, -1.23, 0.21, -0.63, -0.55, -0.07, -1.08, 0)

local pdata = ProximityData()
device:setQ(beg, state)
if collisionDetector:inCollision(state, pdata) then
    error("Initial configuration in collision!")
end
device:setQ(fin, state)
if collisionDetector:inCollision(state, pdata) then
    error("Final configuration in collision!")
end

local result = PathQ()
if planner:query(beg, fin, result) then
    print("Planned path with " .. result:size() .. " configurations")
end
