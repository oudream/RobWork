require("sdurw_core")
require("sdurw_models")
require("sdurw")
require("sdurw_loaders")
require("sdurws")

using("sdurw")
using("sdurw_loaders")
using("sdurws")

if #arg < 1 then
    print("Usage : lua ex-load-workcell.lua <workcell> [OPTION]")
    print("Option: -t      for test run")
    return 1
end

local WC_FILE = arg[1] .. "/scenes/SinglePA10Demo/SinglePA10Demo.wc.xml"
print(WC_FILE)

local wc = WorkCellLoaderFactory.load(WC_FILE)
local rwstudio = getRobWorkStudioInstance();
rwstudio:setWorkCell(wc)

while(isRunning()) do
    if #arg == 2 and arg[2] == "-t" then
        rwstudio:postExit();
    end
    sleep(1);
end

sleep(10);
print("Example-Done");