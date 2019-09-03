from rw import *

def printDeviceNames(workcell):
    print("Workcell " + workcell.getName() + " contains devices:");
    for device in workcell.getDevices():
        print("- " + device.getName())