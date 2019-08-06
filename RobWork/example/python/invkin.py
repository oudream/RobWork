from rw import *

WC_FILE = "/path/to/RobWorkData/devices/serialdev/UR10e_2018/UR10e.xml"

if __name__ == '__main__':
    wc = WorkCellLoaderFactory.load(WC_FILE)
    if wc.isNull():
        raise Exception("WorkCell could not be loaded")
    device = wc.findSerialDevice("UR10e")
    if device.isNull():
        raise Exception("UR10e device could not be found.")

    defState = wc.getDefaultState()
    solver = ClosedFormIKSolverUR(device.asSerialDeviceCPtr(), defState)

    Tdesired = Transform3d(Vector3d(0.2, -0.2, 0.5), EAAd(0, Pi, 0).toRotation3D())
    solutions = solver.solve(Tdesired, defState)

    print("Inverse Kinematics for " + device.getName() + ".")
    print(" Base frame: " + device.getBase().getName())
    print(" End/TCP frame: " + solver.getTCP().getName())
    print(" Target Transform: " + str(Tdesired))
    print("Found " + str(solutions.size()) + " solutions.")
    for solution in solutions:
        print(" " + str(solution))
