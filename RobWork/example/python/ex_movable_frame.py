from sdurw import *
import math

if __name__ == '__main__':
    wc = WorkCellLoaderFactory.load("../ModelData/XMLScenes/RobotOnTable/Scene.xml")
    state = wc.getDefaultState()
    mf = wc.findMovableFrame("m_box")
    t = sdurw.Transform3d()
    print(state)
    print(t)
    mf.setTransform(t,state)