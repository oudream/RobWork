from rw import *

def worldTransforms(frames, state):
    fk = FKTable(state);
    
    result = Transform3dVector();
    for frame in frames:
        result.push_back(fk.get(frame));
    return result;