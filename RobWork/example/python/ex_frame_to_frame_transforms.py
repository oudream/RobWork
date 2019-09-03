from rw import *

def frameToFrameTransforms(a, b, tree_structure, states):
    fk = FKRange(a, b, tree_structure);
    
    result = Transform3dVector();
    for state in states:
        result.push_back(fk.get(state));
    return result;
