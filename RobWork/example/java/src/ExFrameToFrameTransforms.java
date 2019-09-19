import java.util.Iterator;

import org.robwork.rw.*;

public class ExFrameToFrameTransforms {
    public static Transform3dVector frameToFrameTransforms(
            Frame a, Frame b, State tree_structure, StateVector states)
    {
        FKRange fk = new FKRange(a, b, tree_structure);

        Transform3dVector result = new Transform3dVector();
        Iterator<State> iterator = states.iterator();
        while(iterator.hasNext()) {
            State state = iterator.next();
            result.add(fk.get(state));
        }
        return result;
    }
}