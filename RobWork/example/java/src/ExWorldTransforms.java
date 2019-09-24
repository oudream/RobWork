import java.util.Iterator;

import org.robwork.rw.*;

public class ExWorldTransforms {
    public static Transform3dVector worldTransforms(
            FrameVector frames, State state)
    {
        FKTable fk = new FKTable(state);

        Transform3dVector result = new Transform3dVector();
        Iterator<Frame> iterator = frames.iterator();
        while(iterator.hasNext()) {
            Frame frame = iterator.next();
            result.add(fk.get(frame));
        }
        return result;
    }
}