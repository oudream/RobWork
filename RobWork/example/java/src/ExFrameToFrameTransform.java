import org.robwork.sdurw.*;

public class ExFrameToFrameTransform {
    public static Transform3d frameToFrameTransform(
            Frame a, Frame b, State state)
    {
        FKRange fk = new FKRange(a, b, state);
        return fk.get(state);
    }
}
