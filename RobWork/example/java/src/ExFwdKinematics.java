import org.robwork.rw.*;
import static org.robwork.rw.rw.inverse;

public class ExFwdKinematics {
    public static void fwdKinematics(SerialDevicePtr sdevice,
            Frame frame, MovableFrame mframe, State state)
    {
        // calculate the transform from one frame to another
        Transform3d fTmf = Kinematics.frameTframe(frame, mframe, state);
        // calculate the transform from world to frame
        Transform3d wTmf = Kinematics.worldTframe( mframe, state );
        // we can find the world to frame transform by a little jogling
        Transform3d wTf = wTmf.multiply(inverse(fTmf));
        // test if frame is a dynamic attachable frame
        if( Kinematics.isDAF( mframe ) ){
           // attach mframe to end of serial device
           Kinematics.gripFrame(mframe, sdevice.getEnd(), state);
        }
    }
}