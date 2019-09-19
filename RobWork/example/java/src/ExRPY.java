import org.robwork.LoaderRW;
import org.robwork.rw.*;
import static org.robwork.rw.rwConstants.*;

public class ExRPY {
    public static void main(String[] args) throws Exception {
        LoaderRW.load("rw");

        RPYd rpy = new RPYd(Pi,Pi/2,0);
        System.out.println("RPY: " + rpy);
        Rotation3d rotationFromRPY = rpy.toRotation3D();
        System.out.println("Rotation from RPY: " + rotationFromRPY);

        Rotation3d rot = new Rotation3d(-1,0,0,0,0,1,0,1,0);
        System.out.println("Rotation: " + rot);
        RPYd rpyFromRotation = new RPYd(rot);
        System.out.println("RPY from Rotation: " + rpyFromRotation);
    }
}
