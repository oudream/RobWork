import org.robwork.LoaderRW;
import org.robwork.sdurw.*;
import static org.robwork.sdurw.sdurwConstants.*;
import java.lang.Math;

public class ExEAA {
    public static void main(String[] args) throws Exception {
        LoaderRW.load("sdurw");

        EAAd eaa = new EAAd(Math.sqrt(2)/2*Pi,Math.sqrt(2)/2*Pi,0);
        System.out.println("EAA: " + eaa);
        System.out.println(" angle: " + eaa.angle());
        System.out.println(" axis: " + eaa.axis());
        Rotation3d rotationFromEAA = eaa.toRotation3D();
        System.out.println("Rotation from RPY: " + rotationFromEAA);

        Rotation3d rot = new Rotation3d(-1,0,0,0,0,1,0,1,0);
        System.out.println("Rotation: " + rot);
        EAAd eaaFromRotation = new EAAd(rot);
        System.out.println("EAA from Rotation: " + eaaFromRotation);
        System.out.println(" angle: " + eaaFromRotation.angle());
        System.out.println(" axis: " + eaaFromRotation.axis());
    }
}
