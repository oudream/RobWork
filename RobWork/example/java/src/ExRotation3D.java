import org.robwork.LoaderRW;
import org.robwork.rw.*;
import static org.robwork.rw.rw.*;

public class ExRotation3D {
    public static void main(String[] args) throws Exception {
        LoaderRW.load("rw");

        Rotation3d rotd = new Rotation3d(1,0,0,0,0,-1,0,1,0);
        Rotation3f rotf = new Rotation3f(1,0,0,0,0,-1,0,1,0);

        System.out.println("Rotation double:");
        System.out.println(rotd);
        System.out.println("Rotation float:");
        System.out.println(rotf);
        System.out.println("Rotation inverse:");
        System.out.println(rotd.inverse());
        System.out.println("Identity:");
        System.out.println(rotd.multiply(inverse(rotd)));
    }
}
