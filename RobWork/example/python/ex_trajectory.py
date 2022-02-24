#################################################################################
# Example of a trajectory defined as a sequence of interpolators and blends.
#################################################################################

import sdurw_core
import sdurw_math
import sdurw_trajectory


# The range function does not work with floats. Only integer values can be specified as the start, stop, and step arguments.
# This function should rander the problem.
def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step


def trajectory():
    T1 = sdurw_math.Transform3D(sdurw_math.Vector3D(0,0,0) , sdurw_math.EAA(0,0,0))
    T2 = sdurw_math.Transform3D(sdurw_math.Vector3D(1,1,0) , sdurw_math.EAA(1,1,0))
    T3 = sdurw_math.Transform3D(sdurw_math.Vector3D(2,2,0) , sdurw_math.EAA(2,2,0))

    cartInt1 = sdurw_trajectory.LinearInterpolatorTransform3D(T1, T2, 1)
    cartInt2 = sdurw_trajectory.LinearInterpolatorTransform3D(T2, T3, 1)

    blend1 = sdurw_trajectory.ParabolicBlendTransform3D(cartInt1, cartInt2, 0.25)

#    traj = sdurw_trajectory.LinearInterpolatorTransform3D()
#    traj = sdurw_trajectory.InterpolatorTrajectory_d()
    traj = sdurw_trajectory.InterpolatorTrajectoryTransform3D()
#    traj = sdurw_trajectory.InterpolatorTrajectoryQ()
    

#    traj.add(cartInt1)
#    traj.add(blend1, cartInt2)

    dt = 0.01
    for t in range_with_floats(0, traj.duration(), dt):
        x = traj.x(t)
        print("\n ", t, ";", t, "; ", x.P()[0], "; ", x.P()[1], "; ", x.P()[2])



if __name__ == '__main__':
    trajectory()



# Transform3D<> T1(Vector3D<>(0,0,0), EAA<>(0,0,0));
# Transform3D<> T2(Vector3D<>(1,1,0), EAA<>(1,1,0));
# Transform3D<> T3(Vector3D<>(2,0,0), EAA<>(2,2,0));

# LinearInterpolator<Transform3D<> >::Ptr cartInt1 =
#     ownedPtr(new LinearInterpolator<Transform3D<> >(T1, T2, 1));
# LinearInterpolator<Transform3D<> >::Ptr cartInt2 =
#     ownedPtr(new LinearInterpolator<Transform3D<> >(T2, T3, 1));
# ParabolicBlend<Transform3D<> >::Ptr blend1 =
#     ownedPtr(new ParabolicBlend<Transform3D<> >(cartInt1, cartInt2, 0.25));
# InterpolatorTrajectory<Transform3D<> > trajectory;
# trajectory.add(cartInt1);
# trajectory.add(blend1, cartInt2);
# std::ofstream out("test.dat");
# for (double t = 0; t<=trajectory.duration(); t += dt) {
#      Transform3D<> x = trajectory.x(t);
#      out<<t<<" "<<x.P()(0)<<" "<<x.P()(1)<<" "<<x.P()(2)<<std::endl;
# }
# out.close();
