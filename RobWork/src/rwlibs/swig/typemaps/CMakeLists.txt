include(TypeMapMacros.cmake)

generate_pointer_converters(
    "rw::kinematics::Frame"
    TYPES
    "rw::kinematics::FixedFrame"
    "rw::kinematics::MovableFrame"
    "rw::models::Joint"
    "rw::models::DependentJoint"
    "rw::models::RevoluteJoint"
    "rw::models::PrismaticJoint"
    "rw::models::PrismaticSphericalJoint"
    "rw::models::PrismaticUniversalJoint"
    "rw::models::SphericalJoint"
    "rw::models::UniversalJoint"
    "rw::models::VirtualJoint"
    "int"
    CONVERTER
    "toFrame"
)


generate_pointer_converters(
    "rw::kinematics::StateData"
    TYPES
    "rw::kinematics::Frame"
    "rw::kinematics::FixedFrame"
    "rw::kinematics::MovableFrame"
    "rw::models::Joint"
    "rw::models::DependentJoint"
    "rw::models::PrismaticJoint"
    "rw::models::PrismaticSphericalJoint"
    "rw::models::PrismaticUniversalJoint"
    "rw::models::RevoluteJoint"
    "rw::models::SphericalJoint"
    "rw::models::UniversalJoint"
    "rw::models::VirtualJoint"
    "int"
    CONVERTER
    "toStateData"
)

generate_pointer_converters(
    "rw::models::Device"
    TYPES
    "rw::models::JointDevice"
    "rw::models::MobileDevice"
    "rw::models::SE3Device"
    "rw::models::CompositeDevice"
    "rw::models::CompositeJointDevice"
    "rw::models::ParallelDevice"
    "rw::models::SerialDevice"
    "rw::models::TreeDevice"
    CONVERTER
    "toDevice"
)

generate_pointer_converters(
    "rw::models::Joint"
    TYPES
    "rw::models::DependentJoint"
    "rw::models::PrismaticJoint"
    "rw::models::PrismaticSphericalJoint"
    "rw::models::PrismaticUniversalJoint"
    "rw::models::RevoluteJoint"
    "rw::models::SphericalJoint"
    "rw::models::UniversalJoint"
    "rw::models::VirtualJoint"
    "int"
    CONVERTER
    "toJoint"
)

generate_pointer_converters(
    "rw::trajectory::Trajectory<rw::math::Q>"
    TYPES
    "rw::trajectory::InterpolatorTrajectory<rw::math::Q>"
    "int"
    CONVERTER
    "toTrajectoryQ"
)

generate_pointer_converters(
    "rw::trajectory::Blend< rw::math::Vector2D<double>>"
    TYPES
    "rw::trajectory::ParabolicBlend< rw::math::Vector2D<double>>"
    "int"
    CONVERTER
    "toBlendVector2D"
)

generate_pointer_converters(
    "rw::trajectory::Interpolator< rw::math::Vector2D<double>>"
    TYPES
    "rw::trajectory::LinearInterpolator< rw::math::Vector2D<double>>"
    "rw::trajectory::RampInterpolator < rw::math::Vector2D<double>>"
    "rw::trajectory::CubicSplineInterpolator< rw::math::Vector2D<double>>"
    "int"
    CONVERTER
    "toInterpolatorVector2D"

)

generate_pointer_converters(
    "rw::pathplanning::QConstraint"
    CONVERTER
    "toQConstraint"
)

generate_pointer_converters(
    "rwlibs::pathoptimization::ClearanceCalculator"
    TYPES
    "rwlibs::pathoptimization::MinimumClearanceCalculator"
    CONVERTER
    "toClearanceCalculator"
)

generate_pointer_converters(
    "rw::models::ParallelDevice"
    CONVERTER
    "toParallelDevice"
)