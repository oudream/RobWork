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

generate_template_pointer_converters(
    "rw::trajectory::Trajectory"
    TYPES
    "rw::trajectory::InterpolatorTrajectory"
    "rw::trajectory::TrajectorySequence"
    "int"
    TEMPLATES
    "rw::math::Q"
    "rw::math::Vector3D<double>"
    "rw::math::Vector2D<double>"
    "rw::math::Quaternion<double>"
    CONVERTER
    "toTrajectory"
)

generate_template_pointer_converters(
    "rw::trajectory::Blend"
    TYPES
    "rw::trajectory::ParabolicBlend"
    "rw::trajectory::LloydHaywardBlend"
    "int"
    TEMPLATES
    "rw::math::Q"
    "rw::math::Vector3D<double>"
    "rw::math::Vector2D<double>"
    "rw::math::Quaternion<double>"
    CONVERTER
    "toBlend"
)

generate_template_pointer_converters(
    "rw::trajectory::Interpolator"
    TYPES
    "rw::trajectory::LinearInterpolator"
    "rw::trajectory::RampInterpolator"
    "rw::trajectory::CubicSplineInterpolator"
    "int"
    TEMPLATES
    "rw::math::Q"
    "rw::math::Vector3D<double>"
    "rw::math::Vector2D<double>"
    "rw::math::Quaternion<double>"
    CONVERTER
    "toInterpolator"
)

generate_template_pointer_converters("rw::pathplanning::QConstraint" CONVERTER "toQConstraint")

generate_pointer_converters(
    "rwlibs::pathoptimization::ClearanceCalculator" TYPES
    "rwlibs::pathoptimization::MinimumClearanceCalculator" CONVERTER "toClearanceCalculator"
)

generate_pointer_converters("rw::models::ParallelDevice" CONVERTER "toParallelDevice")
