include(TypeMapMacros.cmake)

generate_typemap_check(
    "rw::core::Ptr<rw::kinematics::Frame const>"
    TYPES
    "rw::core::Ptr<rw::kinematics::Frame>"
    "rw::kinematics::Frame"
    "const rw::kinematics::Frame"
    "rw::kinematics::FixedFrame"
    "const rw::kinematics::FixedFrame"
    "rw::kinematics::MovableFrame"
    "const rw::kinematics::FixedFrame"
    "rw::core::Ptr<rw::kinematics::FixedFrame>"
    "rw::core::Ptr<rw::kinematics::FixedFrame const>"
    "rw::core::Ptr<rw::kinematics::MovableFrame>"
    "rw::core::Ptr<rw::kinematics::MovableFrame const>"
    "rw::core::Ptr<rw::models::Joint>"
    "rw::models::Joint"
    "rw::models::DependentJoint"
    "rw::models::PrismaticJoint"
    "rw::models::PrismaticSphericalJoint"
    "rw::models::PrismaticUniversalJoint"
    "rw::models::RevoluteJoint"
    "rw::models::SphericalJoint"
    "rw::models::UniversalJoint"
    "rw::models::VirtualJoint"
    "rw::core::Ptr<rw::models::DependentJoint>"
    "rw::core::Ptr<rw::models::RevoluteJoint>"
    "rw::core::Ptr<rw::models::PrismaticJoint>"
    "rw::core::Ptr<rw::models::PrismaticSphericalJoint>"
    "rw::core::Ptr<rw::models::PrismaticUniversalJoint>"
    "rw::core::Ptr<rw::models::SphericalJoint>"
    "rw::core::Ptr<rw::models::UniversalJoint>"
    "rw::core::Ptr<rw::models::VirtualJoint>"
    "rw::core::Ptr<rw::models::Joint>"
    "const rw::models::Joint"
    "const rw::models::DependentJoint"
    "const rw::models::PrismaticJoint"
    "const rw::models::PrismaticSphericalJoint"
    "const rw::models::PrismaticUniversalJoint"
    "const rw::models::RevoluteJoint"
    "const rw::models::SphericalJoint"
    "const rw::models::UniversalJoint"
    "const rw::models::VirtualJoint"
    "rw::core::Ptr<rw::models::DependentJoint const >"
    "rw::core::Ptr<rw::models::RevoluteJoint const>"
    "rw::core::Ptr<rw::models::PrismaticJoint const >"
    "rw::core::Ptr<rw::models::PrismaticSphericalJoint const >"
    "rw::core::Ptr<rw::models::PrismaticUniversalJoint const >"
    "rw::core::Ptr<rw::models::SphericalJoint const >"
    "rw::core::Ptr<rw::models::UniversalJoint const>"
    "rw::core::Ptr<rw::models::VirtualJoint const>"
    "int"
    FILE
    "./framecptr.i"
    CONVERTER
    "toFrameCPtr"
    ADD_RWPTR_FRAGMENT
)

generate_typemap_check(
    "rw::core::Ptr<rw::kinematics::Frame>"
    TYPES
    "rw::core::Ptr<rw::kinematics::Frame>"
    "rw::kinematics::Frame"
    "rw::kinematics::FixedFrame"
    "rw::kinematics::MovableFrame"
    "rw::core::Ptr<rw::kinematics::FixedFrame>"
    "rw::core::Ptr<rw::kinematics::MovableFrame>"
    "rw::core::Ptr<rw::models::Joint>"
    "rw::models::Joint"
    "rw::models::DependentJoint"
    "rw::models::PrismaticJoint"
    "rw::models::PrismaticSphericalJoint"
    "rw::models::PrismaticUniversalJoint"
    "rw::models::RevoluteJoint"
    "rw::models::SphericalJoint"
    "rw::models::UniversalJoint"
    "rw::models::VirtualJoint"
    "rw::core::Ptr<rw::models::DependentJoint>"
    "rw::core::Ptr<rw::models::RevoluteJoint>"
    "rw::core::Ptr<rw::models::PrismaticJoint>"
    "rw::core::Ptr<rw::models::PrismaticSphericalJoint>"
    "rw::core::Ptr<rw::models::PrismaticUniversalJoint>"
    "rw::core::Ptr<rw::models::SphericalJoint>"
    "rw::core::Ptr<rw::models::UniversalJoint>"
    "rw::core::Ptr<rw::models::VirtualJoint>"
    "int"
    FILE
    "./frameptr.i"
    CONVERTER
    "toFramePtr"
    ADD_RWPTR_FRAGMENT
)

generate_typemap_check(
    "rw::kinematics::Frame*"
    TYPES
    "rw::kinematics::Frame"
    "rw::core::Ptr<rw::kinematics::Frame>"
    "rw::core::Ptr<rw::kinematics::FixedFrame>"
    "rw::core::Ptr<rw::kinematics::MovableFrame>"
    "rw::core::Ptr<rw::models::Joint>"
    "rw::core::Ptr<rw::models::DependentJoint>"
    "rw::core::Ptr<rw::models::RevoluteJoint>"
    "rw::core::Ptr<rw::models::PrismaticJoint>"
    "rw::core::Ptr<rw::models::PrismaticSphericalJoint>"
    "rw::core::Ptr<rw::models::PrismaticUniversalJoint>"
    "rw::core::Ptr<rw::models::SphericalJoint>"
    "rw::core::Ptr<rw::models::UniversalJoint>"
    "rw::core::Ptr<rw::models::VirtualJoint>"
    FILE
    "./frame_pointer.i"
    CONVERTER
    "toFramePointer"
    ADD_POINTER_FRAGMENT
)

generate_typemap_check(
    "rw::core::Ptr<rw::kinematics::StateData const>"
    TYPES
    "rw::kinematics::StateData"
    "const rw::kinematics::StateData"
    "rw::kinematics::Frame"
    "const rw::kinematics::Frame"
    "rw::kinematics::FixedFrame"
    "const rw::kinematics::FixedFrame"
    "rw::kinematics::MovableFrame"
    "const rw::kinematics::FixedFrame"
    "rw::core::Ptr<rw::kinematics::StateData>"
    "rw::core::Ptr<rw::kinematics::StateData const>"
    "rw::core::Ptr<rw::kinematics::FixedFrame>"
    "rw::core::Ptr<rw::kinematics::FixedFrame const>"
    "rw::core::Ptr<rw::kinematics::MovableFrame>"
    "rw::core::Ptr<rw::kinematics::MovableFrame const>"
    "rw::core::Ptr<rw::kinematics::Frame>"
    "rw::core::Ptr<rw::kinematics::Frame const>"
    "rw::models::Joint"
    "rw::models::DependentJoint"
    "rw::models::PrismaticJoint"
    "rw::models::PrismaticSphericalJoint"
    "rw::models::PrismaticUniversalJoint"
    "rw::models::RevoluteJoint"
    "rw::models::SphericalJoint"
    "rw::models::UniversalJoint"
    "rw::models::VirtualJoint"
    "rw::core::Ptr<rw::models::DependentJoint>"
    "rw::core::Ptr<rw::models::RevoluteJoint>"
    "rw::core::Ptr<rw::models::PrismaticJoint>"
    "rw::core::Ptr<rw::models::PrismaticSphericalJoint>"
    "rw::core::Ptr<rw::models::PrismaticUniversalJoint>"
    "rw::core::Ptr<rw::models::SphericalJoint>"
    "rw::core::Ptr<rw::models::UniversalJoint>"
    "rw::core::Ptr<rw::models::VirtualJoint>"
    "rw::core::Ptr<rw::models::Joint>"
    "const rw::models::Joint"
    "const rw::models::DependentJoint"
    "const rw::models::PrismaticJoint"
    "const rw::models::PrismaticSphericalJoint"
    "const rw::models::PrismaticUniversalJoint"
    "const rw::models::RevoluteJoint"
    "const rw::models::SphericalJoint"
    "const rw::models::UniversalJoint"
    "const rw::models::VirtualJoint"
    "rw::core::Ptr<rw::models::DependentJoint const >"
    "rw::core::Ptr<rw::models::RevoluteJoint const>"
    "rw::core::Ptr<rw::models::PrismaticJoint const >"
    "rw::core::Ptr<rw::models::PrismaticSphericalJoint const >"
    "rw::core::Ptr<rw::models::PrismaticUniversalJoint const >"
    "rw::core::Ptr<rw::models::SphericalJoint const >"
    "rw::core::Ptr<rw::models::UniversalJoint const>"
    "rw::core::Ptr<rw::models::VirtualJoint const>"
    "int"
    FILE
    "./statedatacptr.i"
    CONVERTER
    "toStateDataCPtr"
    ADD_RWPTR_FRAGMENT
)

generate_typemap_check(
    "rw::core::Ptr<rw::kinematics::StateData>"
    TYPES
    "rw::kinematics::StateData"
    "rw::kinematics::Frame"
    "rw::kinematics::FixedFrame"
    "rw::kinematics::MovableFrame"
    "rw::core::Ptr<rw::kinematics::StateData>"
    "rw::core::Ptr<rw::kinematics::Frame>"
    "rw::core::Ptr<rw::kinematics::FixedFrame>"
    "rw::core::Ptr<rw::kinematics::MovableFrame>"
    "rw::models::Joint"
    "rw::models::DependentJoint"
    "rw::models::PrismaticJoint"
    "rw::models::PrismaticSphericalJoint"
    "rw::models::PrismaticUniversalJoint"
    "rw::models::RevoluteJoint"
    "rw::models::SphericalJoint"
    "rw::models::UniversalJoint"
    "rw::models::VirtualJoint"
    "rw::core::Ptr<rw::models::DependentJoint>"
    "rw::core::Ptr<rw::models::RevoluteJoint>"
    "rw::core::Ptr<rw::models::PrismaticJoint>"
    "rw::core::Ptr<rw::models::PrismaticSphericalJoint>"
    "rw::core::Ptr<rw::models::PrismaticUniversalJoint>"
    "rw::core::Ptr<rw::models::SphericalJoint>"
    "rw::core::Ptr<rw::models::UniversalJoint>"
    "rw::core::Ptr<rw::models::VirtualJoint>"
    "rw::core::Ptr<rw::models::Joint>"
    "int"
    FILE
    "./statedataptr.i"
    CONVERTER
    "toStateDataPtr"
    ADD_RWPTR_FRAGMENT
)

generate_typemap_check(
    "rw::core::Ptr<rw::models::Device>"
    TYPES
    "rw::models::Device"
    "rw::models::JointDevice"
    "rw::models::MobileDevice"
    "rw::models::SE3Device"
    "rw::models::CompositeDevice"
    "rw::models::CompositeJointDevice"
    "rw::models::ParallelDevice"
    "rw::models::SerialDevice"
    "rw::models::TreeDevice"
    "rw::core::Ptr<rw::models::Device>"
    "rw::core::Ptr<rw::models::JointDevice>"
    "rw::core::Ptr<rw::models::MobileDevice>"
    "rw::core::Ptr<rw::models::SE3Device>"
    "rw::core::Ptr<rw::models::CompositeDevice>"
    "rw::core::Ptr<rw::models::CompositeJointDevice>"
    "rw::core::Ptr<rw::models::ParallelDevice>"
    "rw::core::Ptr<rw::models::SerialDevice>"
    "rw::core::Ptr<rw::models::TreeDevice>"
    FILE
    "./devicePtr.i"
    CONVERTER
    "toDevicePtr"
    ADD_RWPTR_FRAGMENT
)
generate_typemap_check(
    "rw::core::Ptr<rw::models::Device const>"
    TYPES
    "rw::models::Device"
    "rw::models::JointDevice"
    "rw::models::MobileDevice"
    "rw::models::SE3Device"
    "rw::models::CompositeDevice"
    "rw::models::CompositeJointDevice"
    "rw::models::ParallelDevice"
    "rw::models::SerialDevice"
    "rw::models::TreeDevice"
    "rw::core::Ptr<rw::models::Device const>"
    "rw::core::Ptr<rw::models::JointDevice const>"
    "rw::core::Ptr<rw::models::MobileDevice const>"
    "rw::core::Ptr<rw::models::SE3Device const>"
    "rw::core::Ptr<rw::models::CompositeDevice const>"
    "rw::core::Ptr<rw::models::CompositeJointDevice const>"
    "rw::core::Ptr<rw::models::ParallelDevice const>"
    "rw::core::Ptr<rw::models::SerialDevice const>"
    "rw::core::Ptr<rw::models::TreeDevice const>"
    "rw::core::Ptr<rw::models::Device>"
    "rw::core::Ptr<rw::models::JointDevice>"
    "rw::core::Ptr<rw::models::MobileDevice>"
    "rw::core::Ptr<rw::models::SE3Device>"
    "rw::core::Ptr<rw::models::CompositeDevice>"
    "rw::core::Ptr<rw::models::CompositeJointDevice>"
    "rw::core::Ptr<rw::models::ParallelDevice>"
    "rw::core::Ptr<rw::models::SerialDevice>"
    "rw::core::Ptr<rw::models::TreeDevice>"
    FILE
    "./constDevicePtr.i"
    CONVERTER
    "toConstDevicePtr"
    ADD_RWPTR_FRAGMENT
)

generate_typemap_check(
    "rw::models::ParallelDevice*"
    TYPES
    "rw::models::ParallelDevice"
    "rw::core::Ptr<rw::models::ParallelDevice>"
    FILE
    "./ParallelDevicePointer.i"
    CONVERTER
    "toParallelDevicePointer"
    ADD_POINTER_FRAGMENT
)

generate_typemap_check(
    "rw::core::Ptr<rw::models::ParallelDevice const>"
    TYPES
    "rw::models::ParallelDevice"
    "rw::core::Ptr<rw::models::ParallelDevice const>"
    "rw::core::Ptr<rw::models::ParallelDevice>"
    FILE
    "./constParallelDevicePtr.i"
    CONVERTER
    "toConstParallelDevicePtr"
    ADD_RWPTR_FRAGMENT
)


generate_typemap_check(
    "rw::core::Ptr<rw::models::ParallelDevice>"
    TYPES
    "rw::models::ParallelDevice"
    "rw::core::Ptr<rw::models::ParallelDevice>"
    FILE
    "./ParallelDevicePtr.i"
    CONVERTER
    "toParallelDevicePtr"
    ADD_RWPTR_FRAGMENT
)



generate_typemap_check(
    "rw::core::Ptr<rw::models::Joint>"
    TYPES
    "rw::core::Ptr<rw::models::Joint>"
    "rw::models::Joint"
    "rw::models::DependentJoint"
    "rw::models::PrismaticJoint"
    "rw::models::PrismaticSphericalJoint"
    "rw::models::PrismaticUniversalJoint"
    "rw::models::RevoluteJoint"
    "rw::models::SphericalJoint"
    "rw::models::UniversalJoint"
    "rw::models::VirtualJoint"
    "rw::core::Ptr<rw::models::DependentJoint>"
    "rw::core::Ptr<rw::models::RevoluteJoint>"
    "rw::core::Ptr<rw::models::PrismaticJoint>"
    "rw::core::Ptr<rw::models::PrismaticSphericalJoint>"
    "rw::core::Ptr<rw::models::PrismaticUniversalJoint>"
    "rw::core::Ptr<rw::models::SphericalJoint>"
    "rw::core::Ptr<rw::models::UniversalJoint>"
    "rw::core::Ptr<rw::models::VirtualJoint>"
    "int"
    FILE
    "./jointptr.i"
    CONVERTER
    "toJointPtr"
    ADD_RWPTR_FRAGMENT
)

generate_typemap_check(
    "rw::models::Joint*"
    TYPES
    "rw::models::Joint"
    "rw::core::Ptr<rw::models::Joint>"
    "rw::core::Ptr<rw::models::DependentJoint>"
    "rw::core::Ptr<rw::models::RevoluteJoint>"
    "rw::core::Ptr<rw::models::PrismaticJoint>"
    "rw::core::Ptr<rw::models::PrismaticSphericalJoint>"
    "rw::core::Ptr<rw::models::PrismaticUniversalJoint>"
    "rw::core::Ptr<rw::models::SphericalJoint>"
    "rw::core::Ptr<rw::models::UniversalJoint>"
    "rw::core::Ptr<rw::models::VirtualJoint>"
    FILE
    "./joint_pointer.i"
    CONVERTER
    "toJointPointer"
    ADD_POINTER_FRAGMENT
)

generate_typemap_check(
    "rw::core::Ptr<rw::trajectory::Trajectory<rw::math::Q>>"
    TYPES
    "rw::core::Ptr<rw::trajectory::Trajectory<rw::math::Q>>"
    "rw::core::Ptr<rw::trajectory::InterpolatorTrajectory<rw::math::Q>>"
    FILE
    "./TrajectoryPtr.i"
    CONVERTER
    "toTrajectoryPtr"
    ADD_RWPTR_FRAGMENT
)

generate_typemap_check(
    "rw::core::Ptr< rw::trajectory::Blend< rw::math::Vector2D<double>>>"
    TYPES
    "rw::core::Ptr< rw::trajectory::Blend< rw::math::Vector2D<double>>>"
    "rw::core::Ptr< rw::trajectory::ParabolicBlend< rw::math::Vector2D<double>>>"
    "rw::trajectory::ParabolicBlend< rw::math::Vector2D<double>>"
    "rw::trajectory::Blend< rw::math::Vector2D<double>>"
    "int"
    FILE
    "./blendPtr.i"
    CONVERTER
    "toBlendPtr"
    ADD_RWPTR_FRAGMENT
)

generate_typemap_check(
    "rw::core::Ptr< rw::trajectory::Interpolator< rw::math::Vector2D<double>>>"
    TYPES
    "rw::core::Ptr< rw::trajectory::Interpolator< rw::math::Vector2D<double>>>"
    "rw::core::Ptr< rw::trajectory::LinearInterpolator< rw::math::Vector2D<double>>>"
    "int"
    FILE
    "./interpolatorPtr.i"
    CONVERTER
    "toInterpolatorPtr"
    ADD_RWPTR_FRAGMENT
)

generate_typemap_check(
    "rw::core::Ptr<rw::pathplanning::QConstraint const>"
    TYPES 
    "const rw::pathplanning::QConstraint"
    "rw::pathplanning::QConstraint"
    "rw::core::Ptr<rw::pathplanning::QConstraint const>"
    "rw::core::Ptr<rw::pathplanning::QConstraint>"
    FILE 
    "./ConstQConstraintPtr.i"
    CONVERTER 
    "toConstQConstraintPtr"
    ADD_RWPTR_FRAGMENT
)



generate_typemap_check(
    "rw::core::Ptr<rwlibs::pathoptimization::ClearanceCalculator const>"
    TYPES 
    "rwlibs::pathoptimization::ClearanceCalculator"
    "const rwlibs::pathoptimization::ClearanceCalculator"
    "rwlibs::pathoptimization::MinimumClearanceCalculator"
    "const rwlibs::pathoptimization::MinimumClearanceCalculator"
    "rw::core::Ptr<rwlibs::pathoptimization::ClearanceCalculator>"
    "rw::core::Ptr<rwlibs::pathoptimization::ClearanceCalculator const>"
    "rw::core::Ptr<rwlibs::pathoptimization::MinimumClearanceCalculator>"
    "rw::core::Ptr<rwlibs::pathoptimization::MinimumClearanceCalculator const>"
    FILE 
    "./ConstClearanceCalculatorPtr.i"
    CONVERTER 
    "toConstClearanceCalculatorPtr"
    ADD_RWPTR_FRAGMENT
)

generate_typemap_check(
    "rw::core::Ptr<rwlibs::pathoptimization::ClearanceCalculator>"
    TYPES 
    "rwlibs::pathoptimization::ClearanceCalculator"
    "rwlibs::pathoptimization::MinimumClearanceCalculator"
    "rw::core::Ptr<rwlibs::pathoptimization::ClearanceCalculator>"
    "rw::core::Ptr<rwlibs::pathoptimization::MinimumClearanceCalculator>"
    FILE 
    "./ClearanceCalculatorPtr.i"
    CONVERTER 
    "toClearanceCalculatorPtr"
    ADD_RWPTR_FRAGMENT
)