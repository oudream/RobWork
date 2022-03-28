%module sdurw_models

%include <rwlibs/swig/swig_macros.i>
%include <rwlibs/swig/ext_i/rw_vector.i>
%include <exception.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_geometry.i>
%import <rwlibs/swig/sdurw_sensor.i>

%import <rwlibs/swig/ext_i/std.i>

%{
	#include <rw/kinematics/Frame.hpp>
	#include <rw/kinematics/FixedFrame.hpp>
	#include <rw/sensor/CameraModel.hpp>
	#include <rw/sensor/TactileArrayModel.hpp>
	#include <rw/sensor/FTSensorModel.hpp>
	#include <rw/sensor/Scanner25DModel.hpp>
	#include <rw/sensor/Scanner2DModel.hpp>
	#include <rw/sensor/RGBDCameraModel.hpp>
	#include <rw/sensor/StereoCameraModel.hpp>
	#include <rw/models.hpp>
%}

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
%}

%exception {
    try {
        //printf("Entering function : $name\n"); // uncomment to get a print out of all function calls
        $action
    }catch(rw::core::Exception& e ){
        SWIG_exception(SWIG_RuntimeError,e.what());
    }catch(...){
        SWIG_exception(SWIG_RuntimeError,"unknown error");
    }
}

%{
	template<class T>
	char* rw_kinematics_Frame___str__(T* f){
		return printCString<rw::kinematics::Frame>(*f);
	}
	template<class T>
	char* rw_kinematics_Frame_toString(T* f){
		return printCString<rw::kinematics::Frame>(*f);
	}
%}

%{
	#include <rw/proximity/CollisionSetup.hpp>
	#include <rw/proximity/CollisionSetup.cpp>
	#if SWIG_VERSION < 0x040000
		#include <rw/geometry/IndexedTriangle.hpp>
		template<class T >
		using IndexedTriangle = rw::geometry::IndexedTriangle<T>;
	#endif 
%}

#if defined(SWIGJAVA)
	%ignore rw::models::Object::getBase() const;
#endif 
%{
	#include<rw/models/Object.hpp>
%}
%include <rw/models/Object.hpp>
NAMED_OWNEDPTR(Object,rw::models::Object);
%std_vector (ObjectPtrVector, rw::core::Ptr< rw::models::Object > );

#if defined(SWIGJAVA)
	%ignore rw::models::Device::getBase() const;
	%ignore rw::models::Device::getEnd() const;
	%ignore rw::models::Device::getPropertyMap() const;
#endif 
%include <rwlibs/swig/typemaps/constDevicePtr.i>
%include <rwlibs/swig/typemaps/devicePtr.i>
%{
	#include<rw/models/Device.hpp>
%}
%include <rw/models/Device.hpp>
NAMED_OWNEDPTR(Device,rw::models::Device);
%std_vector_f (DevicePtrVector, rw::core::Ptr<rw::models::Device>,std::vector, "toDevicePtrPy");

%{
	std::vector< double,std::allocator< double > > rw_kinematics_StateData_getData(rw::kinematics::StateData *self,rw::kinematics::State &state){
        double* data = self->getData(state);
        std::vector<double> ret(self->size());
        for(int i = 0; i < self->size(); i++){
            ret[i] = data[i];
        }
        return ret;
    }
%}
%include <rwlibs/swig/typemaps/jointptr.i>
%include <rwlibs/swig/typemaps/joint_pointer.i>

%{
	#include<rw/models/Joint.hpp>
%}
%include <rw/models/Joint.hpp>
NAMED_OWNEDPTR(Joint,rw::models::Joint);
%std_vector_f(VectorJoint_p,rw::models::Joint*, std::vector, "toJointPointerPy");

%{
	#include<rw/models/JointDevice.hpp>
%}
%include <rw/models/JointDevice.hpp>
NAMED_OWNEDPTR(JointDevice,rw::models::JointDevice);
%std_vector (JointDevicePtrVector,rw::core::Ptr< rw::models::JointDevice>);


%nodefaultctor JacobianCalculator;
%{
	#include<rw/models/JacobianCalculator.hpp>
%}
%include <rw/models/JacobianCalculator.hpp>
NAMED_OWNEDPTR(JacobianCalculator,rw::models::JacobianCalculator);

%extend rw::models::JacobianCalculator{
	/**
	 * @brief Returns the Jacobian associated to \b state
	 *
	 * @param state [in] State for which to calculate the Jacobian
	 * @return Jacobian for \b state
	 */
	virtual rw::math::Jacobian getJacobian(const rw::kinematics::State& state) const {
		return $self->get(state);
	}
};

%{
	#include<rw/models/CompositeDevice.hpp>
%}
%include <rw/models/CompositeDevice.hpp>
NAMED_OWNEDPTR(CompositeDevice,rw::models::CompositeDevice);
%extend rw::core::Ptr<rw::models::CompositeDevice>{
	rw::core::Ptr<rw::models::Device> asDevicePtr() { return *$self; }
	rw::core::Ptr<rw::models::Device const> asDeviceCPtr() { return *$self; }
	rw::core::Ptr<rw::models::JointDevice> asJointDevicePtr() { return *$self; }
	rw::core::Ptr<rw::models::JointDevice const> asJointDeviceCPtr() { return *$self; }
}

%{
	#include<rw/models/CompositeJointDevice.hpp>
%}
%include <rw/models/CompositeJointDevice.hpp>
NAMED_OWNEDPTR(CompositeJointDevice,rw::models::CompositeJointDevice);

%{
	#include<rw/models/ControllerModel.hpp>
%}
%include <rw/models/ControllerModel.hpp>
NAMED_OWNEDPTR(ControllerModel,rw::models::ControllerModel);
%std_vector(ControllerModelPtrVector,rw::core::Ptr<rw::models::ControllerModel>);

%ignore rw::models::DeformableObject::getNode(int,rw::kinematics::State const &) const;
%{
	#include<rw/models/DeformableObject.hpp>
%}
%include <rw/models/DeformableObject.hpp>
NAMED_OWNEDPTR(DeformableObject,rw::models::DeformableObject);
%std_vector (DeformableObjectPtrVector,rw::core::Ptr<rw::models::DeformableObject>);

%{
	#include<rw/models/DependentJoint.hpp>
%}
%include <rw/models/DependentJoint.hpp>
NAMED_OWNEDPTR(DependentJoint,rw::models::DependentJoint);

%{
	#include<rw/models/DependentPrismaticJoint.hpp>
%}
%include <rw/models/DependentPrismaticJoint.hpp>
NAMED_OWNEDPTR(DependentPrismaticJoint,rw::models::DependentPrismaticJoint);

%{
	#include<rw/models/DependentRevoluteJoint.hpp>
%}
%include <rw/models/DependentRevoluteJoint.hpp>
NAMED_OWNEDPTR(DependentRevoluteJoint,rw::models::DependentRevoluteJoint);

%{
	#include<rw/models/DeviceJacobianCalculator.hpp>
%}
%include <rw/models/DeviceJacobianCalculator.hpp>
NAMED_OWNEDPTR(DeviceJacobianCalculator,rw::models::DeviceJacobianCalculator);

%nodefaultctor DHParameterSet;
%{
	#include<rw/models/DHParameterSet.hpp>
%}
%include <rw/models/DHParameterSet.hpp>
NAMED_OWNEDPTR(DHParameterSet,rw::models::DHParameterSet);
%std_vector (DHParameterSetVector,rw::models::DHParameterSet);

%{
	#include<rw/models/JacobianUtil.hpp>
%}
%include <rw/models/JacobianUtil.hpp>
NAMED_OWNEDPTR(JacobianUtil,rw::models::JacobianUtil);


%{
	#include<rw/models/JointDeviceJacobianCalculator.hpp>
%}
%include <rw/models/JointDeviceJacobianCalculator.hpp>
NAMED_OWNEDPTR(JointDeviceJacobianCalculator,rw::models::JointDeviceJacobianCalculator);

%{
	#include<rw/models/MobileDevice.hpp>
%}
%include <rw/models/MobileDevice.hpp>
NAMED_OWNEDPTR(MobileDevice,rw::models::MobileDevice);

%{
	#include<rw/models/Models.hpp>
%}
%include <rw/models/Models.hpp>
NAMED_OWNEDPTR(Models,rw::models::Models);

%{
	#include<rw/models/ParallelLeg.hpp>
%}
%include <rw/models/ParallelLeg.hpp>
NAMED_OWNEDPTR(ParallelLeg,rw::models::ParallelLeg);

%std_vector(VectorParallelLegPtr, rw::core::Ptr<rw::models::ParallelLeg>);
%std_vector(VectorParallelLeg_p, rw::models::ParallelLeg*);
%std_vector_explicit(VectorParallelDeviceLeg, rw::models::ParallelLeg*, std::vector<std::vector<rw::models::ParallelLeg*>>,"generalToFromPy");

%{
	#include<rw/models/ParallelDevice.hpp>
%}
%include <rw/models/ParallelDevice.hpp>
NAMED_OWNEDPTR(ParallelDevice,rw::models::ParallelDevice);
%std_vector (VectorParallelDevicePtr,rw::core::Ptr<rw::models::ParallelDevice>);

%{
	#include<rw/models/PrismaticJoint.hpp>
%}
%include <rw/models/PrismaticJoint.hpp>
NAMED_OWNEDPTR(PrismaticJoint,rw::models::PrismaticJoint);

%{
	#include<rw/models/PrismaticSphericalJoint.hpp>
%}
%include <rw/models/PrismaticSphericalJoint.hpp>
NAMED_OWNEDPTR(PrismaticSphericalJoint,rw::models::PrismaticSphericalJoint);

%{
	#include<rw/models/PrismaticUniversalJoint.hpp>
%}
%include <rw/models/PrismaticUniversalJoint.hpp>
NAMED_OWNEDPTR(PrismaticUniversalJoint,rw::models::PrismaticUniversalJoint);

%{
	#include<rw/models/RevoluteJoint.hpp>
%}
%include <rw/models/RevoluteJoint.hpp>
NAMED_OWNEDPTR(RevoluteJoint,rw::models::RevoluteJoint);

#if defined(SWIGPYTHON)
%pythoncode {
RevoluteJointClass = RevoluteJoint
class RevoluteJoint(RevoluteJointClass):
    def __new__(clc,name,transform):
        return ownedPtr(RevoluteJointClass(name,transform))
}
#endif

%{
	#include<rw/models/RigidBodyInfo.hpp>
%}
%include <rw/models/RigidBodyInfo.hpp>
NAMED_OWNEDPTR(RigidBodyInfo,rw::models::RigidBodyInfo);

%{
	#include<rw/models/RigidObject.hpp>
%}
%include <rw/models/RigidObject.hpp>
NAMED_OWNEDPTR(RigidObject,rw::models::RigidObject);
%std_vector (VectorRigidObjectPtr,rw::core::Ptr < rw::models::RigidObject > );

%{
	#include<rw/models/SE3Device.hpp>
%}
%include <rw/models/SE3Device.hpp>
NAMED_OWNEDPTR(SE3Device,rw::models::SE3Device);

%{
	#include<rw/models/SerialDevice.hpp>
%}
%include <rw/models/SerialDevice.hpp>
NAMED_OWNEDPTR(SerialDevice,rw::models::SerialDevice);
%std_vector (VectorSerialDevicePtr,rw::core::Ptr<rw::models::SerialDevice>);

%{
	#include<rw/models/SphericalJoint.hpp>
%}
%include <rw/models/SphericalJoint.hpp>
NAMED_OWNEDPTR(SphericalJoint,rw::models::SphericalJoint);

%{
	#include<rw/models/TreeDevice.hpp>
%}
%include <rw/models/TreeDevice.hpp>
NAMED_OWNEDPTR(TreeDevice,rw::models::TreeDevice);
%std_vector(VectorTreeDevicePtr,rw::core::Ptr<rw::models::TreeDevice>);

%{
	#include<rw/models/UniversalJoint.hpp>
%}
%include <rw/models/UniversalJoint.hpp>
NAMED_OWNEDPTR(UniversalJoint,rw::models::UniversalJoint);

%{
	#include<rw/models/VirtualJoint.hpp>
%}
%include <rw/models/VirtualJoint.hpp>
NAMED_OWNEDPTR(VirtualJoint,rw::models::VirtualJoint);

%{
	#include<rw/models/WorkCell.hpp>
%}
%include <rw/models/WorkCell.hpp>
NAMED_OWNEDPTR(WorkCell,rw::models::WorkCell);
//%template (WorkCellChangedEvent) rw::core::Event< rw::models::WorkCell::WorkCellChangedListener, int >;


%extend rw::models::WorkCell {
	/**
	 * @brief Returns MovableFrame with the specified name.
	 *
	 * If multiple frames has the same name, the first frame encountered
	 * will be returned. If no frame is found, the method returns NULL.
	 *
	 * @param name [in] name of Frame.
	 *
	 * @return The MovableFrame with name \b name or NULL if no such frame.
	 */
	rw::kinematics::MovableFrame* findMovableFrame(const std::string& name)
	{ 
		return $self->WorkCell::findFrame<rw::kinematics::MovableFrame>(name); 
	}

	/**
	 * @brief Returns FixedFrame with the specified name.
	 *
	 * If multiple frames has the same name, the first frame encountered
	 * will be returned. If no frame is found, the method returns NULL.
	 *
	 * @param name [in] name of Frame.
	 *
	 * @return The FixedFrame with name \b name or NULL if no such frame.
	 */
	rw::kinematics::FixedFrame* findFixedFrame(const std::string& name)
	{ 
		return $self->WorkCell::findFrame<rw::kinematics::FixedFrame>(name); 
	}

	/**
	 * @brief Returns all \b MovableFrames.
	 * @return all frames of type \b MovableFrames in the workcell
	 */
	std::vector<rw::kinematics::MovableFrame*> findMovableFrames() const
	{ 
		return $self->WorkCell::findFrames<rw::kinematics::MovableFrame>(); 
	}

	/**
	 * @brief Returns all \b FixedFrame.
	 * @return all frames of type \b FixedFrame in the workcell
	 */
	std::vector<rw::kinematics::FixedFrame*> findFixedFrames() const
	{ 
		return $self->WorkCell::findFrames<rw::kinematics::FixedFrame>(); 
	}

	/**
	 * @brief The device named \b name of the workcell.
	 *
	 * NULL is returned if there is no such device.
	 *
	 * @param name [in] The device name
	 *
	 * @return The device named \b name or NULL if no such device.
	 */
	rw::core::Ptr<rw::models::JointDevice> findJointDevice(const std::string& name)
	{ 
		return $self->WorkCell::findDevice<rw::models::JointDevice>(name); 
	}

	/**
	 * @brief The device named \b name of the workcell.
	 *
	 * NULL is returned if there is no such device.
	 *
	 * @param name [in] The device name
	 *
	 * @return The device named \b name or NULL if no such device.
	 */
	rw::core::Ptr<rw::models::SerialDevice> findSerialDevice(const std::string& name)
	{ 
		return $self->WorkCell::findDevice<rw::models::SerialDevice>(name); 
	}

	/**
	 * @brief The device named \b name of the workcell.
	 *
	 * NULL is returned if there is no such device.
	 *
	 * @param name [in] The device name
	 *
	 * @return The device named \b name or NULL if no such device.
	 */
	rw::core::Ptr<rw::models::TreeDevice> findTreeDevice(const std::string& name)
	{ 
		return $self->WorkCell::findDevice<rw::models::TreeDevice>(name); 
	}

	/**
	 * @brief The device named \b name of the workcell.
	 *
	 * NULL is returned if there is no such device.
	 *
	 * @param name [in] The device name
	 *
	 * @return The device named \b name or NULL if no such device.
	 */
	rw::core::Ptr<rw::models::ParallelDevice> findParallelDevice(const std::string& name)
	{ 
		return $self->WorkCell::findDevice<rw::models::ParallelDevice>(name); 
	}

	/**
	 * @brief Returns a vector with pointers to the Device(s) with a
	 * specific type \b JointDevice in the WorkCell
	 *
	 * @return vector with pointers to Device(s) of type T.
	 */
	std::vector < rw::core::Ptr<rw::models::JointDevice> > findJointDevices()
	{ 
		return $self->WorkCell::findDevices<rw::models::JointDevice>(); 
	}

	/**
	 * @brief Returns a vector with pointers to the Device(s) with a
	 * specific type \b SerialDevice in the WorkCell
	 *
	 * @return vector with pointers to Device(s) of type T.
	 */
	std::vector < rw::core::Ptr<rw::models::SerialDevice> > findSerialDevices()
	{ 
		return $self->WorkCell::findDevices<rw::models::SerialDevice>(); 
	}

	/**
	 * @brief Returns a vector with pointers to the Device(s) with a
	 * specific type \b TreeDevice in the WorkCell
	 *
	 * @return vector with pointers to Device(s) of type T.
	 */
	std::vector < rw::core::Ptr<rw::models::TreeDevice> > findTreeDevices()
	{ 
		return $self->WorkCell::findDevices<rw::models::TreeDevice>(); 
	}

	/**
	 * @brief Returns a vector with pointers to the Device(s) with a
	 * specific type \b ParallelDevice in the WorkCell
	 *
	 * @return vector with pointers to Device(s) of type T.
	 */
	std::vector < rw::core::Ptr<rw::models::ParallelDevice> > findParallelDevices()
	{ 
		return $self->WorkCell::findDevices<rw::models::ParallelDevice>(); 
	}

};
