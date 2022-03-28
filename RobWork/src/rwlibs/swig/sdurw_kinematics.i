%module sdurw_kinematics

%include <exception.i>
%include <rwlibs/swig/swig_macros.i>
%include <rwlibs/swig/ext_i/rw_vector.i>
%include <std_pair.i>


%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>

%import <rwlibs/swig/ext_i/std.i>

%{
    #include <rw/common/BINArchive.hpp>
    #include <rw/common/INIArchive.hpp>
    #include <rw/kinematics.hpp>
    #include <rw/models.hpp>

    #include <rw/models/Joint.hpp>
    #include <rw/models/DependentJoint.hpp>
    #include <rw/models/PrismaticJoint.hpp>
    #include <rw/models/PrismaticSphericalJoint.hpp>
    #include <rw/models/PrismaticUniversalJoint.hpp>
    #include <rw/models/SphericalJoint.hpp>
    #include <rw/models/RevoluteJoint.hpp>
    #include <rw/models/UniversalJoint.hpp>
    #include <rw/models/VirtualJoint.hpp>
    
%}


%pragma(java) jniclassimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
%}
%pragma(java) moduleimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
%}
%typemap(javaimports) SWIGTYPE %{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
%}

/*************************************
 *  CODE for Handeling Ptr<Ptr<Type>>
 *************************************/
%{
namespace rw{ namespace core{
    template <class T, class R>
    T ownedPtr(R frame){
        return frame;
    }
}}
%}
namespace rw{ namespace core {
    template <class T, class R>
    T ownedPtr(R frame);
}}

/*************************************
 *  START of SWIG
 *************************************/
%include <rwlibs/swig/typemaps/statedatacptr.i>
%include <rwlibs/swig/typemaps/statedataptr.i>

%warnfilter(508) rw::core::Ptr<rw::kinematics::StateData>::operator==;
%warnfilter(508) rw::kinematics::StateData::operator==;
%ignore rw::kinematics::StateData::getCache() const;
%ignore rw::kinematics::StateData::getCache(rw::kinematics::State &);
%{
    #include <rw/kinematics/StateData.hpp>
%}
%include <rw/kinematics/StateData.hpp>
%extend rw::kinematics::StateData{
    /**
     * @brief An array of length size() containing the values for
     * the state data.
     *
     * It is OK to call this method also for a StateData with zero size.
     *
     * @param state [in] The state containing the StateData values.
     *
     * @return The values for the frame.
     */
    std::vector<double> getData(rw::kinematics::State& state){
        double* data = $self->getData(state);
        std::vector<double> ret($self->size());
        for(int i = 0; i < $self->size(); i++){
            ret[i] = data[i];
        }
        return ret;
    }
}
%std_vector(VectorStateDataPtr,rw::core::Ptr<rw::kinematics::StateData>);
NAMED_OWNEDPTR(StateData, rw::kinematics::StateData);


%ignore rw::kinematics::Frame::getPropertyMap() const;
%ignore rw::kinematics::Frame::getDafParent(rw::kinematics::State const &) const;
%ignore rw::kinematics::Frame::getDafChildren(rw::kinematics::State const &) const;
%ignore rw::kinematics::Frame::getParent(rw::kinematics::State const &) const;
%ignore rw::kinematics::Frame::getChildren(rw::kinematics::State const &) const;
%ignore rw::kinematics::Frame::getChildren() const;
%ignore rw::kinematics::Frame::getParent() const;


%include <rwlibs/swig/typemaps/framecptr.i>
%include <rwlibs/swig/typemaps/frameptr.i>
%include <rwlibs/swig/typemaps/frame_pointer.i>

#if defined(SWIGPYTHON)
%warnfilter(508) rw::core::Ptr<rw::kinematics::Frame>::operator==;
%warnfilter(508) rw::kinematics::Frame::operator==;
#pragma SWIG nowarn=508
#endif

%{
    #include <rw/kinematics/Frame.hpp>
%}
%include <rw/kinematics/Frame.hpp>
NAMED_OWNEDPTR(Frame, rw::kinematics::Frame);

%std_vector_f (FrameVector, rw::kinematics::Frame*,std::vector,"toFramePointerPy");
%std_vector_explicit (FramePairVector, rw::kinematics::Frame *,SWIG_CORE_DEFINE(std::vector<std::pair< rw::kinematics::Frame *, rw::kinematics::Frame * >> ),"generalToFromPy");
%std_vector_explicit (VectorVectorFrame,rw::kinematics::Frame*,std::vector<std::vector<rw::kinematics::Frame*>>,"generalToFromPy");
%std_vector_f (VectorFramePtr, rw::core::Ptr<rw::kinematics::Frame>,std::vector,"toFramePtrPy");

%template (ownedPtr) rw::core::ownedPtr<rw::core::Ptr<rw::kinematics::Frame>,rw::core::Ptr<rw::kinematics::Frame>>;
%template (MapStringFrame) std::map<std::string,rw::kinematics::Frame*>;
%template (FramePair) std::pair< rw::kinematics::Frame *, rw::kinematics::Frame * >;

%{
    #include <rw/kinematics/FixedFrame.hpp>
%}
%include <rw/kinematics/FixedFrame.hpp>
%std_vector(VectorFixedFrame, rw::kinematics::FixedFrame*);
%template (ownedPtr) rw::core::ownedPtr<rw::core::Ptr<rw::kinematics::FixedFrame>,rw::core::Ptr<rw::kinematics::FixedFrame>>;
NAMED_OWNEDPTR(FixedFrame, rw::kinematics::FixedFrame);

#if defined(SWIGPYTHON)
%pythoncode {
FixedFrameClass = FixedFrame
class FixedFrame(FixedFrameClass):
    def __new__(clc,name,transform):
        return ownedPtr(FixedFrameClass(name,transform))
}
#endif

%extend rw::core::Ptr<rw::kinematics::FixedFrame> {
    bool operator == (rw::kinematics::Frame* rhs){
        return $self->get() == rhs;
    }
}


%{
    #include <rw/kinematics/MovableFrame.hpp>
%}
%include <rw/kinematics/MovableFrame.hpp>
%std_vector (MovableFrameVector, rw::kinematics::MovableFrame *) ;
%template (ownedPtr) rw::core::ownedPtr<rw::core::Ptr<rw::kinematics::MovableFrame>,rw::core::Ptr<rw::kinematics::MovableFrame>>;
NAMED_OWNEDPTR(MovableFrame, rw::kinematics::MovableFrame);

#if defined(SWIGPYTHON)
%pythoncode {
movableFrameClass = MovableFrame
class MovableFrame(movableFrameClass):
    def __new__(clc,name):
        return ownedPtr(movableFrameClass(name))
}
#endif

%extend rw::core::Ptr<rw::kinematics::MovableFrame> {
    bool operator == (rw::kinematics::Frame* rhs){
        return $self->get() == rhs;
    }
}

%{
    #include <rw/kinematics/FKRange.hpp>
%}
%include <rw/kinematics/FKRange.hpp>
NAMED_OWNEDPTR(FKRange,rw::kinematics::FKRange);

%ignore rw::kinematics::FKTable::get(const rw::kinematics::Frame&) const;
%{
    #include <rw/kinematics/FKTable.hpp>
%}
%include <rw/kinematics/FKTable.hpp>
NAMED_OWNEDPTR(FKTable,rw::kinematics::FKTable);

%{
    #include <rw/kinematics/FrameMap.hpp>
%}
%include <rw/kinematics/FrameMap.hpp>
//NAMED_OWNEDPTR(FrameMap,rw::kinematics::FrameMap);

%{
    #include <rw/kinematics/FramePairMap.hpp>
%}
%include <rw/kinematics/FramePairMap.hpp>
//NAMED_OWNEDPTR(FramePairMap,rw::kinematics::FramePairMap);

%{
    #include <rw/kinematics/FrameType.hpp>
%}
%include <rw/kinematics/FrameType.hpp>
NAMED_OWNEDPTR(FrameType,rw::kinematics::FrameType);

%nodefaultctor Kinematics;
%{
    #include <rw/kinematics/Kinematics.hpp>
%}
%include <rw/kinematics/Kinematics.hpp>
NAMED_OWNEDPTR(Kinematics,rw::kinematics::Kinematics);


%ignore rw::kinematics::QState::getQ;
%ignore rw::kinematics::QState::setQ;
%{
    #include <rw/kinematics/QState.hpp>
%}
%include <rw/kinematics/QState.hpp>
NAMED_OWNEDPTR(QState,rw::kinematics::QState);
%extend rw::kinematics::QState{
    /**
     * @brief An array of length frame.getDOF() containing the joint values
     * for \b frame.
     *
     * It is OK to call this method also for frames with zero degrees of
     * freedom.
     *
     * @return The joint values for the frame.
     */
    std::vector<double> getQ(const rw::kinematics::StateData& SData){
        double* data = $self->getQ(SData);
        std::vector<double> ret($self->size());
        for(int i = 0; i < $self->size(); i++){
            ret.push_back(data[i]);
        }
        return ret;
    }
    
    /**
     * @brief Assign for \b frame the frame.getDOF() joint values of the
     * array \b vals.
     *
     * The array \b vals must be of length at least frame.getDOF().
     *
     * @param data [in] The StateData for which the joint values are assigned.
     *
     * @param vals [in] The joint values to assign.
     *
     * setQ() and getQ() are related as follows:
     * \code
     * q_state.setQ(frame, q_in);
     * const double* q_out = q_state.getQ(frame);
     * for (int i = 0; i < frame.getDOF(); i++)
     *   q_in[i] == q_out[i];
     * \endcode
     */
    void setQ(const rw::kinematics::StateData& data, const std::vector<double> vals){
        $self->setQ(data,vals.data());
    }
}

%{
    #include <rw/kinematics/State.hpp>
%}
%include <rw/kinematics/State.hpp>
%std_vector (VectorState,rw::kinematics::State);

%{
    #include <rw/kinematics/StateCache.hpp>
%}
%include <rw/kinematics/StateCache.hpp>
NAMED_OWNEDPTR(StateCache, rw::kinematics::StateCache);

%ignore rw::kinematics::Stateless::getStateStructure() const;
%{
    #include <rw/kinematics/Stateless.hpp>
%}
%include <rw/kinematics/Stateless.hpp>
//NAMED_OWNEDPTR(Stateless,rw::kinematics::Stateless);

%{
    #include <rw/kinematics/StatelessData.hpp>
%}
%include <rw/kinematics/StatelessData.hpp>
//NAMED_OWNEDPTR(StatelessData,rw::kinematics::StatelessData);

%ignore rw::kinematics::StateSetup::getFrame(int) const;
%ignore rw::kinematics::StateSetup::getTree() const;
%{
    #include <rw/kinematics/StateSetup.hpp>
%}
%include <rw/kinematics/StateSetup.hpp>
NAMED_OWNEDPTR(StateSetup, rw::kinematics::StateSetup);

%ignore rw::kinematics::StateStructure::getRoot() const;
%ignore rw::kinematics::StateStructure::addData(rw::kinematics::StateData*);
%{
    #include <rw/kinematics/StateStructure.hpp>
%}
%include <rw/kinematics/StateStructure.hpp>
NAMED_OWNEDPTR(StateStructure, rw::kinematics::StateStructure);

%ignore rw::kinematics::TreeState::getParent(rw::kinematics::Frame const *) const;
%{
    #include <rw/kinematics/TreeState.hpp>
%}
%include <rw/kinematics/TreeState.hpp>
NAMED_OWNEDPTR(TreeState,rw::kinematics::TreeState);
