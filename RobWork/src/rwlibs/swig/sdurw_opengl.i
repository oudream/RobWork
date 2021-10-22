%module sdurw_opengl

%{
#include <RobWorkConfig.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/PrismaticSphericalJoint.hpp>
#include <rw/models/PrismaticUniversalJoint.hpp>
#include <rw/models/SphericalJoint.hpp>
#include <rw/models/UniversalJoint.hpp>
#include <rw/models/VirtualJoint.hpp>
#include <rw/models/DependentJoint.hpp>
#include <rw/models/DependentRevoluteJoint.hpp>
#include <rw/models/DependentPrismaticJoint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>

using namespace rwlibs::swig;

%}
%include <exception.i>
%include <std_vector.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/sdurw_sensor.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_sensor.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_sensor.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_sensor.*;
%}

/**
 * @brief RenderImage renders a image in a plane defined by
 * [-w/2;h/2][w/2;-h/2]. The image need to be scaled into metric
 * units. This can be done using a scaling parameter.
 */
class RenderImage
{
public:
    /**
     * @brief constructor
     * @param scale [in] scale from image coordinates to meters.
     */
    RenderImage(float scale=1.0/1000.0);

    /**
     * @brief Constructs
     * @param img [in} the image that is to be rendered
     * @param scale [in] scale from image coordinates to meters.
     */
    RenderImage(const rw::sensor::Image& img, float scale=1.0/1000.0);

    /**
     * @brief set the image that is to be rendered.
     * @param img [in] image to render
     */
    void setImage(const rw::sensor::Image& img);

    /* Functions inherited from Render */

    //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
    void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                rw::graphics::DrawableNode::DrawType type,
                double alpha) const;
};


/**
 * @brief Legacy type of a smart pointer for RenderImage.
 * @deprecated Use RenderImage::Ptr instead. This type will be removed sometime in the future.
 */
%template (RenderImagePtr) rw::core::Ptr<RenderImage>;
OWNEDPTR(RenderImage);