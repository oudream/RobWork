%module sdurw_opengl

%{
#include <RobWorkConfig.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>

using namespace rwlibs::swig;

%}

%include <std_vector.i>
%import <rwlibs/swig/sdurw.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw.*;
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
    RenderImage(const Image& img, float scale=1.0/1000.0);

    /**
     * @brief set the image that is to be rendered.
     * @param img [in] image to render
     */
    void setImage(const Image& img);

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
%template (RenderImagePtr) rw::common::Ptr<RenderImage>;
OWNEDPTR(RenderImage);