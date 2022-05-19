%module sdurw_graphics

%include <exception.i>
%include <rwlibs/swig/swig_macros.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_geometry.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/sdurw_sensor.i>

%import <rwlibs/swig/ext_i/std.i>

%pragma(java) jniclassimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
    import org.robwork.sdurw_kinematics.*;
    import org.robwork.sdurw_geometry.*;
    import org.robwork.sdurw_models.*;
    import org.robwork.sdurw_sensor.*;
%}
%pragma(java) moduleimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
    import org.robwork.sdurw_kinematics.*;
    import org.robwork.sdurw_geometry.*;
    import org.robwork.sdurw_models.*;
    import org.robwork.sdurw_sensor.*;
%}
%typemap(javaimports) SWIGTYPE %{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
    import org.robwork.sdurw_kinematics.*;
    import org.robwork.sdurw_geometry.*;
    import org.robwork.sdurw_models.*;
    import org.robwork.sdurw_sensor.*;
%}
%{
    #include <rw/geometry/Geometry.hpp>
    #include <rw/geometry/Model3D.hpp>
    #include <rw/geometry/IndexedTriMesh.hpp>
    #include <rw/kinematics/MovableFrame.hpp>
    #include <rw/kinematics/FixedFrame.hpp>
    #include <rw/geometry/Line.hpp>
    #include <rw/models.hpp>
%}

%nodefaultctor rw::graphics::DrawableNode;
%nodefaultctor rw::graphics::WorkCellScene;


%{
	#include <rw/graphics/Model3D.hpp>
%}
%include <rw/graphics/Model3D.hpp>

%{
	#include <rw/graphics/SceneNode.hpp>
%}
%include <rw/graphics/SceneNode.hpp>
NAMED_OWNEDPTR(SceneNode, rw::graphics::SceneNode);

%{
	#include <rw/graphics/DrawableNode.hpp>
%}
%include <rw/graphics/DrawableNode.hpp>
NAMED_OWNEDPTR(DrawableNode, rw::graphics::DrawableNode);

%{
	#include <rw/graphics/SceneGraph.hpp>
%}
%include <rw/graphics/SceneGraph.hpp>
NAMED_OWNEDPTR(SceneGraph, rw::graphics::SceneGraph);

%{
	#include <rw/graphics/DrawableGeometryNode.hpp>
%}
%include <rw/graphics/DrawableGeometryNode.hpp>
NAMED_OWNEDPTR(DrawableGeometryNode, rw::graphics::DrawableGeometryNode);

%{
	#include <rw/graphics/GroupNode.hpp>
%}
%include <rw/graphics/GroupNode.hpp>

NAMED_OWNEDPTR(GroupNode, rw::graphics::GroupNode);

//%ignore rw::graphics::SceneCamera::_name;
%{
	#include <rw/graphics/SceneCamera.hpp>
%}
%include <rw/graphics/SceneCamera.hpp>

//NAMED_OWNEDPTR(SceneCamera, rw::graphics::SceneCamera);

%{
	#include <rw/graphics/WorkCellScene.hpp>
%}
%include <rw/graphics/WorkCellScene.hpp>

NAMED_OWNEDPTR(WorkCellScene, rw::graphics::WorkCellScene);

%{
	#include <rw/graphics/SceneViewer.hpp>
%}
%include <rw/graphics/SceneViewer.hpp>

NAMED_OWNEDPTR(SceneViewer, rw::graphics::SceneViewer);

%{
	#include <rw/graphics/Render.hpp>
%}
%include <rw/graphics/Render.hpp>

NAMED_OWNEDPTR(Render, rw::graphics::Render);

%{
	#include <rw/graphics/TextureData.hpp>
%}
%include <rw/graphics/TextureData.hpp>

NAMED_OWNEDPTR(TextureData, rw::graphics::TextureData);

%{
	#include <rw/graphics/DrawableNodeClone.hpp>
%}
%include <rw/graphics/DrawableNodeClone.hpp>

NAMED_OWNEDPTR(DrawableNodeClone, rw::graphics::DrawableNodeClone);



%constant int DNodePhysical = rw::graphics::DrawableNode::Physical;
%constant int DNodeVirtual = rw::graphics::DrawableNode::Virtual;
%constant int DNodeDrawableObject = rw::graphics::DrawableNode::DrawableObject;
%constant int DNodeCollisionObject = rw::graphics::DrawableNode::CollisionObject;