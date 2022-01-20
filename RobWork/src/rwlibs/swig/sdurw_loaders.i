%module sdurw_loaders

%include <stl.i>
%include <std_vector.i>
%include <exception.i>
%include <rwlibs/swig/swig_macros.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_geometry.i>
%import <rwlibs/swig/sdurw_sensor.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/sdurw_proximity.i>
%import <rwlibs/swig/sdurw_graphics.i>
%import <rwlibs/swig/sdurw_trajectory.i>

%import <rwlibs/swig/ext_i/std.i>

%{
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

#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
%}

%pragma(java) jniclassimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
    import org.robwork.sdurw_kinematics.*;
    import org.robwork.sdurw_geometry.*;
    import org.robwork.sdurw_sensor.*;
    import org.robwork.sdurw_models.*;
    import org.robwork.sdurw_proximity.*;
    import org.robwork.sdurw_graphics.*;
    import org.robwork.sdurw_trajectory.*;
%}
%pragma(java) moduleimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
    import org.robwork.sdurw_kinematics.*;
    import org.robwork.sdurw_geometry.*;
    import org.robwork.sdurw_sensor.*;
    import org.robwork.sdurw_models.*;
    import org.robwork.sdurw_proximity.*;
    import org.robwork.sdurw_graphics.*;
    import org.robwork.sdurw_trajectory.*;
%}
%typemap(javaimports) SWIGTYPE %{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_common.*;
    import org.robwork.sdurw_math.*;
    import org.robwork.sdurw_kinematics.*;
    import org.robwork.sdurw_geometry.*;
    import org.robwork.sdurw_sensor.*;
    import org.robwork.sdurw_models.*;
    import org.robwork.sdurw_proximity.*;
    import org.robwork.sdurw_graphics.*;
    import org.robwork.sdurw_trajectory.*;
%}


RENAME(rw::loaders::WorkCellLoader::Factory, WorkCellLoaderFactory);
%{
	#include <rw/loaders/WorkCellLoader.hpp>
%}
%template(ExtensionPointWorkCellLoader) rw::core::ExtensionPoint< rw::loaders::WorkCellLoader >;
%include <rw/loaders/WorkCellLoader.hpp>
%template (WorkCellLoaderPtr) rw::core::Ptr<rw::loaders::WorkCellLoader>;

RENAME(rw::loaders::Model3DLoader::Factory,Model3DLoaderFactory);
%{
	#include <rw/loaders/Model3DLoader.hpp>
%}
%template(ExtensionPointModel3DLoader) rw::core::ExtensionPoint< rw::loaders::Model3DLoader >;
%include <rw/loaders/Model3DLoader.hpp>

%{
	#include <rw/loaders/GeometryFactory.hpp>
%}
%template(ExtensionPointGeometryData) rw::core::ExtensionPoint< rw::geometry::GeometryData >;
%template(ExtensionPointGeometryFactory) rw::core::ExtensionPoint< rw::loaders::GeometryFactory >;
%include <rw/loaders/GeometryFactory.hpp>

%{
	#include <rw/loaders/path/PathLoader.hpp>
%}
%include <rw/loaders/path/PathLoader.hpp>

%{
	#include <rw/loaders/path/PathLoaderCSV.hpp>
%}
%include <rw/loaders/path/PathLoaderCSV.hpp>

%{
	#include <rw/loaders/colsetup/CollisionSetupLoader.hpp>
%}
%include <rw/loaders/colsetup/CollisionSetupLoader.hpp>

RENAME(rw::loaders::ImageLoader::Factory,ImageLoaderFactory);
%{
	#include <rw/loaders/ImageLoader.hpp>
%}
%template(ExtensionPointImageLoader) rw::core::ExtensionPoint< rw::loaders::ImageLoader >;
%include <rw/loaders/ImageLoader.hpp>
%template (ImageLoaderPtr) rw::core::Ptr<rw::loaders::ImageLoader>;


/*
#if defined(RW_HAVE_XERCES)
#include <rw/loaders/xml/XMLPathFormat.hpp>
#include <rw/loaders/xml/XMLPathLoader.hpp>
#include <rw/loaders/xml/XMLPathSaver.hpp>
#include <rw/loaders/xml/XMLTrajectoryFormat.hpp>
#include <rw/loaders/xml/XMLTrajectoryLoader.hpp>
#include <rw/loaders/xml/XMLTrajectorySaver.hpp>
#include <rw/loaders/xml/XMLPropertyFormat.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#include <rw/loaders/xml/XMLProximitySetupLoader.hpp>
#include <rw/loaders/dom/DOMPropertyMapFormat.hpp>
#include <rw/loaders/dom/DOMPropertyMapLoader.hpp>
#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>
#include <rw/loaders/dom/DOMPathLoader.hpp>
#include <rw/loaders/dom/DOMPathSaver.hpp>
#include <rw/loaders/dom/DOMTrajectoryLoader.hpp>
#include <rw/loaders/dom/DOMTrajectorySaver.hpp>
#include <rw/loaders/dom/DOMProximitySetupLoader.hpp>
#include <rw/loaders/dom/DOMProximitySetupSaver.hpp>
#include <rw/loaders/dom/DOMWorkCellSaver.hpp>
*/
/*
%{
	#include <rw/loaders/model3d/LoaderAC3D.hpp>
%}
%{
	#include <rw/loaders/model3d/Loader3DS.hpp>
%}
%{
	#include <rw/loaders/model3d/Model3DS.hpp>
%}
%{
	#include <rw/loaders/model3d/LoaderOBJ.hpp>
%}
%{
	#include <rw/loaders/model3d/LoaderTRI.hpp>
%}
%{
	#include <rw/loaders/model3d/STLFile.hpp>
%}
%{
	#include <rw/loaders/rwxml/DependencyGraph.hpp>
%}
%{
	#include <rw/loaders/rwxml/MultipleFileIterator.hpp>
%}
%{
	#include <rw/loaders/rwxml/XMLParserUtil.hpp>
%}
%{
	#include <rw/loaders/rwxml/XMLRWPreParser.hpp>
%}
%{
	#include <rw/loaders/rwxml/XMLRWParser.hpp>
%}
%{
	#include <rw/loaders/rwxml/XMLRWLoader.hpp>
%}
%{
	#include <rw/loaders/rwxml/XMLErrorHandler.hpp>
%}
%{
	#include <rw/loaders/rwxml/XMLParser.hpp>
%}
%{
	#include <rw/loaders/rwxml/XML.hpp>
%}
%{
	#include <rw/loaders/dom/DOMBasisTypes.hpp>
%}
%{
	#include <rw/loaders/dom/DOMPropertyMapFormat.hpp>
%}
%{
	#include <rw/loaders/dom/DOMPropertyMapLoader.hpp>
%}
%{
	#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>
%}
%{
	#include <rw/loaders/dom/DOMPathLoader.hpp>
%}
%{
	#include <rw/loaders/dom/DOMPathSaver.hpp>
%}
%{
	#include <rw/loaders/dom/DOMTrajectoryLoader.hpp>
%}
%{
	#include <rw/loaders/dom/DOMTrajectorySaver.hpp>
%}
%{
	#include <rw/loaders/dom/DOMProximitySetupLoader.hpp>
%}
%{
	#include <rw/loaders/dom/DOMProximitySetupSaver.hpp>
%}
%{
	#include <rw/loaders/dom/DOMWorkCellSaver.hpp>
%}
%{
	#include <rw/loaders/image/PGMLoader.hpp>
%}
%{
	#include <rw/loaders/image/PPMLoader.hpp>
%}
%{
	#include <rw/loaders/image/RGBLoader.hpp>
%}
*/