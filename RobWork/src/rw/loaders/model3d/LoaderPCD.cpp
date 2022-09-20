#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/PointCloud.hpp>
#include <rw/loaders/model3d/LoaderPCD.hpp>

#include <boost/filesystem/path.hpp>

using namespace rw::loaders;
using namespace rw::geometry;
using namespace rw::core;

rw::graphics::Model3D::Ptr LoaderPCD::load (const std::string& filename)
{
    std::string name = _defaultName;
    if (name.empty ()) {
        name = boost::filesystem::path (filename).filename ().string();
    }

    rw::geometry::PointCloud::Ptr img = rw::geometry::PointCloud::loadPCD (filename);
    Geometry::Ptr geom                = ownedPtr (new Geometry (img));
    // convert to model3d
    Model3D::Ptr model = ownedPtr (new Model3D (name));
    model->addGeometry (_defaultMat, geom);

    return model;
}
