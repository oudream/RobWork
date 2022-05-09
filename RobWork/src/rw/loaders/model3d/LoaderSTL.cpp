#include <rw/loaders/model3d/LoaderSTL.hpp>

#include <boost/filesystem/path.hpp>

using namespace rw::loaders;
using namespace rw::geometry;
using namespace rw::core;

rw::graphics::Model3D::Ptr LoaderSTL::load (const std::string& filename)
{
    std::string name = _defaultName;
    if (name.empty ()) {
        name = boost::filesystem::path (filename).filename ().c_str ();
    }

    rw::geometry::PlainTriMeshN1F::Ptr mesh = STLFile::load (filename);
    Model3D::Ptr model                      = rw::core::ownedPtr (new Model3D (name));

    model->addTriMesh (_defaultMat, *mesh);

    return model;
}
