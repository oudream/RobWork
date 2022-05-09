#include <rw/geometry/Model3D.hpp>
#include <rw/loaders/Model3DFactory.hpp>
#include <rw/loaders/model3d/STLFile.hpp>

#include <boost/filesystem.hpp>

using namespace rw::loaders;
using namespace rw::geometry;

int main (int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <model>" << std::endl;
        return 1;
    }

    std::string file = argv[1];

    // Find file
    if (boost::filesystem::exists (file)) {
        // Auto load for testing
        if (boost::filesystem::is_directory (file)) {
            std::string n_file = file + "/objects/EngineeringTable.STEP";
            if (boost::filesystem::exists (n_file)) {
                file = n_file;
            }
            else {
                std::cout << "The specified file is a directory: " << file << std::endl;
                return 1;
            }
        }
    }
    else {
        std::cout << "Could not find file: " << file << std::endl;
        return 1;
    }

    // Load File
    Model3D::Ptr model = Model3DFactory::loadModel (file, "Steven");
    if (model.isNull ()) {
        std::cout << "Model could not be loaded." << std::endl;
        return 1;
    }
    std::cout << "Model " << model->getName () << " successfully loaded." << std::endl;

    // Save File
    boost::filesystem::path out = file;
    out.replace_extension("");
    out                  =  out.string() + ".rw.stl";
    STLFile::save (model->toGeometryData ()->getTriMesh (), out.string ());

    return 0;
}
