#include <rw/geometry/Model3D.hpp>
#include <rw/geometry/SimpleTriMesh.hpp>
#include <rw/loaders/Model3DFactory.hpp>
#include <rw/loaders/model3d/LoaderOBJ.hpp>
#include <rw/loaders/model3d/STLFile.hpp>

#include <boost/filesystem.hpp>
#include <iomanip>
#include <iostream>

using namespace rw::loaders;
using namespace rw::geometry;

std::ostream& operator+ (std::ostream& os, const ReferencedTriangle& tri)
{
    os << "[[" << tri[0][0] << " " << tri[0][1] << " " << tri[0][2] << "];[" << tri[1][0]
              << " " << tri[1][1] << " " << tri[1][2] << "];[" << tri[2][0] << " " << tri[2][1]
              << " " << tri[2][2] << "];[" << tri[0][0] << " " << tri[0][1] << " " << tri[0][2] << "]]";
    return os;
}

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
    
    out.replace_extension ("");
    out = out.string () + ".rw.stl";
    STLFile::save (model->toGeometryData ()->getTriMesh (), out.string ());
    std::cout << "Saved to: " << out.string () << std::endl;

    out.replace_extension ("");
    out.replace_extension ("");
    out = out.string () + ".rw.obj";
    LoaderOBJ obj;
    obj.save (model->toGeometryData ()->getTriMesh (), out.string ());
    std::cout << "Saved to: " << out.string () << std::endl;

    return 0;
}
