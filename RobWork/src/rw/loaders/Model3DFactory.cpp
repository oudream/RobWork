/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "Model3DFactory.hpp"

#include <RobWorkConfig.hpp>
#include <rw/core/Exception.hpp>
#include <rw/core/IOUtil.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/core/StringUtil.hpp>
#include <rw/core/macros.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/PointCloud.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/loaders/model3d/Loader3DS.hpp>
#include <rw/loaders/model3d/LoaderAC3D.hpp>
#include <rw/loaders/model3d/LoaderAssimp.hpp>
#include <rw/loaders/model3d/LoaderOBJ.hpp>
#include <rw/loaders/model3d/LoaderSTEP.hpp>
#include <rw/loaders/model3d/LoaderSTL.hpp>
#include <rw/loaders/model3d/LoaderTRI.hpp>
#include <rw/loaders/model3d/STLFile.hpp>


#include <sstream>
#include <string>
#include <sys/stat.h>

using namespace rw;
using namespace rw::core;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::sensor;
using namespace rw::loaders;
namespace {
std::string getLastModifiedStr (const std::string& file)
{
    struct stat status;
    stat (file.c_str (), &status);
    // std::cout << "LAST MODIFIED DATE: " << status.st_mtime << std::endl;
    std::stringstream sstr;
    sstr << status.st_mtime;
    return sstr.str ();
}
}    // namespace

Model3D::Ptr Model3DFactory::getModel (const std::string& str, const std::string& name,
                                       bool useCache, Model3D::Material mat)
{
    if (useCache && getCache ().isInCache (str, "")) {
        Model3D::Ptr res = ownedPtr (new Model3D (*getCache ().get (str)));
        res->setName (name);
        return res;
    }
    if (str[0] == '#') {
        return constructFromGeometry (str, name, useCache, mat);
    }
    else {
        return loadModel (str, name, useCache, mat);
    }
}

Model3D::Ptr Model3DFactory::constructFromGeometry (const std::string& str, const std::string& name,
                                                    bool useCache, Model3D::Material mat)
{
    if (useCache) {
        if (getCache ().isInCache (str, "")) {
            Model3D::Ptr res = ownedPtr (new Model3D (*getCache ().get (str)));
            res->setName (name);
            return res;
        }
    }
    Geometry::Ptr geometry = GeometryFactory::getGeometry (str);
    Model3D* model         = new Model3D (name);
    model->addTriMesh (mat, *geometry->getGeometryData ()->getTriMesh ());

    return ownedPtr (model);
}

Model3DFactory::FactoryCache& Model3DFactory::getCache ()
{
    static FactoryCache cache;
    return cache;
}

Model3D::Ptr Model3DFactory::loadModel (const std::string& raw_filename, const std::string& name,
                                        bool useCache, Model3D::Material mat)
{
    std::vector< std::string > extensions = Model3DLoader::Factory::getSupportedFormats ();

    const std::string& filename = IOUtil::resolveFileName (raw_filename, extensions);
    const std::string& filetype = StringUtil::toUpper (StringUtil::getFileExtension (filename));
    // if the file does not exist then throw an exception
    if (filetype.empty ()) {
        RW_THROW ("No file type known for file " << StringUtil::quote (raw_filename)
                                                 << " that was resolved to file name " << filename);
    }

    std::string moddate = getLastModifiedStr (filename);
    if (useCache && getCache ().isInCache (filename, moddate)) {
        Model3D::Ptr res = ownedPtr (new Model3D (*getCache ().get (filename)));
        res->setName (name);
        return res;
    }

    bool gotThrow = false;
    Exception exception;

    if (Model3DLoader::Factory::hasModel3DLoader (filetype)) {
        bool foundLoader = true;
        size_t i         = 0;
        while (foundLoader) {
            Model3DLoader::Ptr loader;
            try {
                loader = Model3DLoader::Factory::getModel3DLoader (filetype, i++);
            }
            catch (...) {
                foundLoader = false;
                break;
            }

            loader->setDefaultMaterial (mat);
            loader->setDefaultName (name);

            try {
                Model3D::Ptr model = loader->load (filename);
                getCache ().add (filename, model, moddate);
                return ownedPtr(new Model3D(*(getCache ().get (filename))));
            }
            catch (const Exception& e) {
                gotThrow  = true;
                exception = e;
            }
        }
    }

    if (gotThrow) {
        throw exception;
    }
    RW_THROW ("Unknown extension " << StringUtil::quote (StringUtil::getFileExtension (filename))
                                   << " for file " << StringUtil::quote (raw_filename)
                                   << " that was resolved to file name " << filename);

    RW_ASSERT (!"Impossible");
    return NULL;    // To avoid a compiler warning.
}
