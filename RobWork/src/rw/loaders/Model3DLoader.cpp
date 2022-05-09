#include "Model3DLoader.hpp"
#include <RobWorkConfig.hpp>
#include <rw/core/Extension.hpp>
#include <rw/core/StringUtil.hpp>
#include <rw/loaders/model3d/Loader3DS.hpp>
#include <rw/loaders/model3d/LoaderAC3D.hpp>
#include <rw/loaders/model3d/LoaderAssimp.hpp>
#include <rw/loaders/model3d/LoaderOBJ.hpp>
#include <rw/loaders/model3d/LoaderSTEP.hpp>
#include <rw/loaders/model3d/LoaderSTL.hpp>
#include <rw/loaders/model3d/LoaderTRI.hpp>
#include <rw/loaders/model3d/LoaderPCD.hpp>

using namespace rw::loaders;
using namespace rw::core;

std::vector< Model3DLoader::Ptr > staticLoaders ({
    ownedPtr (new LoaderSTL ()), ownedPtr (new Loader3DS ()), ownedPtr (new LoaderOBJ ()),
        ownedPtr (new LoaderAC3D ()), ownedPtr (new LoaderTRI ()), ownedPtr (new LoaderPCD ()),
#if RW_HAVE_ASSIMP
        ownedPtr (new LoaderAssimp ()),
#endif
#if RW_HAVE_OCC
        ownedPtr (new LoaderSTEP ())
#endif
});

rw::core::Ptr< Model3DLoader > Model3DLoader::Factory::getModel3DLoader (const std::string& format,
                                                                         size_t skip)
{
    Model3DLoader::Factory ep;
    std::vector< Extension::Ptr > exts = ep.getExtensions ();
    for (Extension::Ptr ext : exts) {
        if (!ext->getProperties ().has (format))
            continue;
        // else try casting to ImageLoader
        Model3DLoader::Ptr loader = ext->getObject ().cast< Model3DLoader > ();
        if (skip-- == 0) {
            return loader;
        }
    }
    // find NonePlugin loaders
    for (Model3DLoader::Ptr loader : staticLoaders) {
        if (loader->isSupported (format)) {
            if (skip-- == 0) {
                return loader;
            }
        }
    }
    RW_THROW ("No loader using that format exists...");
    return NULL;
}

bool Model3DLoader::Factory::hasModel3DLoader (const std::string& format)
{
    Model3DLoader::Factory ep;
    std::vector< Extension::Descriptor > exts = ep.getExtensionDescriptors ();
    for (Extension::Descriptor& ext : exts) {
        if (!ext.getProperties ().has (format))
            continue;
        return true;
    }
    // find NonePlugin loaders
    for (Model3DLoader::Ptr loader : staticLoaders) {
        if (loader->isSupported (format)) {
            return true;
        }
    }
    return false;
}

namespace {
std::vector< std::string >& operator+= (std::vector< std::string >& lhs,
                                        const std::vector< std::string >& rhs)
{
    lhs.insert (lhs.end (), rhs.begin (), rhs.end ());
    return lhs;
}
}    // namespace

std::vector< std::string > Model3DLoader::Factory::getSupportedFormats ()
{
    std::vector< std::string > formats;
    Model3DLoader::Factory ep;
    std::vector< Extension::Ptr > exts = ep.getExtensions ();
    for (Extension::Ptr ext : exts) {
        Model3DLoader::Ptr loader = ext->getObject ().cast< Model3DLoader > ();
        if (!loader.isNull ()) {
            formats += loader->getModelFormats ();
        }
    }
    for (Model3DLoader::Ptr loader : staticLoaders) {
        formats += loader->getModelFormats ();
    }
    return formats;
}

bool Model3DLoader::isSupported (std::string format)
{
    std::vector< std::string > support = getModelFormats ();
    format                             = StringUtil::toUpper (format);
    return std::find (support.begin (), support.end (), format) != support.end ();
}

void Model3DLoader::setDefaultName (std::string name)
{
    _defaultName = name;
}

void Model3DLoader::setDefaultMaterial (rw::geometry::Model3D::Material mat)
{
    _defaultMat = mat;
}
Model3DLoader::Model3DLoader () : _defaultName (""), _defaultMat ("gray", 0.7f, 0.7f, 0.7f)
{}