#include "Model3DLoader.hpp"

#include <rw/core/Extension.hpp>

using namespace rw::loaders;
using namespace rw::core;

rw::core::Ptr< Model3DLoader > Model3DLoader::Factory::getModel3DLoader (const std::string& format)
{
    Model3DLoader::Factory ep;
    std::vector< Extension::Ptr > exts = ep.getExtensions ();
    for (Extension::Ptr ext : exts) {
        if (!ext->getProperties ().has (format))
            continue;
        // else try casting to ImageLoader
        Model3DLoader::Ptr loader = ext->getObject ().cast< Model3DLoader > ();

        return loader;
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
    return false;
}

std::vector< std::string > Model3DLoader::Factory::getSupportedFormats ()
{
    std::set< std::string > formats;
    Model3DLoader::Factory ep;
    std::vector< Extension::Ptr > exts = ep.getExtensions ();
    for (Extension::Ptr ext : exts) {
        Model3DLoader::Ptr loader = ext->getObject ().cast< Model3DLoader > ();
        if (!loader.isNull ()) {
            const std::vector< std::string > extFormats = loader->getModelFormats ();
            formats.insert (extFormats.begin (), extFormats.end ());
        }
    }
    return std::vector< std::string > (formats.begin (), formats.end ());
}