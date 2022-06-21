#include <rw/geometry/CSGEngine.hpp>
#include <rw/core/Extension.hpp>
using namespace rw::geometry;
using namespace rw::core;

CSGEngine::Ptr CSGEngine::Factory::getDefaultEngine ()
{
    CSGEngine::Factory ep;
    std::vector< Extension::Ptr > exts = ep.getExtensions ();
    if (exts.size () > 0)
        return exts[0]->getObject ().cast< CSGEngine > ();
    return NULL;
}

std::vector< std::string > CSGEngine::Factory::getAvailableEngines ()
{
    std::vector< std::string > ids;
    CSGEngine::Factory ep;
    std::vector< Extension::Descriptor > exts = ep.getExtensionDescriptors ();
    for (Extension::Descriptor& ext : exts) {
        ids.push_back (ext.getProperties ().get ("strategyID", ext.name));
    }
    return ids;
}

CSGEngine::Ptr CSGEngine::Factory::getCSGEngine (std::string id)
{
    std::string upper = id;
    std::transform (upper.begin (), upper.end (), upper.begin (), ::toupper);
    CSGEngine::Factory ep;
    std::vector< Extension::Ptr > exts = ep.getExtensions ();
    for (Extension::Ptr& ext : exts) {
        std::string id = ext->getProperties ().get ("strategyID", ext->getName ());
        std::transform (id.begin (), id.end (), id.begin (), ::toupper);
        if (id == upper) {
            return ext->getObject ().cast< CSGEngine > ();
        }
    }
    return NULL;
}

bool CSGEngine::Factory::hasEngine (const std::string& engine)
{
    std::string upper = engine;
    std::transform (upper.begin (), upper.end (), upper.begin (), ::toupper);

    CSGEngine::Factory ep;
    std::vector< Extension::Descriptor > exts = ep.getExtensionDescriptors ();
    for (Extension::Descriptor& ext : exts) {
        std::string id = ext.getProperties ().get ("strategyID", ext.name);
        std::transform (id.begin (), id.end (), id.begin (), ::toupper);
        if (id == upper)
            return true;
    }
    return false;
}