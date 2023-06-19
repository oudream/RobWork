#include <rw/core/Extension.hpp>
#include <rw/geometry/CSGEngine.hpp>
using namespace rw::geometry;
using namespace rw::core;

namespace{
    CSGEngine::Ptr _defaultEngine(NULL);
}


CSGEngine::Ptr CSGEngine::Factory::getDefaultEngine() {
    if(_defaultEngine.isNull()) {
        CSGEngine::Factory ep;
        std::vector<Extension::Ptr> exts = ep.getExtensions();
        if(exts.size() > 0) { _defaultEngine = exts.back()->getObject().cast<CSGEngine>().get(); }
    }
    return _defaultEngine;
}

void CSGEngine::Factory::setDefaultEngine(CSGEngine::Ptr engine) {
    //_defaultEngine = engine;
}

std::vector<std::string> CSGEngine::Factory::getAvailableEngines() {
    std::vector<std::string> ids;
    CSGEngine::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    for(Extension::Descriptor& ext : exts) {
        ids.push_back(ext.getProperties().get("EngineID", ext.name));
    }
    return ids;
}

CSGEngine::Ptr CSGEngine::Factory::getCSGEngine(std::string id) {
    std::string upper = id;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
    CSGEngine::Factory ep;
    std::vector<Extension::Ptr> exts = ep.getExtensions();
    for(Extension::Ptr& ext : exts) {
        std::string id = ext->getProperties().get("EngineID", ext->getName());
        std::transform(id.begin(), id.end(), id.begin(), ::toupper);
        if(id == upper) { return ext->getObject().cast<CSGEngine>(); }
    }
    return NULL;
}

bool CSGEngine::Factory::hasEngine(const std::string& engine) {
    std::string upper = engine;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

    CSGEngine::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    for(Extension::Descriptor& ext : exts) {
        std::string id = ext.getProperties().get("EngineID", ext.name);
        std::transform(id.begin(), id.end(), id.begin(), ::toupper);
        if(id == upper) return true;
    }
    return false;
}

