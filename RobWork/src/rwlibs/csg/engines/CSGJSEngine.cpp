#include <rw/core/Ptr.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/math/Vector3D.hpp>
#include <rwlibs/csg/engines/CSGJSEngine.hpp>

#define CSGJS_HEADER_ONLY
#include <csgjs/csgjs.cpp>

using namespace rw::geometry;
using namespace rw::core;
using namespace rw::math;
using namespace rwlibs::csg::engines;

namespace {
rw::core::Ptr<csgjs_model> toCSGJS(TriMeshData::Ptr m1) {
    std::vector<std::vector<Vector3D<double>>> normals(m1->_vertecies.rows());
    Ptr<csgjs_model> model = ownedPtr(new csgjs_model);

    for(Eigen::Index i = 0; i < m1->_vertecies.rows(); i++) {
        csgjs_vertex v;
        v.pos.x = m1->_vertecies(i, 0);
        v.pos.y = m1->_vertecies(i, 1);
        v.pos.z = m1->_vertecies(i, 2);
        model->vertices.push_back(v);
    }
    for(Eigen::Index i = 0; i < m1->_triangles.rows(); i++) {
        int v1 = m1->_triangles(i, 0);
        int v2 = m1->_triangles(i, 1);
        int v3 = m1->_triangles(i, 2);

        Triangle<double> t(
            Vector3D<double>(m1->_vertecies(v1, 0), m1->_vertecies(v1, 1), m1->_vertecies(v1, 2)),
            Vector3D<double>(m1->_vertecies(v2, 0), m1->_vertecies(v2, 1), m1->_vertecies(v2, 2)),
            Vector3D<double>(m1->_vertecies(v3, 0), m1->_vertecies(v3, 1), m1->_vertecies(v3, 2)));
        model->indices.push_back(v1);
        model->indices.push_back(v2);
        model->indices.push_back(v3);

        normals[v1].push_back(t.calcFaceNormal());
        normals[v2].push_back(t.calcFaceNormal());
        normals[v3].push_back(t.calcFaceNormal());
    }

    for(size_t i = 0; i < normals.size(); i++) {
        Vector3D<double> n;
        for(size_t j = 0; j < normals[i].size(); j++) { n += normals[i][j]; }
        n /= n.norm2();
        model->vertices[i].normal.x = n[0];
        model->vertices[i].normal.x = n[1];
        model->vertices[i].normal.x = n[2];
    }
    return model;
}

TriMeshData::Ptr fromCSGJS(Ptr<csgjs_model> m1) {
    TriMeshData::Ptr res = ownedPtr(new TriMeshData);

    res->_vertecies.resize(m1->vertices.size(), 3);
    res->_triangles.resize(m1->indices.size() / 3, 3);
    for(Eigen::Index i = 0; i < (Eigen::Index) m1->vertices.size(); i++) {
        res->_vertecies(i, 0) = m1->vertices[i].pos.x;
        res->_vertecies(i, 1) = m1->vertices[i].pos.y;
        res->_vertecies(i, 2) = m1->vertices[i].pos.z;
    }

    for(Eigen::Index i = 0; i < (Eigen::Index) m1->indices.size(); i++) {
        res->_triangles(i / 3, i % 3) = m1->indices[i];
    }
    return res;
}

}    // namespace

TriMeshData::Ptr CSGJSEngine::Union(TriMeshData::Ptr m1, TriMeshData::Ptr m2) const {
    rw::core::Ptr<csgjs_model> c1 = toCSGJS(m1);
    rw::core::Ptr<csgjs_model> c2 = toCSGJS(m2);
    return fromCSGJS(ownedPtr(new csgjs_model(csgjs_union(*c1, *c2))));
}

TriMeshData::Ptr CSGJSEngine::Difference(TriMeshData::Ptr m1, TriMeshData::Ptr m2) const {
    rw::core::Ptr<csgjs_model> c1 = toCSGJS(m1);
    rw::core::Ptr<csgjs_model> c2 = toCSGJS(m2);
    return fromCSGJS(ownedPtr(new csgjs_model(csgjs_difference(*c1, *c2))));
}

TriMeshData::Ptr CSGJSEngine::Intersection(TriMeshData::Ptr m1, TriMeshData::Ptr m2) const {
    rw::core::Ptr<csgjs_model> c1 = toCSGJS(m1);
    rw::core::Ptr<csgjs_model> c2 = toCSGJS(m2);

    return fromCSGJS(ownedPtr(new csgjs_model(csgjs_intersection(*c1, *c2))));
}

TriMeshData::Ptr CSGJSEngine::SymmetricDifference(TriMeshData::Ptr m1, TriMeshData::Ptr m2) const {
    rw::core::Ptr<csgjs_model> c1 = toCSGJS(m1);
    rw::core::Ptr<csgjs_model> c2 = toCSGJS(m2);

    rw::core::Ptr<csgjs_model> c3 = ownedPtr(new csgjs_model(csgjs_union(*c1, *c2)));
    rw::core::Ptr<csgjs_model> c4 = ownedPtr(new csgjs_model(csgjs_intersection(*c1, *c2)));

    return fromCSGJS(ownedPtr(new csgjs_model(csgjs_difference(*c3, *c4))));
}

RW_ADD_PLUGIN(CSGEJSEnginePlugin)

CSGEJSEnginePlugin::CSGEJSEnginePlugin() :
    Plugin("CSGEJSEnginePlugin", "CSGEJSEnginePlugin", "1.0") {}

CSGEJSEnginePlugin::~CSGEJSEnginePlugin() {}

std::vector<rw::core::Extension::Descriptor> CSGEJSEnginePlugin::getExtensionDescriptors() {
    std::vector<Extension::Descriptor> exts;
    exts.push_back(Extension::Descriptor("CSGJSEngine", "rw.geometry.CSGEngine"));
    exts.back().getProperties().set<std::string>("EngineID", CSGJSEngine().getID());

    return exts;
}

//! @copydoc rw::core::Plugin::makeExtension
rw::core::Ptr<rw::core::Extension> CSGEJSEnginePlugin::makeExtension(const std::string& id) {
    static const CSGJSEngine::Ptr engine = ownedPtr(new CSGJSEngine());

    const Extension::Ptr extension = ownedPtr(
        new Extension("CSGJSEngine", "rw.geometry.CSGEngine", this, engine.cast<CSGEngine>()));

    extension->getProperties().set<std::string>("EngineID", engine->getID());
    return extension;
}

//! @brief Register the plugins extensions in the rw::core::ExtensionRegistry.
void CSGEJSEnginePlugin::registerPlugin() {
    ExtensionRegistry::getInstance()->registerExtensions(ownedPtr(new CSGEJSEnginePlugin()));
}
