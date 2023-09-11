#include <rw/geometry/Object3D.hpp>

using namespace rw::geometry;
using namespace rw::math;

void Object3DGeneric::setMaterial(std::size_t material) {
    if(_materialMap.size() == 0 || _materialMap.back().matId != material) {
        _materialMap.push_back(MaterialMapData(material, countFaces(), 0));
    }
}

void Object3DGeneric::getTriangle(size_t idx, rw::geometry::Triangle<double>& dst) const {
    dst = getTriangle(idx);
}

void Object3DGeneric::getTriangle(size_t idx, rw::geometry::Triangle<float>& dst) const {
    rw::geometry::Triangle<double> res = getTriangle(idx);
    dst[0]                             = Vector3D<float>(res[0].e());
    dst[1]                             = Vector3D<float>(res[1].e());
    dst[2]                             = Vector3D<float>(res[2].e());
}