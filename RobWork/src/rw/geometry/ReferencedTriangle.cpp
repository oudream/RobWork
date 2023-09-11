#include <rw/core/macros.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/geometry/ReferencedTriangle.hpp>
#include <rw/geometry/TriTriIntersectDeviller.hpp>

using namespace rw::geometry;
using namespace rw::math;

ReferencedTriangle::ReferencedTriangle(TriMeshData::Ptr ref, uint32_t triangle) :
    _mesh(ref), _triIndex(triangle) {
    RW_ASSERT(_mesh != NULL);
}

ReferencedVertice ReferencedTriangle::operator[](size_t i) const {
    return ReferencedVertice(_mesh, _mesh->_triangles(_triIndex, i));
}

uint32_t ReferencedTriangle::idx(size_t i) const {
    return _mesh->_triangles(_triIndex, i);
}

uint32_t& ReferencedTriangle::idx(size_t i) {
    return _mesh->_triangles(_triIndex, i);
}

ReferencedEdge ReferencedTriangle::edge(size_t i) const {
    i = i % 3;
    return ReferencedEdge(_mesh, idx(i), idx((i + 1) % 3));
}

bool ReferencedTriangle::has(const ReferencedEdge& edge) const {
    if(has(edge[0]) && has(edge[1])) return true;
    return false;
}

bool ReferencedTriangle::has(const ReferencedVertice& vertice) const {
    if(vertice._mesh != _mesh) return false;
    if(idx(0) == vertice.idx()) return true;
    if(idx(1) == vertice.idx()) return true;
    if(idx(2) == vertice.idx()) return true;
    return false;
}

ReferencedEdge ReferencedTriangle::opposit(const ReferencedVertice& vertice) const {
    if(vertice._mesh != _mesh) { RW_THROW("vertice is not part of same mesh as triangle"); }
    if(vertice.idx() == idx(0)) return edge(1);
    if(vertice.idx() == idx(1)) return edge(2);
    if(vertice.idx() == idx(2)) return edge(0);
    RW_THROW("The vertice is not part of the triangle");
}

ReferencedVertice ReferencedTriangle::opposit(const ReferencedEdge& edge) const {
    if(edge._mesh != _mesh) { RW_THROW("Edge is not part of same mesh as triangle"); }

    if(idx(0) == edge._edgeIdx1) {
        if(idx(1) == edge._edgeIdx2) { return (*this)[2]; }
        else if(idx(2) == edge._edgeIdx2) { return (*this)[1]; }
    }
    else if(idx(1) == edge._edgeIdx1) {
        if(idx(0) == edge._edgeIdx2) { return (*this)[2]; }
        else if(idx(2) == edge._edgeIdx2) { return (*this)[0]; }
    }
    else if(idx(2) == edge._edgeIdx1) {
        if(idx(1) == edge._edgeIdx2) { return (*this)[0]; }
        else if(idx(0) == edge._edgeIdx2) { return (*this)[1]; }
    }
    RW_THROW("Edge is not part of the triangle");
}

bool ReferencedTriangle::intersects(const ReferencedTriangle& t2) const {
    ReferencedTriangle t1(_mesh, _triIndex);
    if(t1._mesh != t2._mesh) return false;
    if(t1._triIndex == t2._triIndex) return false;

    bool has1 = has(t2[0]);
    bool has2 = has(t2[1]);
    bool has3 = has(t2[2]);

    // Check if the triangles are on top of eachother
    if(has1 && has2 && has3) return true;

    if(has1 && has2) return false;
    if(has2 && has3) return false;
    if(has3 && has1) return false;

    ReferencedEdge e;
    Vector3D<double> s;
    if(has1) {
        e = t2.edge(1);
        s = t2[0];
    }
    else if(has2) {
        e = t2.edge(2);
        s = t2[1];
    }
    else if(has3) {
        e = t2.edge(0);
        s = t2[2];
    }

    if(e._mesh) {
        Vector3D<double> e1 = e[0];
        Vector3D<double> e2 = e[1];
        Plane p(t1[0], t1[1], t1[2]);
        double d1 = p.distance(e1);
        double d2 = p.distance(e2);
        if(d1 * d2 > 0) { return false; }

        Vector3D<double> intersect = p.intersection(e1, e2);

        Vector3D<double> is  = intersect - s;
        Vector3D<double> e1s = e1 - s;
        Vector3D<double> e2s = e2 - s;

        double a1 = acos(e1s.dot(e2s) / (e1s.norm2() * e2s.norm2()));
        double a2 = acos(e1s.dot(is) / (e1s.norm2() * is.norm2()));

        if(a1 * a2 < 0) return false;
        if(abs(a1) < abs(a2)) return false;
        return true;
    }

    TriTriIntersectDeviller<double> col;
    return col.inCollision(t1[0], t1[1], t1[2], t2[0], t2[1], t2[2]);
}

bool ReferencedTriangle::sameMesh(const ReferencedTriangle& t) const {
    return _mesh == t._mesh;
}

ReferencedTriangle& ReferencedTriangle::operator=(rw::geometry::Triangle<double> rhs) {
    (*this)[0] = rhs[0];
    (*this)[1] = rhs[1];
    (*this)[2] = rhs[2];
    return *this;
}

ReferencedTriangle& ReferencedTriangle::operator=(rw::geometry::Triangle<float> rhs) {
    (*this)[0] = rhs[0];
    (*this)[1] = rhs[1];
    (*this)[2] = rhs[2];
    return *this;
}

ReferencedTriangle& ReferencedTriangle::operator=(rw::math::Vector3D<int> rhs) {
    _mesh->_triangles(_triIndex, 0) = rhs[0];
    _mesh->_triangles(_triIndex, 1) = rhs[1];
    _mesh->_triangles(_triIndex, 2) = rhs[2];

    return *this;
}

ReferencedTriangle::operator rw::geometry::Triangle<double>() const {
    return rw::geometry::Triangle<double>((*this)[0], (*this)[1], (*this)[2]);
}

ReferencedTriangle::operator rw::geometry::Triangle<float>() const {
    return rw::geometry::Triangle<float>((*this)[0], (*this)[1], (*this)[2]);
}

ReferencedTriangle::operator int() const {
    return _triIndex;
}

namespace rw { namespace geometry {
    std::ostream& operator<<(std::ostream& os, const ReferencedTriangle& t) {
        {
            os << "{"
               << "[" << t.idx(0) << "] " << t[0] << ", "
               << "[" << t.idx(1) << "] " << t[1] << ", "
               << "[" << t.idx(2) << "] " << t[2] << "}";
            return os;
        }
    }
}}    // namespace rw::geometry
