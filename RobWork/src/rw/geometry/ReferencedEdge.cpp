#include <rw/geometry/ReferencedEdge.hpp>

using namespace rw::geometry;

ReferencedEdge::ReferencedEdge() : _mesh(NULL) {}

ReferencedEdge::ReferencedEdge(TriMeshData::Ptr ref, uint32_t vertice1, uint32_t vertice2) :
    _mesh(ref), _edgeIdx1(vertice1), _edgeIdx2(vertice2) {
    RW_ASSERT(_mesh != NULL);
}

ReferencedVertice ReferencedEdge::operator[](size_t i) const {
    if(i == 0) return ReferencedVertice(_mesh, _edgeIdx1);
    return ReferencedVertice(_mesh, _edgeIdx2);
}

uint32_t ReferencedEdge::idx(size_t i) const {
    if(i == 0) return _edgeIdx1;
    return _edgeIdx2;
}

uint32_t& ReferencedEdge::idx(size_t i) {
    if(i == 0) return _edgeIdx1;
    return _edgeIdx2;
}

bool ReferencedEdge::has(const ReferencedVertice& v) const {
    if(_mesh != v._mesh) return false;
    if(_edgeIdx2 == v.idx()) return true;
    if(_edgeIdx1 == v.idx()) return true;
    return false;
}

bool ReferencedEdge::has(const ReferencedVertice& v1, const ReferencedVertice& v2) const {
    return has(v1) && has(v2);
}
