#include <rw/core/Ptr.hpp>
#include <rw/geometry/BSphere.hpp>
#include <rw/geometry/SimpleTriMesh.hpp>

#include <Eigen/Core>
#include <deque>
#include <thread>
#include <vector>

using namespace rw::math;
using namespace rw::geometry;
using namespace rw::core;

SimpleTriMesh::SimpleTriMesh(TriMeshData::Ptr data) :
    _engine(CSGEngine::Factory::getDefaultEngine()) {
    if(data.isNull()) { _data = rw::core::ownedPtr(new TriMeshData()); }
    else if(data.isShared()) { _data = data; }
    else {
        _data = rw::core::ownedPtr(new TriMeshData());

        _data->_triangles = data->_triangles;
        _data->_vertecies = data->_vertecies;
    }
}

SimpleTriMesh::~SimpleTriMesh() {
    if(!_data.isShared()) { delete _data.get(); }
}

SimpleTriMesh::SimpleTriMesh(const SimpleTriMesh& copy) :
    _data(rw::core::ownedPtr(new TriMeshData())), _engine(copy._engine) {
    _data->_triangles = copy._data->_triangles;
    _data->_vertecies = copy._data->_vertecies;
}

SimpleTriMesh::SimpleTriMesh(const SimpleTriMesh&& copy) :
    _data(copy._data), _engine(copy._engine) {
    if(_data.isNull()) { _data = rw::core::ownedPtr(new TriMeshData()); }
    else if(!_data.isShared()) {
        _data             = rw::core::ownedPtr(new TriMeshData());
        _data->_triangles = copy._data->_triangles;
        _data->_vertecies = copy._data->_vertecies;
    }
}

SimpleTriMesh::SimpleTriMesh(const rw::core::Ptr<SimpleTriMesh>& copy) :
    _data(rw::core::ownedPtr(new TriMeshData())),
    _engine(copy.isNull() ? CSGEngine::Factory::getDefaultEngine() : copy->_engine) {
    if(copy.isNull()) return;
    _data->_triangles = copy->_data->_triangles;
    _data->_vertecies = copy->_data->_vertecies;
}
SimpleTriMesh::SimpleTriMesh(const rw::geometry::TriMesh& copy) :
    _data(rw::core::ownedPtr(new TriMeshData())), _engine(CSGEngine::Factory::getDefaultEngine()) {
    fromTriMesh(copy);
}

SimpleTriMesh::SimpleTriMesh(rw::geometry::GeometryData& copy) : SimpleTriMesh(copy.getTriMesh()) {}

SimpleTriMesh::SimpleTriMesh(rw::geometry::GeometryData&& copy) :
    SimpleTriMesh(copy.getTriMesh()) {}

SimpleTriMesh::SimpleTriMesh(const rw::core::Ptr<rw::geometry::GeometryData>& copy) :
    SimpleTriMesh(copy->getTriMesh()) {}

SimpleTriMesh::SimpleTriMesh(const rw::core::Ptr<rw::geometry::TriMesh>& copy) :
    SimpleTriMesh(*copy) {}

SimpleTriMesh::SimpleTriMesh(std::vector<ReferencedTriangle> triangles,
                             rw::core::Ptr<TriMeshData> data) :
    _data(rw::core::ownedPtr(new TriMeshData())),
    _engine(CSGEngine::Factory::getDefaultEngine()) {
    _data->_triangles.resize(triangles.size(), 3);
    std::map<int, int> idx2idx;

    uint32_t idx  = 0;
    uint32_t loop = 0;
    for(const ReferencedTriangle& t : triangles) {
        for(size_t i = 0; i < 3; i++) {
            int index = t.idx(i);
            if(idx2idx.find(index) == idx2idx.end()) { idx2idx[index] = idx++; }
            triangle(loop).idx(i) = idx2idx[index];
        }
        loop++;
    }
    _data->_vertecies.resize(idx, 3);

    for(std::pair<int, int> i2i : idx2idx) {
        ReferencedVertice old(data, i2i.first);
        ReferencedVertice nEw(_data, i2i.second);
        nEw = (Vector3D<double>) old;
    }
}

rw::geometry::Triangle<double> SimpleTriMesh::getTriangle(size_t idx) const {
    return this->triangle(idx);
}

void SimpleTriMesh::getTriangle(size_t idx, rw::geometry::Triangle<double>& dst) const {
    dst = this->triangle(idx);
}

void SimpleTriMesh::getTriangle(size_t idx, rw::geometry::Triangle<float>& dst) const {
    dst = this->triangle(idx);
}

size_t SimpleTriMesh::size() const {
    return this->_data->_triangles.rows();
}

rw::core::Ptr<TriMesh> SimpleTriMesh::clone() const {
    return rw::core::ownedPtr(new SimpleTriMesh(*this));
}

rw::core::Ptr<TriMesh> SimpleTriMesh::getTriMesh(bool forceCopy) {
    if(forceCopy || !this->_data.isShared()) { return clone(); }
    else { return rw::core::ownedPtr(new SimpleTriMesh(this->_data)); }
}

void SimpleTriMesh::scale(double scale) {
    for(Eigen::Index i = 0; i < _data->_vertecies.rows(); i++) {
        for(Eigen::Index j = 0; j < 3; j++) { _data->_vertecies(i, j) *= scale; }
    }
}

void SimpleTriMesh::scale(const rw::math::Vector3D<double>& scale) {
    for(Eigen::Index i = 0; i < _data->_vertecies.rows(); i++) {
        for(Eigen::Index j = 0; j < 3; j++) { _data->_vertecies(i, j) *= scale[j]; }
    }
}

ReferencedTriangle SimpleTriMesh::triangle(size_t idx) const {
    return ReferencedTriangle(_data, (uint32_t) idx);
}

size_t SimpleTriMesh::triangles() const {
    return _data->_triangles.rows();
}

ReferencedVertice SimpleTriMesh::vertice(size_t idx) const {
    return ReferencedVertice(_data, (uint32_t) idx);
}

size_t SimpleTriMesh::vertices() const {
    return _data->_vertecies.rows();
}

void SimpleTriMesh::resize(size_t triangles, size_t vertices) {
    _data->_triangles.resize(triangles, 3);
    _data->_vertecies.resize(vertices, 3);
}

struct Edge
{
    Edge(uint32_t i, uint32_t k) : i(i), k(k) {}
    Edge(const ReferencedTriangle& tri, int index) {
        index = index % 3;
        i     = tri.idx(index);
        k     = tri.idx((index + 1) % 3);

        if(i > k) {
            uint32_t tmp = i;
            i            = k;
            k            = tmp;
        }
    }
    uint32_t i;
    uint32_t k;

    bool operator<(const Edge& rhs) const { return i < rhs.i || (i == rhs.i && k < rhs.k); }
    bool operator==(const Edge& rhs) const { return i == rhs.i && k == rhs.k; }
    bool operator>(const Edge& rhs) const { return !(*this == rhs || *this < rhs); }
};

std::vector<SimpleTriMesh> SimpleTriMesh::separateMeshes() const {
    std::map<Edge, std::vector<ReferencedTriangle>> conn;

    for(size_t i = 0; i < this->triangles(); i++) {
        ReferencedTriangle t = this->triangle(i);
        conn[Edge(t, 0)].push_back(t);
        conn[Edge(t, 1)].push_back(t);
        conn[Edge(t, 2)].push_back(t);
    }

    std::vector<int> visited(this->triangles(), false);
    std::vector<SimpleTriMesh> res;

    for(size_t i = 0; i < this->triangles(); i++) {
        if(!visited[i]) {
            std::vector<ReferencedTriangle> newMesh;
            std::deque<ReferencedTriangle> toDo;

            toDo.push_back(this->triangle(i));
            while(!toDo.empty()) {
                ReferencedTriangle t = toDo.front();

                toDo.pop_front();
                if(!visited[t]) {
                    newMesh.push_back(t);
                    visited[t] = true;

                    for(size_t j = 0; j < 3; j++) {
                        for(const ReferencedTriangle& t_new : conn[Edge(t, (int) j)]) {
                            toDo.push_back(t_new);
                        }
                    }
                }
            }
            res.push_back(SimpleTriMesh(newMesh, _data));
        }
    }
    return res;
}

SimpleTriMesh SimpleTriMesh::combine(const SimpleTriMesh& mesh) const {
    TriMeshData::Ptr data                          = ownedPtr(new TriMeshData);
    Eigen::Matrix<uint32_t, Eigen::Infinity, 3>& t = data->_triangles;
    Eigen::Matrix<double, Eigen::Infinity, 3>& v   = data->_vertecies;

    t.resize(triangles() + mesh.triangles(), 3);
    t << _data->_triangles, mesh._data->_triangles;

    v.resize(vertices() + mesh.vertices(), 3);
    v << _data->_vertecies, mesh._data->_vertecies;

    for(Eigen::Index i = _data->_triangles.rows(); i < t.rows(); i++) {
        t(i, 0) += vertices();
        t(i, 1) += vertices();
        t(i, 2) += vertices();
    }

    return SimpleTriMesh(data);
}

namespace {
size_t indexOf(std::vector<Vector3D<double>>& vec, Vector3D<double> elem) {
    std::vector<Vector3D<double>>::iterator it;
    for(size_t i = 0; i < vec.size(); i++) {
        if(vec[i] == elem) { return i; }

        /*if ((vec[i] - elem).norm2 () < prox) {
            return i;
        }*/
    }

    vec.push_back(elem);
    return vec.size() - 1;
}
}    // namespace

void SimpleTriMesh::fromTriMesh(const rw::geometry::TriMesh& copy) {
    std::vector<Vector3D<double>> vertices;
    std::vector<Vector3D<uint32_t>> triangles;
    for(size_t i = 0; i < copy.size(); i++) {
        Vector3D<uint32_t> tri;
        Triangle<double> old_tri = copy.getTriangle(i);
        for(size_t j = 0; j < 3; j++) { tri[j] = indexOf(vertices, old_tri[j]); }
        triangles.push_back(tri);
    }

    this->resize(triangles.size(), vertices.size());

    for(size_t i = 0; i < vertices.size(); i++) {
        for(size_t j = 0; j < 3; j++) { _data->_vertecies(i, j) = vertices[i][j]; }
    }
    for(size_t i = 0; i < triangles.size(); i++) {
        for(size_t j = 0; j < 3; j++) { _data->_triangles(i, j) = triangles[i][j]; }
    }
}

SimpleTriMesh& SimpleTriMesh::operator*=(const rw::math::Transform3D<double>& trans) {
    for(size_t i = 0; i < vertices(); i++) { vertice(i) *= trans; }
    return *this;
}

SimpleTriMesh SimpleTriMesh::operator*(const rw::math::Transform3D<double>& trans) const {
    SimpleTriMesh t(*this);
    for(size_t i = 0; i < t.vertices(); i++) { t.vertice(i) *= trans; }
    return t;
}

SimpleTriMesh SimpleTriMesh::operator+(const SimpleTriMesh& rhs) const {
    if(_engine.isNull()) { RW_THROW("NO CSGEngine Found"); }

    auto data = _engine->Union(this->_data, rhs._data);
    return SimpleTriMesh(data);
}

SimpleTriMesh& SimpleTriMesh::operator+=(const SimpleTriMesh& rhs) {
    if(_engine.isNull()) { RW_THROW("NO CSGEngine Found"); }
    this->_data = _engine->Union(this->_data, rhs._data);
    return *this;
}

SimpleTriMesh SimpleTriMesh::operator-(const SimpleTriMesh& rhs) const {
    if(_engine.isNull()) { RW_THROW("NO CSGEngine Found"); }
    return SimpleTriMesh(_engine->Difference(this->_data, rhs._data));
}

SimpleTriMesh& SimpleTriMesh::operator-=(const SimpleTriMesh& rhs) {
    if(_engine.isNull()) { RW_THROW("NO CSGEngine Found"); }
    this->_data = _engine->Difference(this->_data, rhs._data);
    return *this;
}

SimpleTriMesh SimpleTriMesh::operator&(const SimpleTriMesh& rhs) const {
    if(_engine.isNull()) { RW_THROW("NO CSGEngine Found"); }
    return SimpleTriMesh(_engine->Intersection(this->_data, rhs._data));
}

SimpleTriMesh& SimpleTriMesh::operator&=(const SimpleTriMesh& rhs) {
    if(_engine.isNull()) { RW_THROW("NO CSGEngine Found"); }
    this->_data = _engine->Intersection(this->_data, rhs._data);
    return *this;
}

SimpleTriMesh SimpleTriMesh::operator^(const SimpleTriMesh& rhs) const {
    if(_engine.isNull()) { RW_THROW("NO CSGEngine Found"); }
    return SimpleTriMesh(_engine->SymmetricDifference(this->_data, rhs._data));
}

SimpleTriMesh& SimpleTriMesh::operator^=(const SimpleTriMesh& rhs) {
    if(_engine.isNull()) { RW_THROW("NO CSGEngine Found"); }
    this->_data = _engine->SymmetricDifference(this->_data, rhs._data);
    return *this;
}

SimpleTriMesh& SimpleTriMesh::operator=(TriMeshData::Ptr data) {
    _data = data;
    if(!_data.isNull() && _data.isShared()) { return *this; }

    _data = rw::core::ownedPtr(new TriMeshData());
    if(!data.isNull()) {
        _data->_triangles = data->_triangles;
        _data->_vertecies = data->_vertecies;
    }
    return *this;
}

SimpleTriMesh& SimpleTriMesh::operator=(const SimpleTriMesh& copy) {
    _engine = copy._engine;
    _data   = rw::core::ownedPtr(new TriMeshData(copy._data));
    return *this;
}

SimpleTriMesh& SimpleTriMesh::operator=(const SimpleTriMesh&& tmp) {
    _engine = tmp._engine;
    _data   = tmp._data;
    return *this;
}

SimpleTriMesh& SimpleTriMesh::operator=(const rw::core::Ptr<SimpleTriMesh>& copy) {
    _engine      = copy->_engine;
    _data        = rw::core::ownedPtr(new TriMeshData(copy->_data));
    return *this = SimpleTriMesh(copy);
}

SimpleTriMesh& SimpleTriMesh::operator=(const rw::geometry::TriMesh& copy) {
    return *this = SimpleTriMesh(copy);
}

SimpleTriMesh& SimpleTriMesh::operator=(rw::geometry::GeometryData& copy) {
    return *this = SimpleTriMesh(copy);
}

SimpleTriMesh& SimpleTriMesh::operator=(rw::geometry::GeometryData&& copy) {
    return *this = SimpleTriMesh(copy);
}

SimpleTriMesh& SimpleTriMesh::operator=(const rw::core::Ptr<rw::geometry::GeometryData>& copy) {
    return *this = SimpleTriMesh(copy);
}

SimpleTriMesh& SimpleTriMesh::operator=(const rw::core::Ptr<rw::geometry::TriMesh>& copy) {
    return *this = SimpleTriMesh(copy);
}
