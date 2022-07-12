#include <rw/core/Ptr.hpp>
#include <rw/geometry/BSphere.hpp>
#include <rw/geometry/SimpleTriMesh.hpp>

#include <thread>
#include <vector>
using namespace rw::math;
using namespace rw::geometry;
using namespace rw::core;

ReferencedVertice::ReferencedVertice (TriMeshData::Ptr ref, uint32_t vertice) :
    _mesh (ref), _verIndex (vertice)
{
    RW_ASSERT (_mesh != NULL);
}

double& ReferencedVertice::operator[] (size_t i) const
{
    return _mesh->_vertecies (_verIndex, i);
}
ReferencedVertice& ReferencedVertice::operator= (rw::math::Vector3D< double > rhs)
{
    _mesh->_vertecies (_verIndex, 0) = rhs[0];
    _mesh->_vertecies (_verIndex, 1) = rhs[1];
    _mesh->_vertecies (_verIndex, 2) = rhs[2];
    return *this;
}
ReferencedVertice& ReferencedVertice::operator= (rw::math::Vector3D< float > rhs)
{
    _mesh->_vertecies (_verIndex, 0) = rhs[0];
    _mesh->_vertecies (_verIndex, 1) = rhs[1];
    _mesh->_vertecies (_verIndex, 2) = rhs[2];
    return *this;
}

ReferencedVertice& ReferencedVertice::operator*= (rw::math::Transform3D< double > rhs)
{
    Vector3D< double > v = *this;
    return *this         = (rhs * v);
}

ReferencedVertice::operator rw::math::Vector3D< double > () const
{
    return rw::math::Vector3D< double > (_mesh->_vertecies (_verIndex, 0),
                                         _mesh->_vertecies (_verIndex, 1),
                                         _mesh->_vertecies (_verIndex, 2));
}
ReferencedVertice::operator rw::math::Vector3D< float > () const
{
    return rw::math::Vector3D< float > (_mesh->_vertecies (_verIndex, 0),
                                        _mesh->_vertecies (_verIndex, 1),
                                        _mesh->_vertecies (_verIndex, 2));
}

ReferencedTriangle::ReferencedTriangle (TriMeshData::Ptr ref, uint32_t triangle) :
    _mesh (ref), _triIndex (triangle)
{
    RW_ASSERT (_mesh != NULL);
}

ReferencedVertice ReferencedTriangle::operator[] (size_t i) const
{
    return ReferencedVertice (_mesh, _mesh->_triangles (_triIndex, i));
}

uint32_t& ReferencedTriangle::idx (size_t i) const
{
    return _mesh->_triangles (_triIndex, i);
}

ReferencedTriangle& ReferencedTriangle::operator= (rw::geometry::Triangle< double > rhs)
{
    (*this)[0] = rhs[0];
    (*this)[1] = rhs[1];
    (*this)[2] = rhs[2];
    return *this;
}

ReferencedTriangle& ReferencedTriangle::operator= (rw::geometry::Triangle< float > rhs)
{
    (*this)[0] = rhs[0];
    (*this)[1] = rhs[1];
    (*this)[2] = rhs[2];
    return *this;
}

ReferencedTriangle::operator rw::geometry::Triangle< double > () const
{
    return rw::geometry::Triangle< double > ((*this)[0], (*this)[1], (*this)[2]);
}
ReferencedTriangle::operator rw::geometry::Triangle< float > () const
{
    return rw::geometry::Triangle< float > ((*this)[0], (*this)[1], (*this)[2]);
}

namespace rw { namespace geometry {
    std::ostream& operator<< (std::ostream& os, const ReferencedVertice& v)
    {
        os << "{" << v[0] << ", " << v[1] << ", " << v[2] << "}";
        return os;
    }
    std::ostream& operator<< (std::ostream& os, const ReferencedTriangle& t)
    {
        {
            os << "{"
               << "[" << t.idx (0) << "] " << t[0] << ", "
               << "[" << t.idx (1) << "] " << t[1] << ", "
               << "[" << t.idx (2) << "] " << t[2] << "}";
            return os;
        }
    }
}}    // namespace rw::geometry

SimpleTriMesh::SimpleTriMesh (TriMeshData::Ptr data) :
    _engine (CSGEngine::Factory::getDefaultEngine ())
{
    if (data.isNull ()) {
        _data = rw::core::ownedPtr (new TriMeshData ());
    }
    else {
        _data = data;
    }
}

SimpleTriMesh::~SimpleTriMesh ()
{
    if (!_data.isShared ()) {
        delete _data.get ();
    }
}

SimpleTriMesh::SimpleTriMesh (const SimpleTriMesh& copy) :
    _data (rw::core::ownedPtr (new TriMeshData ())),
    _engine (CSGEngine::Factory::getDefaultEngine ())
{
    _data->_triangles = copy._data->_triangles;
    _data->_vertecies = copy._data->_vertecies;
}

SimpleTriMesh::SimpleTriMesh (const SimpleTriMesh&& copy) :
    _data (copy._data), _engine (copy._engine)
{}

SimpleTriMesh::SimpleTriMesh (const rw::core::Ptr< SimpleTriMesh >& copy) :
    _data (rw::core::ownedPtr (new TriMeshData ())),
    _engine (CSGEngine::Factory::getDefaultEngine ())
{
    _data->_triangles = copy->_data->_triangles;
    _data->_vertecies = copy->_data->_vertecies;
}
SimpleTriMesh::SimpleTriMesh (const rw::geometry::TriMesh& copy) :
    _data (rw::core::ownedPtr (new TriMeshData ())),
    _engine (CSGEngine::Factory::getDefaultEngine ())
{
    fromTriMesh (copy);
}

SimpleTriMesh::SimpleTriMesh (rw::geometry::GeometryData& copy) : SimpleTriMesh (copy.getTriMesh ())
{}

SimpleTriMesh::SimpleTriMesh (rw::geometry::GeometryData&& copy) :
    SimpleTriMesh (copy.getTriMesh ())
{}

SimpleTriMesh::SimpleTriMesh (const rw::core::Ptr< rw::geometry::GeometryData >& copy) :
    SimpleTriMesh (copy->getTriMesh ())
{}

SimpleTriMesh::SimpleTriMesh (const rw::core::Ptr< rw::geometry::TriMesh >& copy) :
    SimpleTriMesh (*copy)
{}

rw::geometry::Triangle< double > SimpleTriMesh::getTriangle (size_t idx) const
{
    return this->triangle (idx);
}

void SimpleTriMesh::getTriangle (size_t idx, rw::geometry::Triangle< double >& dst) const
{
    dst = this->triangle (idx);
}

void SimpleTriMesh::getTriangle (size_t idx, rw::geometry::Triangle< float >& dst) const
{
    dst = this->triangle (idx);
}

size_t SimpleTriMesh::size () const
{
    return this->_data->_triangles.rows ();
}

rw::core::Ptr< TriMesh > SimpleTriMesh::clone () const
{
    return rw::core::ownedPtr (new SimpleTriMesh (*this));
}

rw::core::Ptr< TriMesh > SimpleTriMesh::getTriMesh (bool forceCopy)
{
    if (forceCopy || !this->_data.isShared ()) {
        return clone ();
    }
    else {
        return rw::core::ownedPtr (new SimpleTriMesh (this->_data));
    }
}

void SimpleTriMesh::scale (double scale)
{
    for (Eigen::Index i = 0; i < _data->_vertecies.rows (); i++) {
        for (Eigen::Index j = 0; j < 3; j++) {
            _data->_vertecies (i, j) *= scale;
        }
    }
}
void SimpleTriMesh::scale (const rw::math::Vector3D< double >& scale)
{
    for (Eigen::Index i = 0; i < _data->_vertecies.rows (); i++) {
        for (Eigen::Index j = 0; j < 3; j++) {
            _data->_vertecies (i, j) *= scale[j];
        }
    }
}

ReferencedTriangle SimpleTriMesh::triangle (size_t idx) const
{
    return ReferencedTriangle (_data, idx);
}

size_t SimpleTriMesh::triangles () const
{
    return _data->_triangles.rows ();
}

ReferencedVertice SimpleTriMesh::vertice (size_t idx) const
{
    return ReferencedVertice (_data, idx);
}

size_t SimpleTriMesh::vertices () const
{
    return _data->_vertecies.rows ();
}

void SimpleTriMesh::resize (size_t triangles, size_t vertices)
{
    _data->_triangles.resize (triangles, 3);
    _data->_vertecies.resize (vertices, 3);
}

namespace {
size_t indexOf (std::vector< Vector3D< double > >& vec, Vector3D< double > elem)
{
    std::vector< Vector3D< double > >::iterator it;
    for (size_t i = 0; i < vec.size (); i++) {
        if (vec[i] == elem) {
            return i;
        }

        /*if ((vec[i] - elem).norm2 () < prox) {
            return i;
        }*/
    }

    vec.push_back (elem);
    return vec.size () - 1;
}
}    // namespace

void SimpleTriMesh::fromTriMesh (const rw::geometry::TriMesh& copy)
{
    std::vector< Vector3D< double > > vertices;
    std::vector< Vector3D< uint32_t > > triangles;
    for (size_t i = 0; i < copy.size (); i++) {
        Vector3D< uint32_t > tri;
        Triangle< double > old_tri = copy.getTriangle (i);
        for (size_t j = 0; j < 3; j++) {
            tri[j] = indexOf (vertices, old_tri[j]);
        }
        triangles.push_back (tri);
    }

    this->resize (triangles.size (), vertices.size ());

    for (size_t i = 0; i < vertices.size (); i++) {
        for (size_t j = 0; j < 3; j++) {
            _data->_vertecies (i, j) = vertices[i][j];
        }
    }
    for (size_t i = 0; i < triangles.size (); i++) {
        for (size_t j = 0; j < 3; j++) {
            _data->_triangles (i, j) = triangles[i][j];
        }
    }
}

SimpleTriMesh& SimpleTriMesh::operator*= (const rw::math::Transform3D< double >& trans)
{
    for (size_t i = 0; i < vertices (); i++) {
        vertice (i) *= trans;
    }
    return *this;
}

SimpleTriMesh SimpleTriMesh::operator* (const rw::math::Transform3D< double >& trans) const
{
    SimpleTriMesh t (*this);
    for (size_t i = 0; i < t.vertices (); i++) {
        t.vertice (i) *= trans;
    }
    return t;
}

SimpleTriMesh SimpleTriMesh::operator+ (const SimpleTriMesh& rhs) const
{
    if (_engine.isNull ()) {
        RW_THROW ("NO CSGEngine Found");
    }
    return SimpleTriMesh (_engine->Union (this->_data, rhs._data));
}

SimpleTriMesh& SimpleTriMesh::operator+= (const SimpleTriMesh& rhs)
{
    if (_engine.isNull ()) {
        RW_THROW ("NO CSGEngine Found");
    }
    this->_data = _engine->Union (this->_data, rhs._data);
    return *this;
}

SimpleTriMesh SimpleTriMesh::operator- (const SimpleTriMesh& rhs) const
{
    if (_engine.isNull ()) {
        RW_THROW ("NO CSGEngine Found");
    }
    return SimpleTriMesh (_engine->Difference (this->_data, rhs._data));
}

SimpleTriMesh& SimpleTriMesh::operator-= (const SimpleTriMesh& rhs)
{
    if (_engine.isNull ()) {
        RW_THROW ("NO CSGEngine Found");
    }
    this->_data = _engine->Difference (this->_data, rhs._data);
    return *this;
}

SimpleTriMesh SimpleTriMesh::operator& (const SimpleTriMesh& rhs) const
{
    if (_engine.isNull ()) {
        RW_THROW ("NO CSGEngine Found");
    }
    return SimpleTriMesh (_engine->Intersection (this->_data, rhs._data));
}

SimpleTriMesh& SimpleTriMesh::operator&= (const SimpleTriMesh& rhs)
{
    if (_engine.isNull ()) {
        RW_THROW ("NO CSGEngine Found");
    }
    this->_data = _engine->Intersection (this->_data, rhs._data);
    return *this;
}

SimpleTriMesh SimpleTriMesh::operator^ (const SimpleTriMesh& rhs) const
{
    if (_engine.isNull ()) {
        RW_THROW ("NO CSGEngine Found");
    }
    return SimpleTriMesh (_engine->SymmetricDifference (this->_data, rhs._data));
}

SimpleTriMesh& SimpleTriMesh::operator^= (const SimpleTriMesh& rhs)
{
    if (_engine.isNull ()) {
        RW_THROW ("NO CSGEngine Found");
    }
    this->_data = _engine->SymmetricDifference (this->_data, rhs._data);
    return *this;
}