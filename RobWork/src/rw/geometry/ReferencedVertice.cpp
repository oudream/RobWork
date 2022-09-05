#include <rw/geometry/ReferencedVertice.hpp>


using namespace rw::geometry;
using namespace rw::math;

ReferencedVertice::ReferencedVertice (TriMeshData::Ptr ref, uint32_t vertice) :
    _mesh (ref), _verIndex (vertice)
{
    RW_ASSERT (_mesh != NULL);
}

ReferencedVertice::ReferencedVertice () : _mesh (NULL), _verIndex (0)
{}

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
namespace rw { namespace geometry {
    std::ostream& operator<< (std::ostream& os, const ReferencedVertice& v)
    {
        os << "{" << v[0] << ", " << v[1] << ", " << v[2] << "}";
        return os;
    }
}}    // namespace rw::geometry
