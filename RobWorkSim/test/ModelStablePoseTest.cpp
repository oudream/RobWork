/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#include <rw/geometry/Face.hpp>
#include <rw/geometry/FaceArrayFactory.hpp>
#include <vector>

#include <sandbox/geometry/STLFile.hpp>
#include <sandbox/geometry/Triangle.hpp>
#include <sandbox/geometry/PlainTriMesh.hpp>
#include <sandbox/geometry/TriangleUtil.hpp>
#include <sandbox/geometry/GeometryFactory.hpp>
#include <sandbox/geometry/IndexedTriMesh.hpp>

#include <dynamics/ContactPoint.hpp>
#include <dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <sandbox/geometry/GeometryUtil.hpp>

#include <dynamics/ContactManifold.hpp>
#include <sandbox/geometry/GeometryFactory.hpp>

#include <geometry/GiftWrapHull3D.hpp>

using namespace rw::math;
using namespace boost::numeric;
using namespace rw::math;
using namespace rw::geometry;
using namespace rw::geometry::sandbox;


using namespace boost::numeric::ublas;

int main(int argc, char** argv)
{
	if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of stl, ac3d or 3ds file" << std::endl;
	    std::cout << "- Arg 2 mass of model in kg\n" << std::endl;
	    return 0;
	}
	std::string filename(argv[1]);
	double mass = 1.0;
	if(argc>2)
	    mass = std::atof(argv[2]);

	Geometry *geo = GeometryFactory::getGeometry(filename);
	std::vector<Geometry*> geoms;
	geoms.push_back(geo);

	Vector3D<> masscenter = GeometryUtil::estimateCOG(geoms);
	InertiaMatrix<> inertia = GeometryUtil::estimateInertia(mass, geoms);

	typedef std::pair<matrix<double>, vector<double> > Result;
	Result res = LinearAlgebra::eigenDecompositionSymmetric( inertia.m() );

	std::cout << "------- Model properties ----- \n"
			  << "- COG     : " << masscenter << "\n"
			  << "- Inertia : " << inertia << "\n";

	std::cout << "- inertia on pricipal from: \n"
			  << "- inertia          : " << res.second << "\n"
			  << "- Body orientation  : " << res.first << "\n";

	std::cout << "- Calculating invariant axes: \n";

	// now create the convex hull of the geometry
	TriMesh *mesh = GeometryUtil::toTriMesh(geo);
	//IndexedTriMesh<float> *idxMesh = dynamic_cast<IndexedTriMesh<float>* >(mesh);
	IndexedTriMesh<> *idxMesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<> >(*mesh,0.00001);
	RW_ASSERT(idxMesh);

	GiftWrapHull3D hull;
    hull.rebuild( idxMesh->getVertices() );
    PlainTriMesh<TriangleN1<> > *fmesh = hull.toTriMesh();
    // now project the center of mass onto all triangles in the trimesh
    // If it is inside a triangle then the triangle is a stable pose
    std::vector<TriangleN1<> > result;
    for(size_t i=0;i<fmesh->size();i++){
        if( (*fmesh)[i].isInside(masscenter) )
            result.push_back((*fmesh)[i]);
    }

    std::vector<TriangleN1<> > result2;
    BOOST_FOREACH(TriangleN1<>& tri, result){
        Vector3D<> n = tri.getFaceNormal();
        bool hasNormal = false;
        BOOST_FOREACH(TriangleN1<> &t, result2){
            if( MetricUtil::dist2(n, t.getFaceNormal())<0.1 ){
                hasNormal=true;
                break;
            }
        }
        if(!hasNormal)
            result2.push_back(tri);
    }

    BOOST_FOREACH(TriangleN1<>& tri, result2){
        std::cout << "- " << -tri.getFaceNormal() << std::endl;
    }
    std::cout << "------- Model properties END ----- \n";
    //std::cout << "write to file..." << std::endl;
    //STLFile::writeSTL(*fmesh, "resultingHull.stl");
    //STLFile::writeSTL(*idxMesh, "testSTL.stl");

	delete geo;

	return 0;
}
