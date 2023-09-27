#include <RobWorkConfig.hpp>
#include <rw/core/StringUtil.hpp>
#include <rw/geometry/IndexedTriangle.hpp>
#include <rw/geometry/Model3D.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/loaders/model3d/LoaderSTEP.hpp>
#include <rw/loaders/model3d/STLFile.hpp>

#include <boost/filesystem.hpp>

using namespace rw::core;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::geometry;

#if RW_HAVE_OCC

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <IGESControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <StlAPI_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

namespace {
TopoDS_Shape loadShape(std::string const& filename) {
    TopoDS_Shape shape;
    const std::string& filetype = StringUtil::toUpper(StringUtil::getFileExtension(filename));

    if(filetype == ".STP" || filetype == ".STEP") {
        STEPControl_Reader Reader;
        Reader.ReadFile(filename.c_str());
        Reader.NbRootsForTransfer();
        Reader.TransferRoots();
        shape = Reader.OneShape();
        BRepMesh_IncrementalMesh Mesh(shape, 0.01);
        Mesh.Perform();
    }
    else if(filetype == ".IGS") {
        IGESControl_Reader Reader;
        Reader.ReadFile(filename.c_str());
        Reader.NbRootsForTransfer();
        Reader.TransferRoots();
        shape = Reader.OneShape();
        BRepMesh_IncrementalMesh Mesh(shape, 0.01);
        Mesh.Perform();
    }
    else if(filetype == ".BREP") {
        BRep_Builder b;
        BRepTools::Read(shape, filename.c_str(), b);
    }
    else { RW_THROW("Unsupported file"); }

    return shape;
}
}    // namespace

rw::graphics::Model3D::Ptr LoaderSTEP::load(const std::string& filename) {
    TopoDS_Shape shape = loadShape(filename);
    return toModel(shape, boost::filesystem::path(filename).filename().c_str());
}

rw::graphics::Model3D::Ptr LoaderSTEP::toModel(const TopoDS_Shape& shape, std::string name) {
    PlainTriMesh<Triangle<double>> rwMesh;
    for(TopExp_Explorer aExpFace(shape, TopAbs_FACE); aExpFace.More(); aExpFace.Next()) {
        TopoDS_Face aFace                  = TopoDS::Face(aExpFace.Current());
        TopAbs_Orientation faceOrientation = aFace.Orientation();

        TopLoc_Location aLocation;
        Handle(Poly_Triangulation) aTr = BRep_Tool::Triangulation(aFace, aLocation);

        if(!aTr.IsNull()) {
            const TColgp_Array1OfPnt& aNodes       = aTr->Nodes();
            const Poly_Array1OfTriangle& triangles = aTr->Triangles();
            // const TColgp_Array1OfPnt2d& uvNodes    = aTr->UVNodes ();

            TColgp_Array1OfPnt aPoints(1, aNodes.Length());
            for(Standard_Integer i = 1; i < aNodes.Length() + 1; i++)
                aPoints(i) = aNodes(i).Transformed(aLocation);

            Standard_Integer nnn = aTr->NbTriangles();
            Standard_Integer nt, n1, n2, n3;

            for(nt = 1; nt < nnn + 1; nt++) {
                triangles(nt).Get(n1, n2, n3);
                gp_Pnt aPnt1 = aPoints(n1);
                gp_Pnt aPnt2 = aPoints(n2);
                gp_Pnt aPnt3 = aPoints(n3);

                /*gp_Pnt2d uv1 = uvNodes (n1);
                gp_Pnt2d uv2 = uvNodes (n2);
                gp_Pnt2d uv3 = uvNodes (n3);*/

                Vector3D<double> p1, p2, p3;
                if(faceOrientation == TopAbs_Orientation::TopAbs_FORWARD) {
                    p1 = Vector3D<double>(aPnt1.X(), aPnt1.Y(), aPnt1.Z());
                    p2 = Vector3D<double>(aPnt2.X(), aPnt2.Y(), aPnt2.Z());
                    p3 = Vector3D<double>(aPnt3.X(), aPnt3.Y(), aPnt3.Z());
                }
                else {
                    p1 = Vector3D<double>(aPnt3.X(), aPnt3.Y(), aPnt3.Z());
                    p2 = Vector3D<double>(aPnt2.X(), aPnt2.Y(), aPnt2.Z());
                    p3 = Vector3D<double>(aPnt1.X(), aPnt1.Y(), aPnt1.Z());
                }
                rwMesh.add(Triangle<double>(p1, p2, p3));
            }
        }
    }
    rwMesh.scale(0.001);
    Model3D::Ptr model =
        ownedPtr(new Model3D(name));
    model->addTriMesh(Model3D::Material("White", 0.8, 0.8, 0.8), rwMesh);

    return model;
}

#else
rw::graphics::Model3D::Ptr LoaderSTEP::load(const std::string& filename) {
    RW_THROW("RobWork is not compilled with OpenCascade Support");
    return NULL;
}
#endif