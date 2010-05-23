
#include "Loader3DS.hpp"
#include <rwlibs/os/rwgl.hpp>
#include <cmath>                       // Header file for the math library
#include <iostream>

#include "Model3DS.hpp"
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
using namespace rw::common;
using namespace rw::math;
using namespace rw::geometry;
using namespace rwlibs::drawable;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

namespace {
	Vector3D<float> toVector3D(float* vals){
		return Vector3D<float>(vals[0],vals[1],vals[2]);
	}

	Vector2D<float> toVector2D(float* vals){
		return Vector2D<float>(vals[0],vals[1]);
	}

}

Model3DPtr Loader3DS::load(const std::string& name)
{
	Model3DS m3ds;
	setlocale(LC_ALL, "C");
	m3ds.Load(name);
	setlocale(LC_ALL, "");

	Model3DPtr model( new Model3D() );

	// TODO: convert 3ds model into rw model

	std::vector<RWGLTexture*> &textures = model->_textures;

	model->_materials.resize(m3ds.Materials.size());
	std::vector<Model3D::Material> &materials = model->_materials;
	// first we copy all materials
	for(size_t i=0; i<materials.size(); i++){
		const Model3DS::Material& mat_src = m3ds.Materials[i];
		Model3D::Material& mat_dst = materials[i];
		// copy name
		mat_dst.name = std::string(mat_src.name);
		// copy color
		for(size_t j=0; j<4; j++)
			mat_dst.rgb[j] = ((unsigned char*)&mat_src.color.r)[j]/256.0f;

		if( mat_src.textured ){
			mat_dst.textured = true;
			textures.push_back(new RWGLTexture(mat_src.tex));
			mat_dst.texId = textures.size()-1;
		}
	}

	// next we copy all objects of the 3ds model
	model->_objects.resize(m3ds.Objects.size());
	std::vector<Model3D::Object3D*> &objects = model->_objects;
	for(size_t i=0; i<objects.size(); i++){
		Model3DS::Object& obj_src = m3ds.Objects[i];
		objects[i] = new Model3D::Object3D(obj_src.name);
		Model3D::Object3D& obj_dst = *objects[i];
		// copy transformation
		RPY<float> rpyX(0,0,obj_src.rot.x*Deg2Rad);
		RPY<float> rpyY(0,obj_src.rot.y*Deg2Rad,0);
		RPY<float> rpyZ(obj_src.rot.z*Deg2Rad,0,0);
		// TODO: perhaps the order that they multiply is wrong
		Rotation3D<float> rot = rpyZ.toRotation3D()*rpyY.toRotation3D()*rpyX.toRotation3D();
		Vector3D<float> pos(obj_src.pos.x,obj_src.pos.y,obj_src.pos.z);
		obj_dst._transform = Transform3D<float>(pos, rot);

		// copy vertices
		obj_dst._vertices.resize(obj_src.Vertexes.size()/3);
		for(size_t j=0; j<obj_dst._vertices.size();j++)
			obj_dst._vertices[j] = toVector3D(&(obj_src.Vertexes[j*3]));

		// copy normals
		obj_dst._normals.resize(obj_src.Normals.size()/3);
		for(size_t j=0; j<obj_dst._normals.size();j++)
			obj_dst._normals[j] = toVector3D(&(obj_src.Normals[j*3]));

		// copy faces
		obj_dst._faces.resize(obj_src.Faces.size()/3);
		// now comes the copying of faces
		for(size_t j=0; j<obj_dst._faces.size(); j++){
			IndexedTriangleN0<>& face = obj_dst._faces[j];
			size_t j3tmp = j*3;
			face[0] = obj_src.Faces[j3tmp+0];
			face[1] = obj_src.Faces[j3tmp+1];
			face[2] = obj_src.Faces[j3tmp+2];
		}

		// and the material faces
		obj_dst._matFaces.resize(obj_src.MatFaces.size());
		for(size_t j=0; j<obj_dst._matFaces.size();j++){
			const Model3DS::MaterialFaces &mat = obj_src.MatFaces[j];
			Model3D::MaterialFaces *faces = new Model3D::MaterialFaces();
			obj_dst._matFaces[j] = faces;

			faces->_matIndex = mat.MatIndex;

			// copy faces
			faces->_subFaces.resize(mat.subFaces.size()/3);
			// now comes the copying of faces
			for(size_t jj=0; jj<faces->_subFaces.size(); jj++){
				IndexedTriangleN0<>& face = faces->_subFaces.at(jj);
				size_t jj3tmp = jj*3;
				face[0] = mat.subFaces[jj3tmp+0];
				face[1] = mat.subFaces[jj3tmp+1];
				face[2] = mat.subFaces[jj3tmp+2];
			}

		}

		// and lastly we add the texture coordinates
		obj_dst._texCoords.resize(obj_src.TexCoords.size()/2);
		for(size_t j=0; j<obj_dst._texCoords.size();j++)
			obj_dst._texCoords[j] = toVector2D(&(obj_src.TexCoords[j*2]));
/*
		std::cout << "Model stats : \n";
		std::cout << "nr vertices : " << obj_dst._vertices.size() << std::endl;
		std::cout << "nr faces    : " << obj_dst._faces.size() << std::endl;
		std::cout << "nr mat faces: " << obj_dst._matFaces.size() << std::endl;
		std::cout << "nr mat faces: " << obj_dst._matFaces[0]->_subFaces.size() << std::endl;
		std::cout << "nr normals  : " << obj_dst._normals.size() << std::endl;
		std::cout << "nr transform: " << obj_dst._transform << std::endl;
		std::cout << "size_of(Vector3D): " << sizeof(Vector3D<float>) << std::endl;
		std::cout << "size_of(Ublas vector): " << sizeof(boost::numeric::ublas::bounded_vector<float, 3>) << std::endl;
		std::cout << "size_of(IndexedTriangle): " << sizeof(IndexedTriangleN0<>) << std::endl;
		std::cout << "size_of(IndexedTriangle): " << sizeof(IndexedTriangleN0<unsigned short>) << std::endl;
		std::cout << "size_of(uint16_t): " << sizeof(uint16_t) << std::endl;
		std::cout << "size_of(uint8_t): " << sizeof(uint8_t) << std::endl;
*/
	}
	return model;
}
