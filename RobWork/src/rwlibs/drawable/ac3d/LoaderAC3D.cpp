/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "LoaderAC3D.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/common/macros.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/math/Constants.hpp>
#include <rw/loaders/ImageFactory.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stack>

#include <fstream>

#include <boost/foreach.hpp>



#define AC3D_OBJECT_WORLD 999
#define AC3D_OBJECT_NORMAL 0
#define AC3D_OBJECT_GROUP 1
#define AC3D_OBJECT_LIGHT 2

#define AC3D_SURFACE_SHADED (1<<4)
#define AC3D_SURFACE_TWOSIDED (1<<5)

#define AC3D_SURFACE_TYPE_POLYGON (0)
#define AC3D_SURFACE_TYPE_CLOSEDLINE (1)
#define AC3D_SURFACE_TYPE_LINE (2)

#define streq(a,b)  ( (*(a)==*(b)) && !strcmp(a,b) )

using namespace rwlibs::drawable;
using namespace rw::math;
using namespace rw::common;
using namespace rw::geometry;

LoaderAC3D::LoaderAC3D():
    _maxAngle(45*Deg2Rad)
{
}

LoaderAC3D::~LoaderAC3D()
{
}

Model3DPtr LoaderAC3D::load(const std::string& filename){

    //    const int BUFF_SIZE = 4096*16;
    //    char mybuffer[BUFF_SIZE];
    std::ifstream in(filename.c_str());
    if (!in.is_open())
        RW_THROW("Can't open file " << StringUtil::quote(filename));
    //    in.rdbuf()->pubsetbuf(mybuffer,BUFF_SIZE);



    std::string line;
    in >> line;

    if (line != "AC3Db") {
        RW_THROW("Data stream does not contain a valid AC3D file.");
    }

    setlocale(LC_ALL, "C");

    ModelAC3D *model = new ModelAC3D();
    model->_currentDir = StringUtil::getDirectoryName(filename);

    AC3DObject *object = load_object(in, NULL, model);

    // now convert object to a Model3D
    Model3D *rwmodel = new Model3D();

    // first we add all materials
    std::vector<Model3D::Material>& materials = rwmodel->getMaterials();
    materials.resize( model->_materials.size() );
    for(size_t i=0;i<materials.size();i++){
        Model3D::Material &m = materials[i];
        AC3DMaterial &ac3m = model->_materials[i];
        m.name = ac3m.name;
        // texture is in the AC3D an object property. for now we set it to null
        m.texId = 0;
        m.textured = false;

        //
        m.simplergb = false;
        std::copy(ac3m.rgb,ac3m.rgb+4,m.rgb);
        std::copy(ac3m.ambient,ac3m.ambient+4,m.ambient);
        std::copy(ac3m.emissive,ac3m.emissive+4,m.emissive);
        std::copy(ac3m.specular,ac3m.specular+4,m.specular);
        m.shininess = ac3m.shininess;
        m.transparency = ac3m.transparency;
    }


    // next we
    std::vector<Model3D::Object3D*> &objects = rwmodel->getObjects();

    std::stack<std::pair<AC3DObject*, Model3D::Object3D*> > mobjects;
    mobjects.push( std::make_pair(object,(Model3D::Object3D*)NULL) );
    while(!mobjects.empty()){
        AC3DObject* obj  = mobjects.top().first;
        Model3D::Object3D* parent  = mobjects.top().second;
        mobjects.pop();

        Model3D::Object3D *rwobj = new Model3D::Object3D(obj->name);
        rwobj->_transform = Transform3D<float>(obj->loc,obj->rot);

        rwobj->_texOffset(0) = obj->texture_offset_x;
        rwobj->_texOffset(1) = obj->texture_offset_y;
        rwobj->_texRepeat(0) = obj->texture_repeat_x;
        rwobj->_texRepeat(1) = obj->texture_repeat_y;
        rwobj->_texture = obj->texture;

        // todo: copy vertex data
        // for now we only support triangle meshes
        size_t nrFaces = obj->surfaces.size();

        rwobj->_faces.resize(nrFaces);
        rwobj->_vertices.resize( obj->vertices.size() );
        //rwobj->_normals.resize( obj->vertices.size() ); // we use one normal per face vertice

        if(obj->texture!=-1)
            rwobj->_texCoords.resize( obj->vertices.size() );

        for(size_t i=0; i<obj->vertices.size();i++)
            rwobj->_vertices[i] = obj->vertices[i].toV3D();

        for(size_t i=0; i<nrFaces;i++){
            AC3DSurface &s = obj->surfaces[i];
            if(s.vertrefs.size()!=3)
                RW_THROW("Only trimesh is currently supported! nr verts: " << s.vertrefs.size());
            // copy the vertice references
            // copy the normals
            if(s.normals.size()!=3){
                rwobj->_normals.push_back(s.normal.toV3D());
                rwobj->_faces[i] = IndexedTriangleN1<float>(s.vertrefs[0],s.vertrefs[1],s.vertrefs[2],rwobj->_normals.size()-1);
            } else {
                rwobj->_normals.push_back( s.normals[0].toV3D() );
                rwobj->_normals.push_back( s.normals[1].toV3D() );
                rwobj->_normals.push_back( s.normals[2].toV3D() );
                size_t nidx = rwobj->_normals.size();
                rwobj->_faces3.push_back( IndexedTriangleN3<float>(s.vertrefs[0],s.vertrefs[1],s.vertrefs[2],
                                            nidx-4,nidx-3,nidx-2 ) );
            }
            // copy texture coords if enabled
            if(obj->texture!=-1){
                if(s.uvs.size()!=3)
                    RW_THROW("Not enough texture coordinates. uvs.size: " << s.uvs.size());
                for(int j = 0;j<3;j++){
                    rwobj->_texCoords[s.vertrefs[j]](0) = s.uvs[j](0);
                    rwobj->_texCoords[s.vertrefs[j]](1) = s.uvs[j](1);
                }
            }
        }

        if(parent!=NULL)
            parent->_kids.push_back(rwobj);
        else
            objects.push_back(rwobj);
        // add all kids here

        for(size_t i=0;i<obj->kids.size(); i++){
            mobjects.push( std::make_pair(obj->kids[i],rwobj) );
        }
    }
    setlocale(LC_ALL, "");
    return ownedPtr(rwmodel);
}

namespace {
    void printArray(std::ostream& out, float *arr, int len){
        for(int i=0;i<len;i++)
            out << " " << arr[i];
    }
}

//void LoaderAC3D::save(Model3DPtr model, const std::string& filename){

    /*
    std::ofstream out(filename.c_str());
    if (!out.is_open())
        RW_THROW("Can't open file " << StringUtil::quote(filename));
    //    in.rdbuf()->pubsetbuf(mybuffer,BUFF_SIZE);

    // write header
    out << "AC3Db\n";

    // write all materials
    BOOST_FOREACH(Model3D::Material &m, model->getMaterials() ){
        // MATERIAL "ac3dmat1" rgb 1 1 1  amb 0.2 0.2 0.2  emis 0 0 0  spec 0.2 0.2 0.2  shi 128  trans 0
        out << "MATERIAL " << StringUtil::quote(m.name);
        out << " rgb"; printArray(out, m.rgb, 3);
        out << " amb"; printArray(out, m.ambient, 3);
        out << " emis"; printArray(out, m.emissive, 3);
        out << " spec"; printArray(out, m.specular, 3);
        out << " shi " << m.shininess;
        out << " trans " << m.transparency << "\n";
    }

    // iterate through all objects
    BOOST_FOREACH(Model3D::Object3D *object, model->getObjects()){
        //
        std::stack<std::pair<Model3D::Object3D*,Model3D::Object3D*> > objstack;
        objstack.push(object);

        while( !objstack.empty() ){
            Model3D::Object3D *obj = objstack.top().first;
            Model3D::Object3D *parent = objstack.top().second;

            objstack.pop();

            std::string type = "poly";
            if(parent == NULL)
                type = "world";
            out << "OBJECT " << type << "\n";


            // last we write number of kids
            out << "kids " << obj->_kids.size();
            for(int i=obj->_kids.size()-1;i>=0;i--){
                objstack.push(obj->_kids[i]);
            }
        }
    }

*/
//}


LoaderAC3D::AC3DMaterial LoaderAC3D::read_material(std::istream& in)
{
    char buff[256];
    AC3DMaterial m;

    // Read the material name
    // the name is within quotes and contain whitespace
    //in >> token;
    in.getline(buff, 256, '"');
    in.getline(buff, 256, '"');
    std::string token(buff);

    // Remove quotes
    //token = token.erase(0, 1);

    //m.name = token.erase(token.length() - 1, 1);
    m.name = token;
    // Read the RGB color
    in >> token;
    if (token == "rgb") {
        in >> (m.rgb[0]);
        in >> (m.rgb[1]);
        in >> (m.rgb[2]);
    } else {
        RW_THROW(
            "Expected \"rgb\" token not found - " << token);
    }

    // Read the Ambient color
    in >> token;
    if (token == "amb") {
        in >> (m.ambient[0]);
        in >> (m.ambient[1]);
        in >> (m.ambient[2]);
    } else {
        RW_THROW("Expected \"amb\" token not found");
    }

    // Read the Emissive Color
    in >> token;
    if (token == "emis") {
        in >> (m.emissive[0]);
        in >> m.emissive[1];
        in >> m.emissive[2];
    } else {
        RW_THROW("Expected \"emis\" token not found");
    }

    // Read the Specular Color
    in >> token;
    if (token == "spec") {
        in >> m.specular[0];
        in >> m.specular[1];
        in >> m.specular[2];
    } else
        RW_THROW("Expected \"spec\" token not found");

    // Read the shininess
    in >> token;
    if (token == "shi")
        in >> m.shininess;
    else
        RW_THROW("Expected \"shi\" token not found");

    // Read the transparency
    in >> token;
    if (token == "trans")
        in >> m.transparency;
    else
        RW_THROW("Expected \"trans\" token not found");
    return m;
}

LoaderAC3D::OBJECT_TYPE LoaderAC3D::string_to_objecttype(const std::string& s)
{
    if (s == "world")
        return OBJECT_WORLD;
    if (s == "group")
        return OBJECT_GROUP;
    if (s == "light")
        return OBJECT_LIGHT;
    return OBJECT_NORMAL;
}

void LoaderAC3D::read_data(std::istream& in, std::vector<char>& out)
{
    int len;
    in >> len;

    // The data is on the next line, therefore we might have to add something to
    // read carry return and linefeed.
    out.resize(len);
    for (int i = 0; i < len; i++)
        in >> out[i];
}

void LoaderAC3D::read_surface(std::istream& in, AC3DSurface& surf, AC3DObject* ob)
{
    char buff[256];
    char token[60],value[60];
    //std::string token;
    while (!in.eof()) {
        in.getline(buff,256);

        if(in.fail()){
            RW_WARN("FAiled in read surface!");
            continue;
        }

        //in >> token;
        int res = sscanf(buff, "%s %s", token, value);
        if(res!=2)
            continue;

        //if (token == "SURF") {
        if ( streq(token, "SURF") ) {
            //in >> token;
            //sscanf(&buff[offset],"%s",token);
            //surf.flags = strtol(token.c_str(), NULL, 16);
            surf.flags = strtol(value, NULL, 16);
        }
        //else if (token == "mat") {
        else if ( streq(token, "mat") ) {
            sscanf(value,"%d",&surf.mat);
        }
        //else if (token == "refs") {
        else if( streq(token, "refs") ){
            //in >> surf.vertref_cnt;
            sscanf(value,"%d",&surf.vertref_cnt);
            surf.vertrefs.resize(surf.vertref_cnt);
            surf.normals.resize(surf.vertref_cnt);
            surf.uvs.resize(surf.vertref_cnt);

            for (int i = 0; i < surf.vertref_cnt; i++) {
                in.getline(buff,256);
                int ref;
                float uvs0,uvs1;
                sscanf(buff, "%d %f %f\n", &ref, &uvs0, &uvs1 );
                surf.vertrefs[i] = ref;
                surf.uvs[i](0) = uvs0;
                surf.uvs[i](1) = uvs1;
                //in >> surf.vertrefs[i] >> surf.uvs[i](0) >> surf.uvs[i](1);
                //in >> ;
                //in ;
            }
            if (surf.vertref_cnt >= 3) {
                const Vector3D<float>& v1 = ob->vertices[surf.vertrefs[0]].toV3D();
                const Vector3D<float>& v2 = ob->vertices[surf.vertrefs[1]].toV3D();
                const Vector3D<float>& v3 = ob->vertices[surf.vertrefs[2]].toV3D();
                surf.normal =
                    normalize(
                        cross(
                            Vector3D<float>(v2 - v1),
                            Vector3D<float>(v3 - v1)));
            }
            return;
        }
        else{
            std::cout << "Token: " << std::string(token) << " Value: " << std::string(value) << std::endl;
            RW_WARN("In AC3D file: Ignoring string ");
        }
    }
}

LoaderAC3D::AC3DObject* LoaderAC3D::load_object(
    std::istream& in, AC3DObject* parent, ModelAC3D* model)
{

    std::string token;
    AC3DObject* ob = NULL;
    while (!in.eof()) {

        in >> token;

        if( in.fail() ){
            RW_WARN("Fail bit set on stream!");
            TimerUtil::sleepMs(100);
            continue;
        }

        switch (token[0]){
        case('c'):
            if( token == "crease"){
                double tmp;
                in>>tmp;
            }
            break;
        case('d'):
            if (token == "data") {
                read_data(in, ob->data);
            }
        break;
        case('k'):
            if (token == "kids") {
                in >> ob->num_kids;
                if (ob->num_kids > 0) {
                    ob->kids.resize(ob->num_kids);
                    for (int i = 0; i < ob->num_kids; i++) {
                        AC3DObject* k = load_object(in, ob, model);
                        if (k != NULL) {
                            ob->kids[i] = k;
                        } else {
                            RW_THROW(
                                "Could not find the specified kid!");
                        }
                    }
                }
                calc_vertex_normals(ob);
                return ob;
            }
        break;
        case('l'):
            if (token == "loc") {
                in >> ob->loc(0);
                in >> ob->loc(1);
                in >> ob->loc(2);
            }
        break;
        case('M'):
            if (token == "MATERIAL") {
                model->_materials.push_back(read_material(in));
            }
        break;
        case('n'):
            if (token == "name") {
                in >> ob->name;
                ob->name = ob->name.substr(1, ob->name.length()-2);
            } else if (token == "numvert") {
                in >> ob->vertex_cnt;
                ob->vertices.resize(ob->vertex_cnt);
                // std::cout << "- Reading vertices: " << ob->surf_cnt << std::endl;
                // long time = TimerUtil::currentTimeMs();
                char buff[1024];
                in.getline(buff,256);
                for (int i = 0; i < ob->vertex_cnt; i++) {
                    in.getline(buff,1024);
                    float v0=10,v1=10,v2=10;
                    sscanf(buff, "%f %f %f", &v0, &v1, &v2 );
                    ob->vertices[i].val[0] = v0;
                    ob->vertices[i].val[1] = v1;
                    ob->vertices[i].val[2] = v2;

                    //in >> ob->vertices[i].val[0];
                    //in >> ob->vertices[i].val[1];
                    //in >> ob->vertices[i].val[2];
                }
                // long timel = TimerUtil::currentTimeMs();
                // std::cout << "- time: " << timel-time << std::endl;

            } else if (token == "numsurf") {
                in >> ob->surf_cnt;
                ob->surfaces.resize(ob->surf_cnt);
                // std::cout << "- Reading surfaces: " << ob->surf_cnt << std::endl;
                // long time = TimerUtil::currentTimeMs();
                for (int i = 0; i < ob->surf_cnt; i++) {
                    read_surface(in, ob->surfaces[i], ob);
                }
                // long timel = TimerUtil::currentTimeMs();
                // std::cout << "- time: " << timel-time << std::endl;
            }
        break;
        case('O'):
            if (token == "OBJECT") {
                model->nrOfObjects++;
                ob = new AC3DObject();
                in >> token;
                ob->type = string_to_objecttype(token);
            }
        break;
        case('r'):
            if (token == "rot") {
                in >> ob->rot(0,0);
                in >> ob->rot(0,1);
                in >> ob->rot(0,2);

                in >> ob->rot(1,0);
                in >> ob->rot(1,1);
                in >> ob->rot(1,2);

                in >> ob->rot(2,0);
                in >> ob->rot(2,1);
                in >> ob->rot(2,2);
            }
        break;
        case('t'):
            if (token == "texture") {
                std::string filename;
                in >> filename;
                filename = filename.substr(1,filename.length()-2);
                ob->texture = loadTexture( model->_currentDir+filename , model);
                std::cout << "Texture with name: " << ob->name << " has ID: " << ob->texture << std::endl;
                //RW_WARN("In AC3D file: Textures not supported yet!");
            } else if (token == "texrep") {
                std::cout << "Setting texture repeat!" << std::endl;
                in >> ob->texture_repeat_x;
                in >> ob->texture_repeat_y;
            } else if (token == "texoff") {
                std::cout << "Setting texture offset!" << std::endl;
                in >> ob->texture_offset_x;
                in >> ob->texture_offset_y;
            }
        break;
        case('u'):
            if (token == "url") {
                in >> ob->url;
            }
        break;
        default:
            RW_WARN("LoaderAC3D: UNKNOWN token!! ");
        }
    }
    calc_vertex_normals(ob);
    return ob;
}

void LoaderAC3D::calc_vertex_normals(AC3DObject *ob)
{
    // std::cout << "---------------- calc_vertex_normals: " << std::endl;
    // std::cout << "- vertex cnt: " << ob->vertex_cnt << std::endl;
    // std::cout << "- surface cnt: " << ob->surf_cnt << std::endl;
    /// long time = TimerUtil::currentTimeMs();
	// create vertexSurfaceNeigh map and matToSurfArray
	// run through all surfaces and add them to the vertex index
	ob->_vertSurfMap.resize( ob->vertex_cnt );
	for (int sIdx = 0; sIdx < ob->surf_cnt; sIdx++) {
		AC3DSurface *surf = &(ob->surfaces[sIdx]);

		//std::pair<int,int> key(surf->mat,surf->flags & 0xf);
    	//ob->_matToSurfArray[ key ].push_back(sIdx);

        for (int vrIdx = 0; vrIdx < surf->vertref_cnt; vrIdx++) {
        	ob->_vertSurfMap[ surf->vertrefs[vrIdx] ].push_back(surf);
        }
	}

	// use vertexSurfaceNeigh map to calculate per vertex normals
	Vector3D<float> n(0.0, 0.0, 0.0);
    for (int vIdx = 0; vIdx < ob->vertex_cnt; vIdx++) {
        BOOST_FOREACH(AC3DSurface *surf, ob->_vertSurfMap[vIdx]){

        	// for each surface calculate the vertex normal considering
        	// the angle between two surfaces
			Vector3D<float> n1 = surf->normal.toV3D();
			n = n1;
        	BOOST_FOREACH(AC3DSurface *nsurf, ob->_vertSurfMap[vIdx]){
				if(nsurf==surf)
					continue;
				// angle between surf normals must be less than MAX_ANGLE
				Vector3D<float> n2 = nsurf->normal.toV3D();
				if( dot(n1,n2) < cos(_maxAngle) )
					continue;
				// if it is then add
	        	n += n2;
        	}
        	// now be sure to update the correct vertex normal in the surface
        	for(int i=0; i<surf->vertref_cnt; i++){
        		if( surf->vertrefs[i]==vIdx){
        			surf->normals[i] = normalize(n);
        			break;
        		}

        	}
        }
    }
    // long timel = TimerUtil::currentTimeMs();
    // std::cout << "- time: " << timel-time << "ms" << std::endl;

}

int LoaderAC3D::loadTexture(const std::string& filename, ModelAC3D* model){
    Log::debugLog() << "LOADING TEXTURE: " << filename <<std::endl;
    rw::sensor::ImagePtr image;
    try{
        image = rw::loaders::ImageFactory::load( filename );
    } catch(...){
        return -1;
    }

    if( image==NULL )
        return -1;

    Log::debugLog() << "creating TEXTURE: " << std::endl;
    RWGLTexture *texture = new RWGLTexture( *image );

    model->_textures.push_back(texture);
    model->_textureMap[texture->getTextureID()] = texture;

    Log::debugLog() << "TEXTURE ID: " << texture->getTextureID() <<std::endl;

    return texture->getTextureID();
}

