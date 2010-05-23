
#include "LoaderTRI.hpp"

#include <cmath>                       // Header file for the math library
#include <iostream>
#include <fstream>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw::common;
using namespace rwlibs::drawable;
using namespace rwlibs::drawable;
using namespace rw::geometry;
using namespace rw::math;

#define LINE_MAX_LENGTH 100

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

LoaderTRI::LoaderTRI(){};
LoaderTRI::~LoaderTRI(){};


Model3DPtr LoaderTRI::load(const std::string& filename)
{

    std::ifstream input_stream(filename.c_str());
    if (!input_stream.is_open()) {
        RW_THROW("Can't open file " << StringUtil::quote(filename));
    }
    setlocale(LC_ALL, "C");
    char *next;
    char  token[LINE_MAX_LENGTH];
    int   width;
    char input[LINE_MAX_LENGTH];
    int nb_points = 0;
    Model3D *model = new Model3D();
    Model3D::Object3D *obj = new Model3D::Object3D("TRIModel");

    int currentMatIdx = model->addMaterial(Model3D::Material("defcol",0.5,0.5,0.5));

    // while characters still exists and no errors occour
    while (  input_stream.fail()==0 && input_stream.eof()==0 ){
        //  Read the next line of the file into INPUT.
        input_stream.getline(input, LINE_MAX_LENGTH);
        //  Advance to the first nonspace character in INPUT.
        for ( next = input; *next != '\0' && *next == 32; next++ ){}
        // 32==SPACE character

        //  Skip blank lines and comments and linebreaks.
        if ( *next == '\0' || *next == '#' || *next == '!' || *next == '$'  ){
            continue;
        }
        //  Extract the first word in this line.
        sscanf ( next, "%s%n", token, &width );

        //  Set NEXT to point to just after this token.
        next = next + width;

        if ( !strcmp( token, "color" ) ){
            float r,g,b;
            sscanf ( next, "%e %e %e", &r, &g, &b );
            // create material in object
            std::stringstream sstr;
            sstr << r <<"_"<< g << "_" << b;
            Model3D::Material mat(sstr.str(), r, g, b);
            currentMatIdx = model->addMaterial(mat);
        } else if( !strcmp( token, "point" ) ){
            float x,y,z,nx,ny,nz;
            sscanf ( next, "%e %e %e %e %e %e", &x, &y, &z, &nx, &ny, &nz );
            Vector3D<float> n(nx,ny,nz);
            Vector3D<float> v(x,y,z);

            obj->_vertices.push_back(v);
            obj->_normals.push_back(n);
            nb_points++;
            if(nb_points%3==0){
                //obj->_faces3.push_back(IndexedTriangleN3<float>(nb_points-3,nb_points-2,nb_points-1,
                //                                                nb_points-3,nb_points-2,nb_points-1));
            }
        } else {
            setlocale(LC_ALL, "");
            RW_THROW("unrecognized keyword " << StringUtil::quote(token));
        }
    }
    setlocale(LC_ALL, "");
	return model;
}
