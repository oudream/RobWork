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


#include "RenderGeometry.hpp"
#include <rw/math/Vector3D.hpp>
#include <rw/geometry/GeometryUtil.hpp>
using namespace rw::geometry;
using namespace rwlibs::drawable;

using namespace rw::math;

namespace
{
    void setArray4(float *array, float v0, float v1, float v2, float v3 ){
    	array[0]=v0;
    	array[1]=v1;
    	array[2]=v2;
    	array[3]=v3;
    }
}


RenderGeometry::RenderGeometry(GeometryPtr geometry):
		_geometry(NULL),
    _r(0.8f),_g(0.8f),_b(0.8f)

{
	setGeometry(geometry);
}

void RenderGeometry::setGeometry(rw::geometry::GeometryPtr geom){
    setArray4(_diffuse, 0.8f,0.8f,0.8f,1.0f);
    setArray4(_ambient, 0.2f,0.2f,0.2f,1.0f);
    setArray4(_emission, 0.0f,0.0f,0.0f,0.0f);
    setArray4(_specular, 0.2f,0.2f,0.2f,1.0f);
    _shininess[0] = 128;

    // create displaylist
    GLuint displayListId = glGenLists(1);
    glNewList(displayListId, GL_COMPILE);
    glPushMatrix();
    //glPopAttrib(); // pop color and material attributes
    glBegin(GL_TRIANGLES);
    // Draw all faces.
    GeometryDataPtr geomdata = geom->getGeometryData();
    TriMeshPtr mesh = GeometryUtil::toTriMesh(geomdata);

    for(size_t i=0;i<mesh->size();i++){
    	TriangleN0<double> tri = mesh->getTriangle(i);
    	Vector3D<float> n = cast<float>(tri.calcFaceNormal());
    	Vector3D<float> v0 = cast<float>(tri[0]);
    	Vector3D<float> v1 = cast<float>(tri[1]);
    	Vector3D<float> v2 = cast<float>(tri[2]);
    	glNormal3fv(&n[0]);
        glVertex3fv(&v0[0]);
        glVertex3fv(&v1[0]);
        glVertex3fv(&v2[0]);
    }

    glEnd();
    glPopMatrix();
    glEndList();
    if(_geometry!=NULL)
    	glDeleteLists(_displayListId, 1);
    _displayListId = displayListId;
    _geometry = geom;

}

RenderGeometry::~RenderGeometry() {
	glDeleteLists(_displayListId, 1);
}

void RenderGeometry::setColor(float r, float g, float b) {
    _r = r;
    _g = g;
    _b = b;
}

void RenderGeometry::draw(DrawType type, double alpha) const{
	glColor4f(_r, _g, _b, (float)alpha);
	_diffuse[3] = (float)alpha;
	glMaterialfv(GL_FRONT, GL_AMBIENT, _ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, _diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, _specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, _shininess);
	glMaterialfv(GL_FRONT, GL_EMISSION, _emission);

	switch(type){
    case Render::SOLID:
    	glPolygonMode(GL_FRONT, GL_FILL);
		glCallList(_displayListId);
		break;
    case Render::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT, GL_FILL);
		glCallList(_displayListId);
    case Render::WIRE: // Draw nice frame
    	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    	glCallList(_displayListId);
    	break;
	}
}
