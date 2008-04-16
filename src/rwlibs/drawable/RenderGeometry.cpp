#include "RenderGeometry.hpp"

using namespace rw::geometry;
using namespace rwlibs::drawable;

namespace
{
    void drawFace(const Face<float>& face)
    {
        glNormal3fv(face._normal);
        glVertex3fv(face._vertex1);
        glVertex3fv(face._vertex2);
        glVertex3fv(face._vertex3);
    }
    
    void setArray4(float *array, float v0, float v1, float v2, float v3 ){
    	array[0]=v0;
    	array[1]=v1;
    	array[2]=v2;
    	array[3]=v3;
    }
}


RenderGeometry::RenderGeometry(Geometry* geometry):
    _geometry(geometry),
    _r(0.8),_g(0.8),_b(0.8)
{
	setArray4(_diffuse, 0.8,0.8,0.8,1.0);
	setArray4(_ambient, 0.2,0.2,0.2,1.0);
	setArray4(_emission, 0.0,0.0,0.0,0.0);
	setArray4(_specular, 0.2,0.2,0.2,1.0);
	_shininess[0] = 128;

	// create displaylist
    _displayListId = glGenLists(1);
    glNewList(_displayListId, GL_COMPILE);
    glPushMatrix();
    //glPopAttrib(); // pop color and material attributes
    glBegin(GL_TRIANGLES);
    // Draw all faces.
    std::for_each(_geometry->getFaces().begin(), 
    		  	  _geometry->getFaces().end(), drawFace);
    glEnd();
    glPopMatrix();
    glEndList();
}


RenderGeometry::~RenderGeometry() {
	glDeleteLists(_displayListId, 1);
    delete _geometry;
}

void RenderGeometry::setColor(float r, float g, float b) {
    _r = r;
    _g = g;
    _b = b;
}

void RenderGeometry::draw(DrawType type, double alpha) const{
	glColor4f(_r, _g, _b, alpha);
	_diffuse[3] = alpha;
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
