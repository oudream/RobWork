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

#include "RenderText.hpp"
#include <rw/math/Math.hpp>
using namespace rwlibs::opengl;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;


namespace {
    rw::math::Vector3D<> project(double x, double y, double z) 
    {
        GLdouble modelMatrix[16];
        GLdouble projMatrix[16];
        GLint viewport[4];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
        glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
        glGetIntegerv(GL_VIEWPORT, viewport);

        GLdouble winx, winy, winz;
        gluProject(x,y,z,modelMatrix,projMatrix,viewport, &winx, &winy ,&winz);
    return Vector3D<>(winx,winy,winz);
    }


    rw::math::Vector3D<> unproject(int x, int y)
    {
        GLfloat depth;
        glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
        GLdouble modelMatrix[16];
        GLdouble projMatrix[16];
        GLint viewport[4];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
        glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
        glGetIntegerv(GL_VIEWPORT, viewport);
        GLdouble objx, objy, objz;
        gluUnProject(x, y, depth, modelMatrix, projMatrix, viewport, &objx, &objy, &objz);
        return Vector3D<>(objx,objy,objz);
    }
    rw::math::Vector3D<> unproject(Vector3D<> pos2D) 
    {
        return unproject(pos2D[0],pos2D[1]);
    }

}


#ifdef RW_HAVE_GLUT
#include <GL/glut.h>
#include <GL/freeglut.h>

void RenderText::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
{   
    void *font = GLUT_BITMAP_HELVETICA_18;

    //Drawing text
    Vector3D<> pos = unproject(project(0,0,0)); //_frame->getTransform(*info._state).P();
    std::cout << pos << std::endl;
    glRasterPos3f( pos[0] , pos[1] , pos[2] );

    glColor3f(0,0,0);
    const unsigned char* text = (unsigned char*)_text.c_str();
    glutBitmapString(font,text);
    
    //get Text Dimentions
    int length = glutBitmapLength(font,text);
    int height = glutBitmapHeight(font);

    //Find camera
    if ( !info._cam.isNull()) {  
        SceneCamera::Ptr cam = info._cam;
        Transform3D<double> wTc = cam->getTransform();
        Transform3D<double> cTw = inverse(wTc);

        Vector3D<> posCam= cTw*pos;
        Vector3D<> posT = wTc*(posCam+Vector3D<>(1,1,0));

        Vector3D<> p2D_1 = project(pos[0],pos[1],pos[2] );
        Vector3D<> p2D_2 = project(posT[0],posT[1],posT[2] );

        double pixelPerMeter = std::abs(p2D_1[0]-p2D_2[0]);
        double x_scale = length/pixelPerMeter;
        double y_scale = height/pixelPerMeter;
       
        double posX = 0/pixelPerMeter;
        double posY = -5/pixelPerMeter;
        Vector3D<> pos1 = wTc*(posCam+Vector3D<>(posX         ,posY         ,0));
        Vector3D<> pos2 = wTc*(posCam+Vector3D<>(x_scale+posX ,posY         ,0));
        Vector3D<> pos3 = wTc*(posCam+Vector3D<>(x_scale+posX ,y_scale+posY ,0));
        Vector3D<> pos4 = wTc*(posCam+Vector3D<>(posX         ,y_scale+posY ,0));

        glPolygonMode(GL_FRONT, GL_FILL);
        glColor4f(255,255,255,255);

        glEnable( GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1,1);
        glBegin(GL_QUADS);
        glVertex3f(pos1[0], pos1[1], pos1[2] );  // vertex 1
        glVertex3f(pos2[0], pos2[1],pos2[2]); // vertex 2
        glVertex3f(pos3[0], pos3[1],pos3[2]); // vertex 3
        glVertex3f(pos4[0], pos4[1],pos4[2]); // vertex 4
        glEnd();
        glDisable(GL_POLYGON_OFFSET_FILL);

        glPolygonMode(GL_FRONT, GL_LINE);
        glColor4f(0,0,0,255);
        glBegin(GL_QUADS);
        glVertex3f(pos1[0], pos1[1], pos1[2] );  // vertex 1
        glVertex3f(pos2[0], pos2[1],pos2[2]); // vertex 2
        glVertex3f(pos3[0], pos3[1],pos3[2]); // vertex 3
        glVertex3f(pos4[0], pos4[1],pos4[2]); // vertex 4
        glEnd();
    }
}

#endif

#ifndef RW_HAVE_GLUT
void RenderText::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const {}
#endif

RenderText::RenderText(std::string text):_text(text)
{

}

