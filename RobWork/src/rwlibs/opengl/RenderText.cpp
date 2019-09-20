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
#include <rw/kinematics/Frame.hpp>

#ifdef RW_HAVE_GLUT
    #include <GL/glut.h>
    #include <GL/freeglut.h>
#endif

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

    void renderSquare(Vector3D<> pos1,Vector3D<> pos2, Vector3D<> pos3, Vector3D<> pos4, std::vector<GLfloat> boxColor, std::vector<GLfloat> borderColor) 
    {
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glEnable(GL_BLEND);
        //Draw Square
        glPolygonMode(GL_FRONT, GL_FILL);
        if ( boxColor.size() == 4u ) {
            glColor4f(boxColor[0],boxColor[1],boxColor[2],boxColor[3]);
        } else if ( boxColor.size() == 3u ) {
            glColor3f(boxColor[0],boxColor[1],boxColor[2]);
        } else {
            glColor3f(255,255,255);
        }
        glColor3f(255,255,255);
        glEnable( GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1,1);
        glBegin(GL_QUADS);
        glVertex3f(pos1[0], pos1[1], pos1[2] );  // vertex 1
        glVertex3f(pos2[0], pos2[1],pos2[2]); // vertex 2
        glVertex3f(pos3[0], pos3[1],pos3[2]); // vertex 3
        glVertex3f(pos4[0], pos4[1],pos4[2]); // vertex 4
        glEnd();
        glDisable(GL_POLYGON_OFFSET_FILL);

        //Draw bounding box
        glPolygonMode(GL_FRONT, GL_LINE);
        if ( boxColor.size() == 4u ) {
            glColor4f(borderColor[0],borderColor[1],borderColor[2],borderColor[3]);
        } else if ( boxColor.size() == 3u ) {
            glColor3f(borderColor[0],borderColor[1],borderColor[2]);
        } else {
            glColor3f(0,0,0);
        }

        glBegin(GL_QUADS);
        glVertex3f(pos1[0], pos1[1], pos1[2] );  // vertex 1
        glVertex3f(pos2[0], pos2[1],pos2[2]); // vertex 2
        glVertex3f(pos3[0], pos3[1],pos3[2]); // vertex 3
        glVertex3f(pos4[0], pos4[1],pos4[2]); // vertex 4
        glEnd();
        glDisable(GL_BLEND);
        glPopAttrib();
    }

    double pixelPerMeter(Transform3D<> fTc, const Vector3D<> &pos, Vector3D<> &posCam)
    {
        Transform3D<double> cTf = inverse(fTc);

        posCam= cTf*pos;
        Vector3D<> posT = fTc*(posCam+Vector3D<>(1,1,0));

        Vector3D<> p2D_1 = project(pos[0],pos[1],pos[2] );
        Vector3D<> p2D_2 = project(posT[0],posT[1],posT[2] );

        return std::abs(p2D_1[0]-p2D_2[0]);
    }
    void moveCloserToCam(Transform3D<> fTc, Vector3D<> &pos, double distance)
    {

        std::cout << "Cam Pos: " << fTc.P() << std::endl;
        std::cout << "Pos    : " << pos << std::endl;
        //Transform3D<double> wTc = cam->getTransform();
        //Transform3D<double> cTw = inverse(wTc);
        //pos=cTw*pos;

        Vector3D<> diff = fTc.P()-pos;

        std::cout << "Dif     : " << diff << std::endl;
        double movePercent = distance/diff.norm2();
        if (movePercent > 0.90) {
            movePercent = 0.9;
        }
        pos+=diff*movePercent;
        //pos=wTc*pos;
    }
}


#ifdef RW_HAVE_GLUT
    #define HELVETICA_18 GLUT_BITMAP_HELVETICA_18
    #define HELVETICA_12 GLUT_BITMAP_HELVETICA_12
    #define HELVETICA_10 GLUT_BITMAP_HELVETICA_10
    namespace {
        void drawText(Vector3D<> pos,std::string text,void* font) 
        {
            glRasterPos3f( pos[0] , pos[1] , pos[2] );
            glColor4f(0,0,0,1.0);
            glutBitmapString(font,(unsigned char*)text.c_str());
        }
    }
    void RenderText::findTextDimensions() {
        _haveGlut=true;
        _textWidth = glutBitmapLength(_font,(unsigned char*)_text.c_str());
        _textHight = glutBitmapHeight(_font);
    }
#endif 
#ifndef RW_HAVE_GLUT
    #define HELVETICA_18 NULL
    #define HELVETICA_12 NULL
    #define HELVETICA_10 NULL
    namespace {void drawText(Vector3D<> pos) {}}
    void RenderText::findTextDimensions() {
        _haveGlut=false;
    }
#endif

RenderText::RenderText(std::string text, Frame::Ptr frame):_frame(frame),_font(HELVETICA_12)
{
    _text=" "+text+" ";
    findTextDimensions();
}



void RenderText::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
{   
    if (_haveGlut && !info._cam.isNull()) {
        SceneCamera::Ptr cam = info._cam;
        Transform3D<> wTf = _frame->wTf(*(info._state));
        Transform3D<> wTc = cam->getTransform();
        Transform3D<> fTc = inverse(wTf)*wTc;
        
        //Drawing text
        Vector3D<> pos(0,0,0);
        moveCloserToCam(fTc,pos,0.5); 
        drawText(pos,_text,_font);

        getLabelCorners(fTc,1,1,0,0);
        Vector3D<> posCam;
        double ppm = pixelPerMeter(fTc,pos,posCam);
        double width = _textWidth/ppm;
        double height = _textHight/ppm;
        double posX = 0/ppm;
        double posY = -5/ppm;

        Vector3D<> pos1 = fTc*(posCam+Vector3D<>(posX       ,posY         ,0));
        Vector3D<> pos2 = fTc*(posCam+Vector3D<>(width+posX ,posY         ,0));
        Vector3D<> pos3 = fTc*(posCam+Vector3D<>(width+posX ,height+posY ,0));
        Vector3D<> pos4 = fTc*(posCam+Vector3D<>(posX       ,height+posY ,0));

        renderSquare(pos1,pos2,pos3,pos4, {255,255,255,float(alpha)}, {0.5,0.5,0.5,float(alpha)});
    }
}

std::vector<rw::math::Vector3D<> > RenderText::getLabelCorners(Transform3D<> fTc, double scale_x, double scale_y, int move_x , int move_y) const
{
    return std::vector<rw::math::Vector3D<> >();
}


