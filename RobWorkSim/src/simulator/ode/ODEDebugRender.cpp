#include "ODEDebugRender.hpp"

#include "ODESimulator.hpp"

#include <rwlibs/os/rwgl.hpp>
#include <ode/ode.h>
#include <boost/foreach.hpp>
#include <rwlibs/drawable/DrawableUtil.hpp>

using namespace drawable;
using namespace rwlibs::drawable;

namespace {

    void odeToGLTransform(
        const dReal* pos,
        const dReal* rot,
        GLfloat* gltrans)
    {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++)
                gltrans[j + 4 * k] =
                    (float)rot[4 *j +  k];

            gltrans[12 + j] =
                (float)pos[j];
        }

        gltrans[3] = gltrans[7] = gltrans[11] = 0;
        gltrans[15] = 1;
    }
}

void ODEDebugRender::draw(DrawType draw, double alpha) const {

  const std::vector<ODESimulator::TriGeomData*>& trimeshs = _sim->getTriMeshs();
  //std::vector<dContact> contacts = _sim->getContacts();

  BOOST_FOREACH(ODESimulator::TriGeomData* trigeom, trimeshs){
      ODESimulator::TriMeshDataPtr trimesh = trigeom->tridata;
      // multiply stack transform with geom transform
      const dReal* pos = dGeomGetPosition(trigeom->geomId);
      const dReal* rot = dGeomGetRotation(trigeom->geomId);

      float gltrans[16];
      odeToGLTransform(pos,rot,gltrans);

      glPushMatrix();
      glMultMatrixf(gltrans);
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      glBegin(GL_TRIANGLES);

      for (size_t i = 0; i < trimesh->indices.size()/3; i++){
        const float *p;
        p = &trimesh->vertices[ trimesh->indices[i * 3 + 0]*3 ];
        glVertex3f((float)p[0],(float)p[1],(float)p[2]);

        p = &trimesh->vertices[ trimesh->indices[i * 3 + 1]*3 ];
        glVertex3f((float)p[0],(float)p[1],(float)p[2]);

        p = &trimesh->vertices[ trimesh->indices[i * 3 + 2]*3 ];
        glVertex3f((float)p[0],(float)p[1],(float)p[2]);
      }

      // draw all contacts
      glEnd();
      glPopMatrix();
  }

  //getContactManifoldMap

  glPushMatrix();
  //glPointSize(20.0);
  //glBegin(GL_POINTS);
  std::vector<ContactPoint> contacts = _sim->getContacts();
  for(size_t i=0; i<contacts.size(); i++){
      ContactPoint &con = contacts[i];
      // draw the contact normal
      //DrawableUtil::drawGLVertex(con.p);
      //if(DRAW_CONTACT_NORMAL & _drawMask){
	  glLineWidth(0.1);
	  glBegin(GL_LINES);
	  glColor3f(1.0, 0.0, 0.0);
	  DrawableUtil::drawGLVertex(con.p);
	  glColor3f(0.0, 1.0, 0.0);
	  DrawableUtil::drawGLVertex(con.p+con.n);
	  glEnd();

	  //}
  }
  //glEnd( );
  glPopMatrix();

  //if(DRAW_CONTACT_NORMAL & _drawMask){
   /*   glPushMatrix();
      //const std::vector<ContactPoint>& contacts = _sim->getContacts();
      for(int i=0; i<contacts.size(); i++){
          ContactPoint con = contacts[i];
          // draw the contact normal
          glLineWidth(0.1);
          glBegin(GL_LINES);
          glColor3f(1.0, 0.0, 0.0);
          DrawableUtil::drawGLVertex(con.p);
          DrawableUtil::drawGLVertex(con.p+con.n);
          glEnd();
      }
      glPopMatrix();
  //}
   * */



}
