#ifndef _MOdule3dDrawers_HH
#define _MOdule3dDrawers_HH
#include <QtGui/QOpenGLFunctions>
#include <opencv2/core.hpp>
#include <memory>
#include "tools/qtgl/object.h"
#include <GL/freeglut.h>
#include <GL/gl.h>


/* CUBE */
class GLCube:public qtgl::Object{
    float sizeX=1.0f,sizeY=1.0f,sizeZ = 1.0f;
public:
    std::string getType()const{return "Cube";} //return an string indicating the type of the subclass

    void setSize(float Size){sizeX=sizeY=sizeZ=Size;}
    void setSize(float SizeX,float SizeY,float SizeZ){sizeX=SizeX;sizeY=SizeY;sizeZ=SizeZ;}

    void  _draw_impl(){
        auto draw=[](cv::Point3d a,cv::Point3d b){
            glVertex3f ( a.x, a.y, a.z );
            glVertex3f ( b.x, b.y, b.z );

        };
        //draw
        float sx=sizeX/2;
        float sy=sizeY/2;
        float sz=sizeZ/2;
        std::vector<cv::Point3f> corners={   cv::Point3d(-sx,sy,sz),
                                            cv::Point3d(sx,sy,sz),
                                            cv::Point3d(sx,-sy,sz),
                                            cv::Point3d(-sx,-sy,sz),
                                            cv::Point3d(-sx,sy,-sz),
                                            cv::Point3d(sx,sy,-sz),
                                            cv::Point3d(sx,-sy,-sz),
                                            cv::Point3d(-sx,-sy,-sz)   };

        glLineWidth ( 3.0 );
        glBegin ( GL_LINES );
        {

            glColor3f ( 1, 1, 0  );
            draw(corners[0],corners[1]);
            glColor3f ( 1, 0, 0  );

            draw(corners[1],corners[2]);
            draw(corners[2],corners[3]);


            glColor3f ( 0, 1, 0  );
            draw(corners[3],corners[0]);

            glColor3f ( 1, 0, 0  );
            draw(corners[4],corners[5]);
            draw(corners[5],corners[6]);
            draw(corners[6],corners[7]);
            draw(corners[7],corners[4]);

            glColor3f ( 0, 0, 1  );
            draw(corners[0],corners[4]);

            glColor3f ( 1, 0, 0  );
            draw(corners[1],corners[5]);
            draw(corners[2],corners[6]);
            draw(corners[3],corners[7]);
        }
        glEnd();
    }

};


/* LINE */
class GLLine: public qtgl::Object {
    cv::Point3f p1, p2;

public:

    // Line color. Red by default
    float color_x = 1.f;
    float color_y = 0.f;
    float color_z = 0.f;

    std::string getType()const{return "Line";}

    void setPoints(const cv::Point3f& point1, const cv::Point3f& point2) {
        p1 = point1;
        p2 = point2;
    }

    void _draw_impl() {
        glLineWidth(2.0f);
        // Set color
        glColor3f(color_x, color_y, color_z);
        // Draw vertex
        glBegin(GL_LINES);
        glVertex3f(p1.x, p1.y, p1.z);
        glVertex3f(p2.x, p2.y, p2.z);
        glEnd();
    }
};




/* POINT */
class GLPoint : public qtgl::Object {
    cv::Point3f point;

public:
    // Point color. Green by default
    float color_x = 0.f;
    float color_y = 1.f;
    float color_z = 0.f;

    std::string getType() const { return "Point"; }

    void setPoint(const cv::Point3f& p) {
        point = p;
    }

    void _draw_impl() {
        glPointSize(5.0f); // Adjust the point size as needed
        // Set color
        glColor3f(color_x, color_y, color_z);
        // Draw vertex
        glBegin(GL_POINTS);
        glVertex3f(point.x, point.y, point.z);
        glEnd();
    }
};




#endif
