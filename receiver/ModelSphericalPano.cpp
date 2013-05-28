#include <stdexcept>
#include <GL/glew.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ModelSphericalPano.hpp"

#define PI 3.14159265359

ModelSphericalPano::ModelSphericalPano(double _radius, double _slices, double _stacks) :
    radius(_radius),
    slices(_slices),
    stacks(_stacks),
    n_elements(0),
    textureImg_w(0),
    textureImg_h(0)
{
    glGenVertexArrays(1, vao);
    glGenBuffers(n_vbo, vbo);
    glGenTextures(1, tex);
    Eigen::Matrix4f::Map(&model_matrix[0]).setIdentity();
    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[0]);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindVertexArray(0);
}

ModelSphericalPano::~ModelSphericalPano()
{
    glDeleteBuffers(1, vbo);
    glDeleteTextures(1, tex);
    glDeleteVertexArrays(1, vao);
}

MyPanoVertex ModelSphericalPano::getPoint(double phi, double theta) const
{
    double x = radius * cos(theta) * sin(phi);
    double y = radius * sin(theta) * sin(phi);
    double z = radius * cos(phi);
    return MyPanoVertex(x, y, z);
}

void ModelSphericalPano::drawSphereDirectRendering() const
{

   glBegin(GL_QUADS);
   for(int i=0; i < vertices.size(); i+=4)
   {

       MyPanoVertex p1 = vertices[i];
       MyPanoVertex p2 = vertices[i+1];
       MyPanoVertex p3 = vertices[i+2];
       MyPanoVertex p4 = vertices[i+3];

       glTexCoord2f(p1.u,p1.v);
       glVertex3f(p1.x,p1.y,p1.z);
       glTexCoord2f(p2.u,p2.v);
       glVertex3f(p2.x,p2.y,p2.z);
       glTexCoord2f(p3.u,p3.v);
       glVertex3f(p3.x,p3.y,p3.z);
       glTexCoord2f(p4.u,p4.v);
       glVertex3f(p4.x,p4.y,p4.z);

   }
   glEnd();
}

void ModelSphericalPano::draw() const
{
    if (n_elements == 0)
        return;
    GLint front_face;
    glGetIntegerv(GL_FRONT_FACE, &front_face);
    glFrontFace(GL_CW);
    glBindVertexArray(vao[0]);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glPushMatrix();
    glMultMatrixf(model_matrix);
    glDrawElements(GL_QUADS, n_elements, GL_UNSIGNED_INT, 0);
    glPopMatrix();
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    glFrontFace(front_face);
}

void ModelSphericalPano::prepareSphereManual()
{
    double stack = (2 * PI) / stacks;
    double slice = (2 * PI) / slices;
    unsigned k = 0;
    for (double theta = 0; theta <= 2 * PI; theta += stack) {
        for (double phi = 0; phi <= 1 * PI; phi += slice) {
            float s0 = theta / (2 * PI);
            float s1 = (theta + stack) / (2 * PI);
            float t0 = phi / (1 * PI);
            float t1 = (phi + slice) / (1 * PI);
            MyPanoVertex p1 = getPoint(phi, theta);
            MyPanoVertex p2 = getPoint(phi + slice, theta);
            MyPanoVertex p3 = getPoint(phi + slice, theta + stack);
            MyPanoVertex p4 = getPoint(phi, theta + stack);
            p1.u = s0; p1.v = t0;
            p2.u = s0; p2.v = t1;
            p3.u = s1; p3.v = t1;
            p4.u = s1; p4.v = t0;
            vertices.push_back(p1);
            vertices.push_back(p2);
            vertices.push_back(p3);
            vertices.push_back(p4);
            vertex_indices.push_back(k);
            vertex_indices.push_back(k+1);
            vertex_indices.push_back(k+2);
            vertex_indices.push_back(k+3);
            tris.push_back(p1.x); tris.push_back(p1.y); tris.push_back(p1.z);
            tris.push_back(p2.x); tris.push_back(p2.y); tris.push_back(p2.z);
            tris.push_back(p3.x); tris.push_back(p3.y); tris.push_back(p3.z);
            tris.push_back(p4.x); tris.push_back(p4.y); tris.push_back(p4.z);
            tex_coord.push_back(p1.u); tex_coord.push_back(p1.v);
            tex_coord.push_back(p2.u); tex_coord.push_back(p2.v);
            tex_coord.push_back(p3.u); tex_coord.push_back(p3.v);
            tex_coord.push_back(p4.u); tex_coord.push_back(p4.v);
            k+=4;
        }
    }
}

void ModelSphericalPano::load(const char* texture_name)
{

    cv::Mat img = cv::imread(texture_name);
    textureImg_w = img.cols;
    textureImg_h = img.rows;
    prepareSphereManual();
    n_elements = vertex_indices.size();
    if (n_elements == 0)
        return;
    glBindVertexArray(vao[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * vertex_indices.size(), vertex_indices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * tris.size(), tris.data(), GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * tex_coord.size(), tex_coord.data(), GL_STATIC_DRAW);
    glTexCoordPointer(2, GL_FLOAT, 0, 0);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureImg_w, textureImg_h, 0, GL_BGR, GL_UNSIGNED_BYTE, img.data);
    glBindVertexArray(0);

}
