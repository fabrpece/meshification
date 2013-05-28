#include <stdexcept>
#include <sstream>
#include <iostream>
#include <GL/glew.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include "ModelCubemappedPano.hpp"

//#define USE_VBO

ModelCubemappedPano::ModelCubemappedPano(float _radius) :
    radius(_radius)
{
    Eigen::Matrix4f::Map(&model_matrix[0]).setIdentity();
    glGenTextures(1, tex);
#ifdef USE_VBO
    glGenVertexArrays(1, vao);
    glGenBuffers(n_vbo, vbo);
    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[0]);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glBindVertexArray(0);
#endif
}

ModelCubemappedPano::~ModelCubemappedPano()
{

}

void ModelCubemappedPano::draw() const
{

    GLint front_face;
    glGetIntegerv(GL_FRONT_FACE, &front_face);
    glFrontFace(GL_CW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#ifdef USE_VBO
    glBindVertexArray(vao[0]);
#endif
    glPushMatrix();
    glMultMatrixf(model_matrix);
#ifdef USE_VBO
    glDrawElements(GL_QUADS, n_elements, GL_UNSIGNED_INT, 0);
#else
    glEnable( GL_TEXTURE_CUBE_MAP );
    glBindTexture( GL_TEXTURE_CUBE_MAP, tex[0] );
    drawCubeManual();
#endif
    glBindTexture( GL_TEXTURE_CUBE_MAP, 0 );
    glDisable( GL_TEXTURE_CUBE_MAP );
    glPopMatrix();
#ifdef USE_VBO
    glBindVertexArray(0);
#endif
    glFrontFace(front_face);
}

void ModelCubemappedPano::drawCubeManual() const
{
    float A[3] = { radius,  radius,  radius };
    float B[3] = { radius,  radius, -radius };
    float C[3] = { radius, -radius, -radius };
    float D[3] = { radius, -radius,  radius };
    float E[3] = {-radius,  radius,  radius };
    float F[3] = {-radius,  radius, -radius };
    float G[3] = {-radius, -radius, -radius };
    float H[3] = {-radius, -radius,  radius };

    float I[3] = { 1.0f,  0.0f,  0.0f };
    float K[3] = {-1.0f,  0.0f,  0.0f };
    float L[3] = { 0.0f,  0.0f, -1.0f };
    float M[3] = { 0.0f,  0.0f,  1.0f };
    float N[3] = { 0.0f,  1.0f,  0.0f };
    float O[3] = { 0.0f, -1.0f,  0.0f };


    glBegin(GL_QUADS);

    glNormal3fv(I);
    glTexCoord3f(1,1,1);
    glVertex3fv(D);
    glTexCoord3f(1,1,-1);
    glVertex3fv(C);
    glTexCoord3f(1,-1,-1);
    glVertex3fv(B);
    glTexCoord3f(1,-1,1);
    glVertex3fv(A);

    glNormal3fv(K);
    glTexCoord3f(-1,1,-1);
    glVertex3fv(G);
    glTexCoord3f(-1,1,1);
    glVertex3fv(H);
    glTexCoord3f(-1,-1,1);
    glVertex3fv(E);
    glTexCoord3f(-1,-1,-1);
    glVertex3fv(F);

    glNormal3fv(L);
    glTexCoord3f(1,1,-1);
    glVertex3fv(C);
    glTexCoord3f(-1,1,-1);
    glVertex3fv(G);
    glTexCoord3f(-1,-1,-1);
    glVertex3fv(F);
    glTexCoord3f(1,-1,-1);
    glVertex3fv(B);

    glNormal3fv(M);
    glTexCoord3f(-1,1,1);
    glVertex3fv(H);
    glTexCoord3f(1,1,1);
    glVertex3fv(D);
    glTexCoord3f(1,-1,1);
    glVertex3fv(A);
    glTexCoord3f(-1,-1,1);
    glVertex3fv(E);

    glNormal3fv(N);
    glTexCoord3f(-1,-1,1);
    glVertex3fv(E);
    glTexCoord3f(1,-1,1);
    glVertex3fv(A);
    glTexCoord3f(1,-1,-1);
    glVertex3fv(B);
    glTexCoord3f(-1,-1,-1);
    glVertex3fv(F);

    glNormal3fv(O);
    glTexCoord3f(-1,1,-1);
    glVertex3fv(G);
    glTexCoord3f(1,1,-1);
    glVertex3fv(C);
    glTexCoord3f(1,1,1);
    glVertex3fv(D);
    glTexCoord3f(-1,1,1);
    glVertex3fv(H);

    glEnd();
}

void ModelCubemappedPano::prepareCubeMap()
{
    //D
    vertex_indices.push_back(0);
    tris.push_back(radius);
    tris.push_back(-radius);
    tris.push_back(radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    //C
    vertex_indices.push_back(1);
    tris.push_back(radius);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    //B
    vertex_indices.push_back(2);
    tris.push_back(radius);
    tris.push_back(radius);
    tris.push_back(-radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);
    //A
    vertex_indices.push_back(3);
    tris.push_back(radius);
    tris.push_back(radius);
    tris.push_back(radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);

    //G
    vertex_indices.push_back(4);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    //H
    vertex_indices.push_back(5);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tris.push_back(radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    //E
    vertex_indices.push_back(6);
    tris.push_back(-radius);
    tris.push_back(radius);
    tris.push_back(radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);
    //F
    vertex_indices.push_back(7);
    tris.push_back(-radius);
    tris.push_back(radius);
    tris.push_back(-radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);

    //C
    vertex_indices.push_back(8);
    tris.push_back(radius);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    //G
    vertex_indices.push_back(9);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    //F
    vertex_indices.push_back(10);
    tris.push_back(-radius);
    tris.push_back(radius);
    tris.push_back(-radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);
    //B
    vertex_indices.push_back(11);
    tris.push_back(radius);
    tris.push_back(radius);
    tris.push_back(-radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);

    //H
    vertex_indices.push_back(12);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tris.push_back(radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    //D
    vertex_indices.push_back(13);
    tris.push_back(radius);
    tris.push_back(-radius);
    tris.push_back(radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    //A
    vertex_indices.push_back(14);
    tris.push_back(radius);
    tris.push_back(radius);
    tris.push_back(radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);
    //E
    vertex_indices.push_back(15);
    tris.push_back(-radius);
    tris.push_back(radius);
    tris.push_back(radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);

    //E
    vertex_indices.push_back(16);
    tris.push_back(-radius);
    tris.push_back(radius);
    tris.push_back(radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);
    //A
    vertex_indices.push_back(17);
    tris.push_back(radius);
    tris.push_back(radius);
    tris.push_back(radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);
    //B
    vertex_indices.push_back(18);
    tris.push_back(radius);
    tris.push_back(radius);
    tris.push_back(-radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);
    //F
    vertex_indices.push_back(19);
    tris.push_back(-radius);
    tris.push_back(radius);
    tris.push_back(-radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(-1.0);

    //G
    vertex_indices.push_back(20);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    //C
    vertex_indices.push_back(21);
    tris.push_back(radius);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(-1.0);
    //D
    vertex_indices.push_back(22);
    tris.push_back(radius);
    tris.push_back(-radius);
    tris.push_back(radius);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
    //H
    vertex_indices.push_back(23);
    tris.push_back(-radius);
    tris.push_back(-radius);
    tris.push_back(radius);
    tex_coord.push_back(-1.0);
    tex_coord.push_back(1.0);
    tex_coord.push_back(1.0);
}

void ModelCubemappedPano::load(const char* cubemap_folder)
{
#ifdef USE_VBO
    if (n_elements == 0)
        return;

    prepareCubeMap();
    glBindVertexArray(vao[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * vertex_indices.size(), vertex_indices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * tris.size(), tris.data(), GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * tex_coord.size(), tex_coord.data(), GL_STATIC_DRAW);
    glTexCoordPointer(3, GL_FLOAT, 0, 0);
#endif

    glBindTexture( GL_TEXTURE_CUBE_MAP, tex[0] );
    glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    std::stringstream ss1;
    cv::Mat img1;
    ss1 << cubemap_folder << "/negx.jpg" << std::flush;
    img1 = cv::imread(ss1.str());
    glTexImage2D( GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGB, img1.cols, img1.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img1.data );

    std::stringstream ss2;
    cv::Mat img2;
    ss2 << cubemap_folder << "/posx.jpg" << std::flush;
    img2 = cv::imread(ss2.str());
    glTexImage2D( GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGB, img2.cols, img2.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img2.data );

    std::stringstream ss3;
    cv::Mat img3;
    ss3 << cubemap_folder << "/negy.jpg" << std::flush;
    img3 = cv::imread(ss3.str());
    glTexImage2D( GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGB, img3.cols, img3.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img3.data );

    std::stringstream ss4;
    cv::Mat img4;
    ss4 << cubemap_folder << "/posy.jpg" << std::flush;
    img4 = cv::imread(ss4.str());
    glTexImage2D( GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGB, img4.cols, img4.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img4.data );

    std::stringstream ss5;
    cv::Mat img5;
    ss5 << cubemap_folder << "/negz.jpg" << std::flush;
    img5 = cv::imread(ss5.str());
    glTexImage2D( GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGB, img5.cols, img5.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img5.data );

    std::stringstream ss6;
    cv::Mat img6;
    ss6 << cubemap_folder << "/posz.jpg" << std::flush;
    img6 = cv::imread(ss6.str());
    glTexImage2D( GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGB, img6.cols, img6.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img6.data );

#ifdef USE_VBO
    glBindVertexArray(0);
#endif
}
