#include <stdexcept>
#include <fstream>
#include <GL/glew.h>
#include <QGLContext>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Model.hpp"
#include "Data3d.hpp"

Model::Model(QObject* parent) :
    QObject(parent),
    n_elements(0)
{
    glGenVertexArrays(1, vao);
    glGenBuffers(n_vbo, vbo);
    glGenTextures(1, tex);
    Eigen::Matrix4f::Map(&model_matrix[0]).setIdentity();
    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[0]);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindVertexArray(0);
}

Model::~Model()
{
    glDeleteBuffers(n_vbo, vbo);
    glDeleteTextures(1, tex);
    glDeleteVertexArrays(1, vao);
}

void Model::draw() const
{
    if (n_elements == 0)
	return;
    glBindVertexArray(vao[0]);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glPushMatrix();
    glMultMatrixf(model_matrix);
    glDrawElements(GL_TRIANGLES, n_elements, GL_UNSIGNED_INT, 0);
    glPopMatrix();
    glBindVertexArray(0);
}

void Model::load(const Data3d* data)
{
    n_elements = data->tri.size();
    if (n_elements == 0)
	return;
    const int n_vertices = data->ver.size() / 3;
    const int n_triangles = data->tri.size() / 3;
    std::vector<float> normals(3 * n_vertices);
    Eigen::Map<const Eigen::MatrixXf> v(&data->ver[0], 3, n_vertices);
    Eigen::Map<Eigen::MatrixXf> n(&normals[0], 3, n_vertices);
    for (int i = 0; i < n_triangles; ++i) {
	unsigned int idx[3];
	for (int j = 0; j < 3; ++j)
	    idx[j] = data->tri[3 * i + j];
	const Eigen::Vector3f v0 = v.col(idx[0]);
	const Eigen::Vector3f v1 = v.col(idx[1]);
	const Eigen::Vector3f v2 = v.col(idx[2]);
	const Eigen::Vector3f normal = -(v1 - v0).cross(v2 - v0);
	for (int j = 0; j < 3; ++j)
	    n.col(idx[j]) += normal;
    }
    for (int i = 0; i < n_vertices; ++i)
	n.col(i).normalize();
    glBindVertexArray(vao[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * data->tri.size(), &data->tri.front(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * data->ver.size(), &data->ver.front(), GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * data->tex.size(), &data->tex.front(), GL_STATIC_DRAW);
    glTexCoordPointer(2, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * normals.size(), &normals.front(), GL_STATIC_DRAW);
    glNormalPointer(GL_FLOAT, 0, 0);
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, data->width, data->height, 0, GL_RGB, GL_UNSIGNED_BYTE, &data->bgr.front());
    glBindVertexArray(0);
}
