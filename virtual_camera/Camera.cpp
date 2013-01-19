#include <iostream>
#include <cstdlib>
#include <GL/glew.h>
#include <Eigen/Core>
#include "Camera.hpp"

static bool gluInvertMatrix(const double m[16], double invOut[16])
{
    double inv[16], det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] - 
             m[5]  * m[11] * m[14] - 
             m[9]  * m[6]  * m[15] + 
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] - 
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] + 
              m[4]  * m[11] * m[14] + 
              m[8]  * m[6]  * m[15] - 
              m[8]  * m[7]  * m[14] - 
              m[12] * m[6]  * m[11] + 
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] - 
             m[4]  * m[11] * m[13] - 
             m[8]  * m[5] * m[15] + 
             m[8]  * m[7] * m[13] + 
             m[12] * m[5] * m[11] - 
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] + 
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] - 
               m[8]  * m[6] * m[13] - 
               m[12] * m[5] * m[10] + 
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] + 
              m[1]  * m[11] * m[14] + 
              m[9]  * m[2] * m[15] - 
              m[9]  * m[3] * m[14] - 
              m[13] * m[2] * m[11] + 
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] - 
             m[0]  * m[11] * m[14] - 
             m[8]  * m[2] * m[15] + 
             m[8]  * m[3] * m[14] + 
             m[12] * m[2] * m[11] - 
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] + 
              m[0]  * m[11] * m[13] + 
              m[8]  * m[1] * m[15] - 
              m[8]  * m[3] * m[13] - 
              m[12] * m[1] * m[11] + 
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] - 
              m[0]  * m[10] * m[13] - 
              m[8]  * m[1] * m[14] + 
              m[8]  * m[2] * m[13] + 
              m[12] * m[1] * m[10] - 
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] - 
             m[1]  * m[7] * m[14] - 
             m[5]  * m[2] * m[15] + 
             m[5]  * m[3] * m[14] + 
             m[13] * m[2] * m[7] - 
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] + 
              m[0]  * m[7] * m[14] + 
              m[4]  * m[2] * m[15] - 
              m[4]  * m[3] * m[14] - 
              m[12] * m[2] * m[7] + 
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] - 
              m[0]  * m[7] * m[13] - 
              m[4]  * m[1] * m[15] + 
              m[4]  * m[3] * m[13] + 
              m[12] * m[1] * m[7] - 
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] + 
               m[0]  * m[6] * m[13] + 
               m[4]  * m[1] * m[14] - 
               m[4]  * m[2] * m[13] - 
               m[12] * m[1] * m[6] + 
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + 
              m[1] * m[7] * m[10] + 
              m[5] * m[2] * m[11] - 
              m[5] * m[3] * m[10] - 
              m[9] * m[2] * m[7] + 
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - 
             m[0] * m[7] * m[10] - 
             m[4] * m[2] * m[11] + 
             m[4] * m[3] * m[10] + 
             m[8] * m[2] * m[7] - 
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + 
               m[0] * m[7] * m[9] + 
               m[4] * m[1] * m[11] - 
               m[4] * m[3] * m[9] - 
               m[8] * m[1] * m[7] + 
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - 
              m[0] * m[6] * m[9] - 
              m[4] * m[1] * m[10] + 
              m[4] * m[2] * m[9] + 
              m[8] * m[1] * m[6] - 
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return false;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

    return true;
}

Camera::Camera(const std::string& name, const int w, const int h) :
    name(name),
    width(w),
    height(h),
    n_points(0),
    color(std::array<float, 3>{0, 1, 0})
{
    glGenFramebuffers(fbo.size(), fbo.data());
    glGenRenderbuffers(rbo.size(), rbo.data());
    glBindRenderbuffer(GL_RENDERBUFFER, rbo[0]);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo[1]);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo[0]);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rbo[0]);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo[1]);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    look_at(0, 2, -0.5, 0, 0.5, 0, 0, 1, 0);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluPerspective(49.8, static_cast<double>(width) / height, 0.1, 100);
    glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix.data());
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glGenVertexArrays(vao.size(), vao.data());
    glGenBuffers(vbo.size(), vbo.data());
}

Camera::~Camera()
{
    glDeleteFramebuffers(fbo.size(), fbo.data());
    glDeleteRenderbuffers(rbo.size(), rbo.data());
    glDeleteBuffers(vbo.size(), vbo.data());
    glDeleteVertexArrays(vao.size(), vao.data());
}

void Camera::render(const std::function<void()>& f)
{
    glBindFramebuffer(GL_FRAMEBUFFER, fbo[0]);
    glPushMatrix();
    glLoadMatrixd(modelview_matrix.data());
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixd(projection_matrix.data());
    glPushAttrib(GL_VIEWPORT_BIT);
    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    f();
    glPopAttrib();
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Camera::get_frame(std::vector<unsigned char>& buffer_rgb, std::vector<float>& point_cloud)
{
    buffer_rgb.resize(3 * width * height);
    point_cloud.resize(3 * width * height);
    std::vector<float> buffer_depth(width * height);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo[0]);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, &buffer_rgb[0]);
    glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, &buffer_depth[0]);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    for (int i = 0; i < width * height / 2; ++i) {
	const div_t coord = std::div(i, width);
	const int x = coord.rem, y = coord.quot, y_inv = height - 1 - y;
	std::swap(buffer_depth[i], buffer_depth[y_inv * width + x]);
	for (int j = 0; j < 3; ++j)
	    std::swap(buffer_rgb[3 * i + j], buffer_rgb[3 * (y_inv * width + x) + j]);
    }
    int viewport[4] = {0, 0, width, height};
    Eigen::Matrix4d id = Eigen::Matrix4d::Identity();
    for (unsigned i = 0; i < buffer_depth.size(); ++i) {
	const std::div_t coords = std::div(i, width);
	double out[3];
	gluUnProject(coords.rem, coords.quot, buffer_depth[i], id.data(), projection_matrix.data(), viewport, out + 0, out + 1, out + 2);
	for (int j = 0; j < 3; ++j)
	    point_cloud[i * 3 + j] = out[j];
	point_cloud[i * 3 + 2] *= -1.0f;
    }
    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, point_cloud.size() * sizeof(float), &point_cloud[0], GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glBindVertexArray(0);
    n_points = point_cloud.size() / 3;
}

void Camera::draw(const std::function<void()>& f) const
{
    double inv[16];
    gluInvertMatrix(modelview_matrix.data(), inv);
    glPushMatrix();
    glMultMatrixd(inv);
    glColor3fv(color.data());
    f();
    glPopMatrix();
}

void Camera::draw_cloud() const
{
    glBindVertexArray(vao[0]);
    glColor3fv(color.data());
    glDrawArrays(GL_POINTS, 0, n_points);
    glBindVertexArray(0);
}

void Camera::look_at(const double x, const double y, const double z, const double target_x, const double target_y, const double target_z, const double up_x, const double up_y, const double up_z)
{
    glPushMatrix();
    glLoadIdentity();
    gluLookAt(x, y, z, target_x, target_y, target_z, up_x, up_y, up_z);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview_matrix.data());
    glPopMatrix();
}

void Camera::set_modelview_matrix(double* m)
{
    std::copy(m, m + 16, modelview_matrix.begin());
}
