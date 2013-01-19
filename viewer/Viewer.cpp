#include <stdexcept>
#include <fstream>
#include <algorithm>
#include <functional>
#include <GL/glew.h>
#include <QKeyEvent>
#include <QMessageBox>
#include "Viewer.hpp"
#include "Model.hpp"
#include "Receiver.hpp"
#include "../virtual_camera/Camera.hpp"

Viewer::Viewer(const int w, const int h, QWidget* parent):
    QGLViewer(parent),
    width(w), height(h)/*,
    depth_stream(new std::ofstream("kinect0.depth", std::ios::binary))*/
{
    setWindowTitle("Meshified Point Cloud Viewer");
}

Viewer::~Viewer()
{}

void Viewer::init()
{
    setKeyDescription(Qt::Key_W, "Toggles wireframe");
    setKeyDescription(Qt::Key_T, "Toggles textures");
    glewInit();
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CW);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat ambient[4] = {0.2f, 0.2f, 0.2f, 1.0f};
    GLfloat diffuse[4] = {0.6f, 0.6f, 0.6f, 1.0f};
    GLfloat specular[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat position[4] = {0.0f, 4.0f, 10.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    setSceneCenter(qglviewer::Vec(0, 0, 4));
    setSceneRadius(5);
    camera()->fitSphere(qglviewer::Vec(0, 0, 4), 5);
    camera()->setFOVToFitScene();
    add_recv();
    cam.reset(new Camera("Cam0", 640, 480));
    cam->look_at(0, 0, 0, 0, 0, -1, 0, 1, 0);
}

void Viewer::draw()
{
    std::for_each(models.begin(), models.end(), std::bind(&Model::draw, std::placeholders::_1));
}

void Viewer::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_W) {
	GLint previous_polygon_mode[2];
	glGetIntegerv(GL_POLYGON_MODE, previous_polygon_mode);
	const int mode = previous_polygon_mode[0] == GL_LINE ? GL_FILL : GL_LINE;
	glPolygonMode(GL_FRONT_AND_BACK, mode);
	updateGL();
    } else if (e->key() == Qt::Key_T) {
	if (glIsEnabled(GL_TEXTURE_2D) == GL_TRUE) {
	    glDisable(GL_TEXTURE_2D);
	    glEnable(GL_LIGHTING);
	}
	else {
	    glEnable(GL_TEXTURE_2D);
	    glDisable(GL_LIGHTING);
	}
	updateGL();
    } else
	QGLViewer::keyPressEvent(e);
}

void Viewer::add_recv()
{
    models.push_back(new Model(this));
    recv.push_back(new Receiver(this, width, height, recv.size()));
    connect(recv.back(), SIGNAL(model_ready(const Data3d*)), models.back(), SLOT(load(const Data3d*)), Qt::BlockingQueuedConnection);
    connect(recv.back(), SIGNAL(model_ready(const Data3d*)), this, SLOT(shot()), Qt::BlockingQueuedConnection);
    connect(recv.back(), SIGNAL(model_ready(const Data3d*)), SLOT(updateGL()));
    connect(recv.back(), SIGNAL(finished()), SLOT(close()));
    recv.back()->start();
}

void Viewer::shot()
{
    GLint previous_polygon_mode[2];
    glGetIntegerv(GL_POLYGON_MODE, previous_polygon_mode);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    cam->render(std::bind(&Model::draw, std::cref(*models.begin())));
    glPolygonMode(GL_FRONT_AND_BACK, previous_polygon_mode[0]);
    std::vector<unsigned char> rgb;
    std::vector<float> point_cloud;
    cam->get_frame(rgb, point_cloud);
    //QImage image(&rgb[0], cam->get_width(), cam->get_height(), QImage::Format_RGB888);
    //if (image.save(QString("frame%1.png").arg(cam->get_name().c_str())) == false)
	//QMessageBox::warning(this, "Grab error", "Unable to write the snapshot");
    const int n_vertices = width * height;
    std::vector<unsigned short> depth_map(n_vertices);
    for (int i = 0; i < n_vertices; ++i)
	depth_map[i] = static_cast<unsigned short>(point_cloud[3 * i + 2] * 1000.0f);
    //depth_stream->write((const char*)depth_map.data(), n_vertices * sizeof(unsigned short));
}
