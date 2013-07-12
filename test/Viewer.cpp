#include <functional>
#include <sstream>
#include <GL/glew.h>
#include <QKeyEvent>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include "Viewer.hpp"
#include "../receiver/xvr_receiver.h"

Viewer::Viewer(const int w, const int h, QWidget* parent):
    QGLViewer(parent),
    width(w), height(h),
    doInteractiveZooming(false),
    loadStatic(false)/*,
      depth_stream(new std::ofstream("kinect0.depth", std::ios::binary))*/
{
    setWindowTitle("Meshified Point Cloud Viewer");
    bool ok;
    QStringList options;
    options.push_back(QString("Yes"));
    options.push_back(QString("No"));
    QString downsample_selected = QInputDialog::getItem(this, tr("Load static environment"),
                                                        tr("Load static environment?"), options,
                                                        0, false, &ok);

    if(downsample_selected == QString("Yes"))
    {
        loadStatic = true;


        QString fileName = QFileDialog::getOpenFileName(NULL,tr("Open PLY file"), ".", tr("*.ply"));
        QByteArray byteArray = fileName.toUtf8();
        envFname = std::string(byteArray.constData());


    }
}

Viewer::~Viewer()
{
    xvr_receiver_destroy();
}

void Viewer::init()
{
    setKeyDescription(Qt::Key_W, "Toggles wireframe");
    setKeyDescription(Qt::Key_T, "Toggles textures");
    setKeyDescription(Qt::Key_Z, "Toggles interactive zooming driven by a face tracker");
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
    setSceneRadius(15);
    camera()->fitSphere(qglviewer::Vec(0, 0, 4), 15);
    camera()->setFOVToFitScene();
    startAnimation();

    xvr_receiver_init();

    if(loadStatic)
        xvr_receiver_load_static(envFname.c_str());

    //xvr_receiver_load_panorama("pano");
}

void Viewer::draw()
{
    if(doInteractiveZooming)
    {
        float fov_h, fov_v;
        xvr_receiver_get_FOV(fov_h, fov_v);
        if(fov_h != 0.0 && fov_v != 0.0)
            camera()->setFieldOfView(fov_v);
    }
    xvr_receiver_draw();
}

void Viewer::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_W)
    {
        GLint previous_polygon_mode[2];
        glGetIntegerv(GL_POLYGON_MODE, previous_polygon_mode);
        const int mode = previous_polygon_mode[0] == GL_LINE ? GL_FILL : GL_LINE;
        glPolygonMode(GL_FRONT_AND_BACK, mode);
        updateGL();
    }
    else if (e->key() == Qt::Key_T)
    {
        if (glIsEnabled(GL_TEXTURE_2D) == GL_TRUE) {
            glDisable(GL_TEXTURE_2D);
            glEnable(GL_LIGHTING);
            this->displayMessage("Texture Disabled");
        }
        else {
            glEnable(GL_TEXTURE_2D);
            glDisable(GL_LIGHTING);
            this->displayMessage("Texture Enabled");
        }
        updateGL();
    }
    else if (e->key() == Qt::Key_Z)
    {
        doInteractiveZooming = !doInteractiveZooming;
        if(doInteractiveZooming)
            this->displayMessage("Interactive Zoom ON");
        else
            this->displayMessage("Interactive Zoom OFF");
        updateGL();
    }
    else if (e->key() == Qt::Key_C)
    {
        qglviewer::Vec camPos = camera()->position();
        std::stringstream ss;
        ss << "Camera Position: " << camPos[0] << " - " << camPos[1] << " - " << camPos[2] << std::flush;
        this->displayMessage(ss.str().c_str(),5000);
    }
    else if (e->key() == Qt::Key_P)
    {
        qglviewer::Vec camPos(-6.20844, -0.802218, -0.0460527);
        std::stringstream ss;
        ss << "Setting Camera Position to: " << camPos[0] << " - " << camPos[1] << " - " << camPos[2] << std::flush;
        this->displayMessage(ss.str().c_str(),5000);
        camera()->setPosition(camPos);
    }
    else
        QGLViewer::keyPressEvent(e);
}
