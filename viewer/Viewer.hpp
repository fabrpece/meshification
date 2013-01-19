#pragma once
#include <memory>
#include <iosfwd>
#include <QGLViewer/qglviewer.h>

class Model;
class Receiver;
class Camera;

class Viewer : public QGLViewer
{
    Q_OBJECT;

    const int width, height;

    std::vector<Receiver*> recv;
    std::vector<Model*> models;

    std::unique_ptr<Camera> cam;
    std::unique_ptr<std::ostream> depth_stream;

    void init();
    void draw();
    void keyPressEvent(QKeyEvent* e);

    void add_recv();

public:
    Viewer(const int w, const int h, QWidget* parent = 0);
    ~Viewer();

public slots:
    void shot();
};
