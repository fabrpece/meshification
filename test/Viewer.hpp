#pragma once
#include <QGLViewer/qglviewer.h>


class Viewer : public QGLViewer
{
    Q_OBJECT;

    const int width, height;
    bool doInteractiveZooming;
    bool loadStatic;
    std::string envFname;

    void init();
    void draw();
    void keyPressEvent(QKeyEvent* e);

public:
    Viewer(const int w, const int h, QWidget* parent = 0);
    ~Viewer();
};
