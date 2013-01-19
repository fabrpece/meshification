#include <memory>
#include <fstream>
#include <QGLViewer/qglviewer.h>
class Camera;

class Viewer : public QGLViewer
{
    Q_OBJECT
    static const unsigned int vao_size = 2;
    static const unsigned int vbo_size = 4;
    unsigned vao[vao_size];
    unsigned vbo[vbo_size];
    int n_elements;
    int n_cam_elements;
    std::unique_ptr<Camera> cam;
    std::ofstream depth_stream, color_stream;

    virtual void init();
    virtual void draw();
    virtual void keyPressEvent(QKeyEvent* e);

    void draw_model() const;
    void draw_camera() const;

public:
    Viewer(QWidget* parent = 0);
    ~Viewer();

public slots:
    void load_model(const QString& filename);
    void shot();
};
