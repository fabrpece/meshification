#pragma once
#include <memory>
#include <iosfwd>
#include <QGLViewer/qglviewer.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Viewer : public QGLViewer
{
    Q_OBJECT;

    const int width, height;

    void init();
    void draw();
    void keyPressEvent(QKeyEvent* e);
    void loadCloud();
    void fillVBOs(bool isBGR=false);
    void enablePointSprite();
    void disablePointSprite();
    void getAllOptions();

    unsigned vao[1], vbo[2];
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    int n_vertices;
    double voxelGridLeafSize;
    int pclCompressionType;
    bool toDownsample;
    bool drawSprite;
    bool doPointSmooth;
    double modelview[16];

public:

    Viewer(const int w, const int h, QWidget* parent = 0);
    ~Viewer();
};
