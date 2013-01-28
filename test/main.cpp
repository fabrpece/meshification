#include <QApplication>
#include "Viewer.hpp"

int main(int argc, char** argv)
{
    const int width = argc > 1 ? std::atoi(argv[1]) : 640;
    const int height = argc > 2 ? std::atoi(argv[2]) : 480;
    QApplication app(argc, argv);
    Viewer v(width, height);
    v.show();
    return app.exec();
}
