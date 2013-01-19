#include <QApplication>
#include "Viewer.hpp"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    Viewer v;
    v.show();
    v.load_model(argv[1]);
    return app.exec();
}
