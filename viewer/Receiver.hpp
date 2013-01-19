#include <vector>
#include <QThread>
#include "Data3d.hpp"

class Receiver : public QThread
{
    Q_OBJECT;
    const int id;
    bool is_running;
    Data3d data;
    const int width, height;

protected:
    void run();

public:
    Receiver(QObject*, const int w, const int h, const int id = 0);
    ~Receiver();
    const std::vector<unsigned>& indices() const {
	return data.tri;
    }
    const std::vector<float>& coords() const {
	return data.ver;
    }
    const std::vector<float>& tex_coords() const {
	return data.tex;
    }
    const std::vector<unsigned char>& image() const {
	return data.bgr;
    }

public slots:
    void stop();

signals:
    void model_ready(const Data3d* data);
};
