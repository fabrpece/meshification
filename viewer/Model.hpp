#pragma once
#include <vector>
#include <iosfwd>
#include <string>
#include <QObject>

class Data3d;

class Model : public QObject
{
    Q_OBJECT;
    static const int n_vbo = 4;
    unsigned vao[1], vbo[n_vbo], tex[1];
    size_t n_elements;
    float model_matrix[16];

    void load_off(std::istream& in, std::vector<float>& ver, std::vector<unsigned>& tri);

    Model(const Model&);
    Model& operator=(const Model&);

public:
    Model(QObject* parent = 0);
    ~Model();
    void draw() const;

public slots:
    void load(const Data3d* data);
};
