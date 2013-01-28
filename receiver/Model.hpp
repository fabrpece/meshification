#pragma once

class Data3d;

class Model
{
    static const int n_vbo = 4;
    unsigned vao[1], vbo[n_vbo], tex[1];
    size_t n_elements = 0;
    float model_matrix[16];

    Model(const Model&);
    Model& operator=(const Model&);

public:
    Model();
    ~Model();
    static void init();
    void draw() const;
    void load(const Data3d& data);
};
