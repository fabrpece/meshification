#pragma once

class StaticModel
{
public:
    StaticModel();
    ~StaticModel();
    void load(const char* fname);
    void draw() const;
    void tooglePointSmooth();

private:
    int n_vertices;
    unsigned vao[1], vbo[2];
    float modelview[16];
    bool use_point_smooth;
};
