#pragma once

class StaticModel
{
public:
    StaticModel();
    ~StaticModel();
    void load(const char* fname);
    void draw();

private:
    int n_vertices;
    unsigned vbo[2];
    float modelview[16];
};
