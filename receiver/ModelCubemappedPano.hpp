#pragma once

class ModelCubemappedPano
{
public:
    ModelCubemappedPano(float _radius = 10.0f);
    ~ModelCubemappedPano();
    void draw() const;
    void load(const char* cubemap_folder); /*folder we will use to load the 6 pano images*/

private:
    float radius;
    static const int n_elements = 6;
    static const int n_vbo = 3;
    unsigned vao[1], vbo[n_vbo], tex[1];
    std::vector<unsigned> vertex_indices;
    std::vector<float> tris;
    std::vector<float> tex_coord;
    float model_matrix[16];

    void drawCubeManual() const;
    void prepareCubeMap();
};
