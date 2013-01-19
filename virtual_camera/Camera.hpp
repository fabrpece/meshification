#pragma once
#include <functional>
#include <vector>
#include <array>
#include <string>

class Camera
{
    std::string name;
    std::array<double, 16> modelview_matrix, projection_matrix;
    const int width, height;

    std::array<unsigned, 1> fbo;
    std::array<unsigned, 2> rbo;
    std::array<unsigned, 1> vao;
    std::array<unsigned, 1> vbo;
    int n_points;

    std::array<float, 3> color;

public:
    Camera(const std::string& name, const int w, const int h);
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
    ~Camera();
    void render(const std::function<void()>& f);
    void get_frame(std::vector<unsigned char>& rgb, std::vector<float>& point_cloud);
    int get_width() const { return width; }
    int get_height() const { return height; }
    int size() const { return width * height; }
    std::string get_name() const { return name; }
    void look_at(const double x, const double y, const double z, const double target_x, const double target_y, const double target_z, const double up_x, const double up_y, const double up_z);
    void set_modelview_matrix(double* m);
    void set_color(const float r, const float g, const float b) { color = std::array<float, 3>{r, g, b}; }
    void draw(const std::function<void()>& f) const;
    void draw_cloud() const;
};
