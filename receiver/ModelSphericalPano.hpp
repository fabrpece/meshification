#pragma once

/*
    Copyright (C) 2011-2013 Fabrizio Pece <fabrpece@cs.ucl.ac.uk>

    This file is part of meshificator.

    meshificator is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    meshificator is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with meshificator.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ModelPano.hpp"

class ModelSphericalPano : public ModelPano
{
public:
    ModelSphericalPano(double _radius = 5.0, double _slices = 500.0, double _stack = 500.0);
    ModelSphericalPano(const ModelPano&);
    ~ModelSphericalPano();

    void draw() const;
    void load(const char* texture_name);

    ModelSphericalPano& operator=(const ModelPano&);

private:
    int n_elements;
    static const int n_vbo = 3;
    unsigned vao[1], vbo[n_vbo], tex[1];
    float model_matrix[16];
    std::vector<MyPanoVertex> vertices;
    std::vector<unsigned> vertex_indices;
    std::vector<float> tris;
    std::vector<float> tex_coord;
    int textureImg_w;
    int textureImg_h;
    float radius;
    float slices;
    float stacks;

    void prepareSphereManual();
    void drawSphereDirectRendering() const;
    MyPanoVertex getPoint(double phi, double theta) const;
};
