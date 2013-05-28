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

#include <vector>

class MyPanoVertex{
public:
    MyPanoVertex() :
        x(0),
        y(0),
        z(0),
        u(0),
        v(0)
    {}

    MyPanoVertex(float _x, float _y, float _z, float _u=0, float _v=0) :
        x(_x),
        y(_y),
        z(_z),
        u(_u),
        v(_v)
    {}

    ~MyPanoVertex(){}

    /*vertex coord*/
    float x;
    float y;
    float z;
    /*text coord*/
    float u;
    float v;
};

class ModelPano
{
public:
    ModelPano();
    ModelPano(const ModelPano&);
    ~ModelPano();

    virtual void draw() const = 0;
    virtual void load(const char* texture_name) = 0;

    ModelPano& operator=(const ModelPano&);
};
