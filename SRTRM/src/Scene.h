#pragma once

#include <vector>

#include "Types.h"
#include "Simd.h"
#include "Camera.h"


struct Sphere {
    glm::vec3 pos;
    float radius;
};

class Scene
{
public:
    std::vector<Sphere> spheres;
    glm::vec3 fractalPos;

    Camera camera;
    glm::vec3 lightDir = {1.0f, -1.0f, 0.0f}; //TODO

    void sdf(const PointPack &pointPack, FloatPack &floatPack) const;
    float sdf(const glm::vec3 &point) const;

    void update(float dt);

private:
    void fractalSdf(const PointPack &pointPack, FloatPack &floatPack) const;
    void sphereSdf(const PointPack &pointPack, FloatPack &floatPack) const;
};

