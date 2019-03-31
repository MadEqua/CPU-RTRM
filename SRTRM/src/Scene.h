#pragma once

#include <vector>

#include "Types.h"
#include "Simd.h"


struct Sphere {
    glm::vec3 pos;
    float radius;
};

//TODO: yeah...
struct Camera {
    glm::vec3 pos = {0.0f, 0.0f, 2.0f};
    glm::vec3 lookAt = {0.0f, 0.0f, 0.0f};
    glm::vec3 up = {0.0f, 1.0f, 0.0f};
};

class Scene
{
public:
    std::vector<Sphere> spheres;
    Camera camera;
    glm::vec3 lightDir = {1.0f, -1.0f, 0.0f}; //TODO

    void sdf(const PointPack &pointPack, float distances[]) const;
    float sdf(const glm::vec3 &point) const;
};

