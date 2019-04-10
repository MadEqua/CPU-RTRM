#pragma once

#include <vector>

#include "Types.h"
#include "Simd.h"
#include "Camera.h"

#include <memory>


struct Sphere {
    glm::vec3 pos;
    float radius;
};

//TODO: read scene from some disk file
class Scene
{
public:
    Scene(Camera *camera);

    std::vector<Sphere> spheres;
    glm::vec3 fractalPos;

    std::unique_ptr<Camera> camera;
    glm::vec3 lightDir = {1.0f, -1.0f, 0.0f}; //TODO

    void sdf(const PointPack &pointPack, FloatPack &floatPack) const;
    float sdf(const glm::vec3 &point) const;

    void update(float dt);

private:
    void fractalSdf(const PointPack &pointPack, FloatPack &floatPack) const;
    void fractalSdf2(const PointPack &pointPack, FloatPack &floatPack) const;
    void sphereSdf(const PointPack &pointPack, FloatPack &floatPack) const;
};

