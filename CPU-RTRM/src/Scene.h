#pragma once

#include <vector>

#include "Types.h"
#include "Simd.h"
#include "Camera.h"

#include <glm/glm.hpp>

#include <memory>


struct Sphere {
    glm::vec3 pos;
    float radius;
};

class Scene
{
public:
    Scene(Camera *camera);

    std::unique_ptr<Camera> camera;
    glm::vec3 lightPos;

    void sdf(const PointPack &pointPack, FloatPack &floatPack) const;

    void update(float dt);

private:
    void fractalSdf(const PointPack &pointPack, FloatPack &floatPack) const;
    SimdReg fractalShape(SimdReg x, SimdReg y, SimdReg z) const;

    void sphereSdf(const PointPack &pointPack, float radius, FloatPack &floatPack) const;
    void cubeSdf(const PointPack &pointPack, const glm::vec3 size, float roundness, FloatPack &floatPack) const;

    void repeatOperator(const PointPack &in, const glm::vec3 &period, PointPack &out) const;
};

