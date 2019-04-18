#pragma once

#include <vector>

#include "Types.h"
#include "Simd.h"
#include "Camera.h"

#include <glm/glm.hpp>

#include <memory>
#include <functional>


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

    std::unique_ptr<Camera> camera;
    glm::vec3 lightDir = {0.5f, -1.0f, 0.0f}; //TODO

    void sdf(const PointPack &pointPack, FloatPack &floatPack) const;
    float sdf(const glm::vec3 &point) const;

    void update(float dt);

private:
    void fractalSdf(const PointPack &pointPack, FloatPack &floatPack) const;
    SimdReg fractalShape(SimdReg x, SimdReg y, SimdReg z) const;

    void sphereSdf(const PointPack &pointPack, FloatPack &floatPack) const;

    using SceneSdfType = void(const Scene*, const PointPack&, FloatPack&);
    void repeatScene(const PointPack &in, const glm::vec3 &period, FloatPack &out, const std::function<SceneSdfType> &sceneSdf) const;
};

