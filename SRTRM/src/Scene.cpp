#include "Scene.h"

#include <limits>

void Scene::sdf(const PointPack &pointPack, FloatPack &floatPack) const {
    SimdReg min = SET_PS1(std::numeric_limits<float>().max());

    SimdReg x = LOAD_PS(pointPack.x);
    SimdReg y = LOAD_PS(pointPack.y);
    SimdReg z = LOAD_PS(pointPack.z);

    for(const auto &sphere : spheres) {
        SimdReg sphX = SET_PS1(sphere.pos.x);
        SimdReg sphY = SET_PS1(sphere.pos.y);
        SimdReg sphZ = SET_PS1(sphere.pos.z);

        //Compute distance between points and sphere
        SimdReg difX = SUB_PS(x, sphX);
        SimdReg difY = SUB_PS(y, sphY);
        SimdReg difZ = SUB_PS(z, sphZ);

        SimdReg difXSq = MUL_PS(difX, difX);
        SimdReg difYSq = MUL_PS(difY, difY);
        SimdReg difZSq = MUL_PS(difZ, difZ);
        SimdReg distanceSq = ADD_PS(difXSq, ADD_PS(difYSq, difZSq));
        SimdReg distanceToCenter = SQRT_PS(distanceSq);
        SimdReg distance = SUB_PS(distanceToCenter, SET_PS1(sphere.radius));

        SimdReg distSmallerThanMinMask = CMP_LT_PS(distance, min);
        min = BLENDV_PS(min, distance, distSmallerThanMinMask);
    }
    
    STORE_PS(floatPack, min);
}

float Scene::sdf(const glm::vec3 &point) const {
    float min = std::numeric_limits<float>().max();

    for(const auto &sphere : spheres) {
        float d = glm::distance(point, sphere.pos) - sphere.radius;
        if(d < min)
            min = d;
    }
    return min;
}
