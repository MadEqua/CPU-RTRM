#include "Scene.h"

#include <limits>

Scene::Scene(Camera *camera) :
    camera(camera) {
}

void Scene::sdf(const PointPack &pointPack, FloatPack &floatPack) const {
    /*SimdReg min = SET_PS1(std::numeric_limits<float>().max());

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
    
    STORE_PS(floatPack, min);*/

    fractalSdf(pointPack, floatPack);
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

 void Scene::fractalSdf(const PointPack &pointPack, FloatPack &floatPack) const {
    const int ITERATIONS = 6;
    const float SCALE = 2.0f;
    
    SimdReg x = LOAD_PS(pointPack.x);
    SimdReg y = LOAD_PS(pointPack.y);
    SimdReg z = LOAD_PS(pointPack.z);

    SimdReg one = SET_PS1(1.0f);
    SimdReg negOne = SET_PS1(-1.0f);

    SimdReg scale = SET_PS1(SCALE);
    SimdReg scaleMinOne = SUB_PS(scale, one);

    SimdReg scalePowAcum = one;

    int n = 0;
    while(n < ITERATIONS) {

        SimdReg cX = one;
        SimdReg cY = one;
        SimdReg cZ = one;

        SimdReg dist = simdSqLengthPack(SUB_PS(x, one), SUB_PS(y, one), SUB_PS(z, one));

        SimdReg d = simdSqLengthPack(SUB_PS(x, negOne), SUB_PS(y, negOne), SUB_PS(z, one));
        SimdReg dLtDistMask = CMP_LT_PS(d, dist);
        cX = BLENDV_PS(cX, negOne, dLtDistMask);
        cY = BLENDV_PS(cY, negOne, dLtDistMask);
        cZ = BLENDV_PS(cZ, one, dLtDistMask);
        dist = BLENDV_PS(dist, d, dLtDistMask);

        d = simdSqLengthPack(SUB_PS(x, one), SUB_PS(y, negOne), SUB_PS(z, negOne));
        dLtDistMask = CMP_LT_PS(d, dist);
        cX = BLENDV_PS(cX, one, dLtDistMask);
        cY = BLENDV_PS(cY, negOne, dLtDistMask);
        cZ = BLENDV_PS(cZ, negOne, dLtDistMask);
        dist = BLENDV_PS(dist, d, dLtDistMask);

        d = simdSqLengthPack(SUB_PS(x, negOne), SUB_PS(y, one), SUB_PS(z, negOne));
        dLtDistMask = CMP_LT_PS(d, dist);
        cX = BLENDV_PS(cX, negOne, dLtDistMask);
        cY = BLENDV_PS(cY, one, dLtDistMask);
        cZ = BLENDV_PS(cZ, negOne, dLtDistMask);
        dist = BLENDV_PS(dist, d, dLtDistMask);

        x = SUB_PS(MUL_PS(scale, x), MUL_PS(cX, scaleMinOne));
        y = SUB_PS(MUL_PS(scale, y), MUL_PS(cY, scaleMinOne));
        z = SUB_PS(MUL_PS(scale, z), MUL_PS(cZ, scaleMinOne));

        n++;
        scalePowAcum = MUL_PS(scalePowAcum, scale);
    }

    SimdReg res = MUL_PS(fractalShape(x, y, z), DIV_PS(one, scalePowAcum));
    STORE_PS(floatPack, res);
}

void Scene::fractalSdf2(const PointPack &pointPack, FloatPack &floatPack) const {
    const int ITERATIONS = 8;
    const float SCALE = 2.0f;

    SimdReg x = LOAD_PS(pointPack.x);
    SimdReg y = LOAD_PS(pointPack.y);
    SimdReg z = LOAD_PS(pointPack.z);

    SimdReg one = SET_PS1(1.0f);
    SimdReg negOne = SET_PS1(-1.0f);
    SimdReg zero = SET_ZERO_PS();

    SimdReg scale = SET_PS1(SCALE);
    SimdReg scaleMinOne = SUB_PS(scale, one);

    SimdReg offset = MUL_PS(one, scaleMinOne);

    SimdReg scalePowAcum = one;

    int n = 0;
    while(n < ITERATIONS) {

        SimdReg xPlusY = ADD_PS(x, y);
        SimdReg mask = CMP_LT_PS(xPlusY, zero);
        SimdReg temp = x;
        x = BLENDV_PS(x, MUL_PS(negOne, y), mask);
        y = BLENDV_PS(y, MUL_PS(negOne, temp), mask);

        SimdReg xPlusZ = ADD_PS(x, z);
        mask = CMP_LT_PS(xPlusZ, zero);
        temp = x;
        x = BLENDV_PS(x, MUL_PS(negOne, z), mask);
        z = BLENDV_PS(z, MUL_PS(negOne, temp), mask);

        SimdReg yPlusZ = ADD_PS(y, z);
        mask = CMP_LT_PS(yPlusZ, zero);
        temp = y;
        y = BLENDV_PS(y, MUL_PS(negOne, z), mask);
        z = BLENDV_PS(z, MUL_PS(negOne, temp), mask);

        x = SUB_PS(MUL_PS(scale, x), offset);
        y = SUB_PS(MUL_PS(scale, y), offset);
        z = SUB_PS(MUL_PS(scale, z), offset);

        n++;
        scalePowAcum = MUL_PS(scalePowAcum, scale);
    }

    SimdReg res = MUL_PS(fractalShape(x, y, z), DIV_PS(one, scalePowAcum));
    STORE_PS(floatPack, res);
}

SimdReg Scene::fractalShape(SimdReg x, SimdReg y, SimdReg z) const {
    static SimdReg invSqrtThree = DIV_PS(SET_PS1(1.0f), SQRT_PS(SET_PS1(3.0f)));

    SimdReg negOne = SET_PS1(-1.0f);
    SimdReg xNeg = MUL_PS(x, negOne);
    SimdReg yNeg = MUL_PS(y, negOne);
    SimdReg zNeg = MUL_PS(z, negOne);

    SimdReg max1 = MAX_PS(ADD_PS(xNeg, ADD_PS(yNeg, zNeg)),
                          ADD_PS(x, ADD_PS(y, zNeg)));
    SimdReg max2 = MAX_PS(ADD_PS(xNeg, ADD_PS(y, z)),
                          ADD_PS(x, ADD_PS(yNeg, z)));
    SimdReg max = MAX_PS(max1, max2);
    SimdReg res = SUB_PS(max, SET_PS1(1.0f));
    return MUL_PS(invSqrtThree, res);
}

void Scene::sphereSdf(const PointPack &pointPack, FloatPack &floatPack) const {
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

void Scene::update(float dt) {
    camera->update(dt);
}