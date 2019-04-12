#include "Scene.h"

#include <limits>

Scene::Scene(Camera *camera) :
    camera(camera) {
    initSimdConstants();
}

void Scene::sdf(const PointPack &pointPack, FloatPack &floatPack) const {
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

/*void Scene::fractalSdf(const PointPack &pointPack, FloatPack &floatPack) const {
    const int ITERATIONS = 6;

    //Huge performance boost by prefetching the constants block
    PREFETCH(reinterpret_cast<char*>(&SIMD_CONSTANTS), _MM_HINT_T0);
    
    SimdReg x = LOAD_PS(pointPack.x);
    SimdReg y = LOAD_PS(pointPack.y);
    SimdReg z = LOAD_PS(pointPack.z);

    SimdReg zero = SET_ZERO_PS();
    SimdReg negOne = LOAD_PS(&SIMD_CONSTANTS[0]);
    SimdReg one = LOAD_PS(&SIMD_CONSTANTS[SIMD_SIZE]);

    SimdReg scale = LOAD_PS(&SIMD_CONSTANTS[SIMD_SIZE * 2]);
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

        x = SUB_PS(MUL_PS(scale, x), cX);
        y = SUB_PS(MUL_PS(scale, y), cY);
        z = SUB_PS(MUL_PS(scale, z), cZ);

        n++;
        scalePowAcum = MUL_PS(scalePowAcum, scale);
    }

    SimdReg res = MUL_PS(fractalShape(x, y, z), DIV_PS(one, scalePowAcum));
    STORE_PS(floatPack, res);
}*/

void Scene::fractalSdf(const PointPack &pointPack, FloatPack &floatPack) const {
    const int ITERATIONS = 4;

    //Huge performance boost by prefetching the constants block
    PREFETCH_T0(&SIMD_CONSTANTS);

    SimdReg x = LOAD_PS(pointPack.x);
    SimdReg y = LOAD_PS(pointPack.y);
    SimdReg z = LOAD_PS(pointPack.z);

    SimdReg zero = SET_ZERO_PS();
    SimdReg negOne = LOAD_PS(&SIMD_CONSTANTS[0]);
    SimdReg one = LOAD_PS(&SIMD_CONSTANTS[SIMD_SIZE]);

    SimdReg scale = LOAD_PS(&SIMD_CONSTANTS[SIMD_SIZE * 2]);
    SimdReg scalePowAcum = one;

    int n = 0;
    while(n < ITERATIONS) {
        SimdReg mask = CMP_LT_PS(ADD_PS(x, y), zero);
        SimdReg temp = x;
        x = BLENDV_PS(x, MUL_PS(negOne, y), mask);
        y = BLENDV_PS(y, MUL_PS(negOne, temp), mask);

        mask = CMP_LT_PS(ADD_PS(x, z), zero);
        temp = x;
        x = BLENDV_PS(x, MUL_PS(negOne, z), mask);
        z = BLENDV_PS(z, MUL_PS(negOne, temp), mask);

        mask = CMP_LT_PS(ADD_PS(y, z), zero);
        temp = y;
        y = BLENDV_PS(y, MUL_PS(negOne, z), mask);
        z = BLENDV_PS(z, MUL_PS(negOne, temp), mask);

        x = SUB_PS(MUL_PS(scale, x), one);
        y = SUB_PS(MUL_PS(scale, y), one);
        z = SUB_PS(MUL_PS(scale, z), one);

        n++;
        scalePowAcum = MUL_PS(scalePowAcum, scale);
    }

    SimdReg shape = fractalShape(x, y, z);
    SimdReg res = MUL_PS(shape, DIV_PS(one, scalePowAcum));
    STORE_PS(floatPack, res);
}

SimdReg Scene::fractalShape(SimdReg x, SimdReg y, SimdReg z) const {
    SimdReg negOne = LOAD_PS(&SIMD_CONSTANTS[0]);
    SimdReg one = LOAD_PS(&SIMD_CONSTANTS[SIMD_SIZE]);
    SimdReg invSqrtThree = LOAD_PS(&SIMD_CONSTANTS[SIMD_SIZE * 3]);

    SimdReg xNeg = MUL_PS(x, negOne);
    SimdReg yNeg = MUL_PS(y, negOne);
    SimdReg zNeg = MUL_PS(z, negOne);

    SimdReg max1 = MAX_PS(ADD_PS(xNeg, ADD_PS(yNeg, zNeg)),
                          ADD_PS(x, ADD_PS(y, zNeg)));
    SimdReg max2 = MAX_PS(ADD_PS(xNeg, ADD_PS(y, z)),
                          ADD_PS(x, ADD_PS(yNeg, z)));
    SimdReg max = MAX_PS(max1, max2);
    SimdReg res = SUB_PS(max, one);
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