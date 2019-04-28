#include "Scene.h"

#include <limits>

Scene::Scene(Camera *camera) :
    camera(camera) {
    initSimdConstants();
}

void Scene::sdf(const PointPack &pointPack, FloatPack &floatPack) const {
    PointPack modPointPack;
    repeatOperator(pointPack, glm::vec3(4.0f), modPointPack);
    
    FloatPack cubeSdfPack;
    cubeSdf(modPointPack, glm::vec3(1.0f), 0.1f, cubeSdfPack);

    FloatPack sphereSdfPack;
    sphereSdf(modPointPack, 1.3f, sphereSdfPack);

    SimdReg c = LOAD_PS(cubeSdfPack);
    SimdReg s = LOAD_PS(sphereSdfPack);
    SimdReg sdf = MIN_PS(c, s);
    STORE_PS(floatPack, sdf);
}

void Scene::update(float dt) {
    camera->update(dt);
    lightPos = camera->getPosition();
}

void Scene::fractalSdf(const PointPack &pointPack, FloatPack &floatPack) const {
    const int ITERATIONS = 3;

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

    for(int i = 0; i < ITERATIONS; ++i) {
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

        scalePowAcum = MUL_PS(scalePowAcum, scale);
    }

    SimdReg shape = fractalShape(x, y, z);
    SimdReg res = MUL_PS(shape, DIV_PS(one, scalePowAcum));
    STORE_PS(floatPack, res);
}

SimdReg Scene::fractalShape(SimdReg x, SimdReg y, SimdReg z) const{
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

void Scene::sphereSdf(const PointPack &pointPack, float radius, FloatPack &floatPack) const {
    SimdReg x = LOAD_PS(pointPack.x);
    SimdReg y = LOAD_PS(pointPack.y);
    SimdReg z = LOAD_PS(pointPack.z);

    SimdReg distanceToCenter = simdLength(x, y, z);
    SimdReg distance = SUB_PS(distanceToCenter, SET_PS1(radius));

    STORE_PS(floatPack, distance);
}

void Scene::cubeSdf(const PointPack &pointPack, const glm::vec3 size, float roundness, FloatPack &floatPack) const {
    SimdReg dx = SUB_PS(simdAbs(LOAD_PS(pointPack.x)), SET_PS1(size.x));
    SimdReg dy = SUB_PS(simdAbs(LOAD_PS(pointPack.y)), SET_PS1(size.y));
    SimdReg dz = SUB_PS(simdAbs(LOAD_PS(pointPack.z)), SET_PS1(size.z));

    SimdReg zero = SET_ZERO_PS();
    SimdReg xMaxZero = MAX_PS(dx, zero);
    SimdReg yMaxZero = MAX_PS(dy, zero);
    SimdReg zMaxZero = MAX_PS(dz, zero);
    SimdReg length = simdLength(xMaxZero, yMaxZero, zMaxZero);
    SimdReg round = SUB_PS(length, SET_PS1(roundness));

    SimdReg max = MIN_PS(zero, MAX_PS(dx, MAX_PS(dy, dz)));
    
    STORE_PS(floatPack, ADD_PS(max, round));
}


void Scene::repeatOperator(const PointPack &in, const glm::vec3 &period, PointPack &out) const {
    //vec3 q = mod(p, period) - 0.5 * period;

    SimdReg periodX = SET_PS1(period.x);
    SimdReg periodY = SET_PS1(period.y);
    SimdReg periodZ = SET_PS1(period.z);

    SimdReg modX = simdMod(LOAD_PS(in.x), periodX);
    SimdReg modY = simdMod(LOAD_PS(in.y), periodY);
    SimdReg modZ = simdMod(LOAD_PS(in.z), periodZ);

    SimdReg half = SET_PS1(0.5f);
    periodX = MUL_PS(periodX, half);
    periodY = MUL_PS(periodY, half);
    periodZ = MUL_PS(periodZ, half);

    STORE_PS(out.x, SUB_PS(modX, periodX));
    STORE_PS(out.y, SUB_PS(modY, periodY));
    STORE_PS(out.z, SUB_PS(modZ, periodZ));
}