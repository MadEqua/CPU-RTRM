#include "RendererSimd.h"

#include <glm/glm.hpp>

#include "Scene.h"
#include "Window.h"

RendererSimd::RendererSimd(const RenderSettings &renderSettings, Scene &scene, Window &window) :
    renderSettings(renderSettings),
    scene(scene),
    window(window),
    pixelCount(renderSettings.width * renderSettings.height) {

    uint32 pixelCount = renderSettings.width * renderSettings.height;
    //Add some dummy data to be a multiple of SIMD_SIZE
    while(pixelCount % SIMD_SIZE != 0) {
        pixelCount++;
    }
    data = new float[pixelCount * 3u];

    //SFML requires RGBA :(
    byteData = new byte[renderSettings.width * renderSettings.height * 4u];


    pixelToX = new float[pixelCount];
    pixelToY = new float[pixelCount];
    for(uint32 px = 0; px < pixelCount; px++) {
        //Compute pixel coordinate of the current pixel
        pixelToX[px] = static_cast<float>(px  % renderSettings.width);

        uint32 y = px / renderSettings.width;

        //The y=0 pixel should be the bottom of the image
        pixelToY[px] = static_cast<float>(renderSettings.height - y - 1);
    }

    initSimdConstants();
}

RendererSimd::~RendererSimd() {
    delete[] data;
    delete[] byteData;

    delete[] pixelToX;
    delete[] pixelToY;
}

void RendererSimd::startRenderLoop() {
    while(!window.isOpen())
        std::this_thread::yield();

    workerPool.resize(renderSettings.renderThreads);
    for(int i = 0; i < renderSettings.renderThreads; ++i) {
        workerPool[i] = std::thread(&RendererSimd::renderTask, this);
        workerPool[i].detach();
    }
    
    while(window.isOpen()) {
        float dt = frameTimer.getElapsedSeconds();
        frameTimer.start();

        {
            std::lock_guard<std::mutex> lock(mutex);
            updateData(dt);
            updateFinished = true;
            workersFinished = 0;
        }

        nextPack.store(0);
        updateFinishedCv.notify_all();

        {
            std::unique_lock<std::mutex> lock(mutex);
            renderFinishedCv.wait(lock, [this] { return workersFinished == renderSettings.renderThreads; });
        }

        //RGB float -> RGBA byte (missing srgb conversion/ tone mapping)
        for(uint32 px = 0; px < pixelCount; ++px) {
            float *src = data + (px * 3u);
            byte *dst = byteData + (px * 4u);

            *(dst + 0) = static_cast<byte>(glm::clamp(*(src + 0) * 255.0f, 0.0f, 255.0f));
            *(dst + 1) = static_cast<byte>(glm::clamp(*(src + 1) * 255.0f, 0.0f, 255.0f));
            *(dst + 2) = static_cast<byte>(glm::clamp(*(src + 2) * 255.0f, 0.0f, 255.0f));
            *(dst + 3) = 255;
        }

        window.notifyUpdate(byteData, dt);
    }
}

void RendererSimd::updateData(float dt) {
    scene.update(dt);
}

void RendererSimd::renderTask() {
    Point2Pack point2Pack;
    RayPack rayPack;
    CollisionPack collisionPack;

    int packCount = pixelCount / SIMD_SIZE;

    while(window.isOpen()) {
        //Wait for update finish to start rendering
        {
            std::unique_lock<std::mutex> lock(mutex);
            updateFinishedCv.wait(lock, [this] { return updateFinished; });
        }

        //TODO: bad traversal order for ray coherency (rays along lines will likely collide with different objects)
        //but it's a good order for writing the results to memory
        for(int currentPack = nextPack.load(); 
            currentPack < packCount; 
            currentPack = nextPack.fetch_add(1) + 1) {
            uint32 startPx = currentPack * SIMD_SIZE;

            memcpy(point2Pack.x, &pixelToX[startPx], SIMD_SIZE * sizeof(float));
            memcpy(point2Pack.y, &pixelToY[startPx], SIMD_SIZE * sizeof(float));

            scene.camera->generateRayPack(point2Pack, rayPack);

            int collisionMask = raymarch(rayPack, collisionPack);
            float *ptr = data + startPx * 3;

            ColorPack colorPack;

            if(collisionMask) {
                shadeBlinnPhong(collisionPack, colorPack);
                //shadeSteps(collisionPack, colorPack);
            }

            for(uint32 i = 0; i < SIMD_SIZE; ++i) {

                if(collisionMask & (1 << i)) {
                    /**(ptr + (i * 3) + 0) = collisionPack.normalX[i] * 0.5f + 0.5f;
                    *(ptr + (i * 3) + 1) = collisionPack.normalY[i] * 0.5f + 0.5f;
                    *(ptr + (i * 3) + 2) = collisionPack.normalZ[i] * 0.5f + 0.5f;*/

                    *(ptr + (i * 3) + 0) = colorPack.x[i];
                    *(ptr + (i * 3) + 1) = colorPack.y[i];
                    *(ptr + (i * 3) + 2) = colorPack.z[i];
                }
                else {
                    *(ptr + (i * 3) + 0) = 135.0f / 256.0f;
                    *(ptr + (i * 3) + 1) = 206.0f / 256.0f;
                    *(ptr + (i * 3) + 2) = 235.0f / 256.0f;
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex);
            workersFinished++;
            updateFinished = false;
        }
        renderFinishedCv.notify_all();
    }
}

int RendererSimd::raymarch(const RayPack &rayPack, CollisionPack &collisionPack) {

    const int ALL_ONES_MASK = (1 << SIMD_SIZE) - 1;
    int lastCollidedMask = 0;
    int collidedMask = 0;

    collisionPack = {0};

    SimdReg t = SET_ZERO_PS();
    SimdReg steps = SET_ZERO_PS();

    for(uint32 i = 0; i < renderSettings.rayMarchingSteps; ++i) {

        //Walk along the rays
        SimdReg pointX = ADD_PS(SET_PS1(rayPack.origin.x), MUL_PS(LOAD_PS(rayPack.directions.x), t));
        SimdReg pointY = ADD_PS(SET_PS1(rayPack.origin.y), MUL_PS(LOAD_PS(rayPack.directions.y), t));
        SimdReg pointZ = ADD_PS(SET_PS1(rayPack.origin.z), MUL_PS(LOAD_PS(rayPack.directions.z), t));

        PointPack pointPack;
        STORE_PS(pointPack.x, pointX);
        STORE_PS(pointPack.y, pointY);
        STORE_PS(pointPack.z, pointZ);

        FloatPack distances;
        scene.sdf(pointPack, distances);

        SimdReg dists = LOAD_PS(distances);
        SimdReg maskEpsilon = CMP_LT_PS(dists, SET_PS1(renderSettings.rayMarchingEpsilon));

        //If there is a collision (dist < epsilon) the mask will be != 0
        collidedMask = MOVE_MASK_PS(maskEpsilon);
        if(collidedMask != lastCollidedMask) {
            lastCollidedMask = collidedMask;

            STORE_PS(collisionPack.points.x, pointX);
            STORE_PS(collisionPack.points.y, pointY);
            STORE_PS(collisionPack.points.z, pointZ);

            STORE_PS(collisionPack.steps, steps);

            //If every ray has collided finish early
            if(collidedMask == ALL_ONES_MASK) {
                return ALL_ONES_MASK;
            }
        }

        //Check if there is a ray that exceeded max distance
        SimdReg maskMax = CMP_GT_PS(dists, SET_PS1(renderSettings.rayMarchingMaxDistance));
        int reachedMaxMask = MOVE_MASK_PS(maskEpsilon);

        //If every ray has exceeded maximum distance
        if(reachedMaxMask == ALL_ONES_MASK) {
            return 0;
        }

        //make sure to not move t for "finished" rays
        SimdReg combinedFinishedMask = OR_PS(maskEpsilon, maskMax);
        SimdReg maskedDists = NOT_AND_PS(combinedFinishedMask, dists);
        SimdReg maskedSteps = NOT_AND_PS(combinedFinishedMask, SET_PS1(1.0f));
        
        //Move t by the distances. Finished rays were masked out and will not change
        t = ADD_PS(t, maskedDists);
        steps = ADD_PS(steps, maskedSteps);
    }

    return collidedMask;
}

void RendererSimd::computeNormals(const PointPack &pointPack, VectorPack &outNormalPack) {
    const float EPSILON = 0.001f;

    PointPack displacedPointPack;
    FloatPack distancePack1;
    FloatPack distancePack2;

    SimdReg e = SET_PS1(EPSILON);

    //Points to compute normals
    SimdReg pointX = LOAD_PS(pointPack.x);
    SimdReg pointY = LOAD_PS(pointPack.y);
    SimdReg pointZ = LOAD_PS(pointPack.z);

    //Displace x points to compute gradient
    SimdReg pointX1 = ADD_PS(pointX, e);
    SimdReg pointX2 = SUB_PS(pointX, e);

    //Compute scene distance for displaced points
    STORE_PS(displacedPointPack.x, pointX1);
    STORE_PS(displacedPointPack.y, pointY);
    STORE_PS(displacedPointPack.z, pointZ);
    scene.sdf(displacedPointPack, distancePack1);

    STORE_PS(displacedPointPack.x, pointX2);
    STORE_PS(displacedPointPack.y, pointY);
    STORE_PS(displacedPointPack.z, pointZ);
    scene.sdf(displacedPointPack, distancePack2);

    SimdReg diffX = SUB_PS(LOAD_PS(distancePack1), LOAD_PS(distancePack2));

    //Do the same for Y and Z
    SimdReg pointY1 = ADD_PS(pointY, e);
    SimdReg pointY2 = SUB_PS(pointY, e);

    STORE_PS(displacedPointPack.x, pointX);
    STORE_PS(displacedPointPack.y, pointY1);
    STORE_PS(displacedPointPack.z, pointZ);
    scene.sdf(displacedPointPack, distancePack1);

    STORE_PS(displacedPointPack.x, pointX);
    STORE_PS(displacedPointPack.y, pointY2);
    STORE_PS(displacedPointPack.z, pointZ);
    scene.sdf(displacedPointPack, distancePack2);

    SimdReg diffY = SUB_PS(LOAD_PS(distancePack1), LOAD_PS(distancePack2));

    SimdReg pointZ1 = ADD_PS(pointZ, e);
    SimdReg pointZ2 = SUB_PS(pointZ, e);

    STORE_PS(displacedPointPack.x, pointX);
    STORE_PS(displacedPointPack.y, pointY);
    STORE_PS(displacedPointPack.z, pointZ1);
    scene.sdf(displacedPointPack, distancePack1);

    STORE_PS(displacedPointPack.x, pointX);
    STORE_PS(displacedPointPack.y, pointY);
    STORE_PS(displacedPointPack.z, pointZ2);
    scene.sdf(displacedPointPack, distancePack2);

    SimdReg diffZ = SUB_PS(LOAD_PS(distancePack1), LOAD_PS(distancePack2));

    //Normalize gradient vectors (normals) for return
    simdNormalizePack(diffX, diffY, diffZ);

    STORE_PS(outNormalPack.x, diffX);
    STORE_PS(outNormalPack.y, diffY);
    STORE_PS(outNormalPack.z, diffZ);
}

void RendererSimd::shadeBlinnPhong(const CollisionPack &collisionPack, ColorPack &outColorPack) {
    
    VectorPack normalPack;
    computeNormals(collisionPack.points, normalPack);

    SimdReg Lx = SET_PS1(-scene.lightDir.x);
    SimdReg Ly = SET_PS1(-scene.lightDir.y);
    SimdReg Lz = SET_PS1(-scene.lightDir.z);
    simdNormalizePack(Lx, Ly, Lz);

    SimdReg Nx = LOAD_PS(normalPack.x);
    SimdReg Ny = LOAD_PS(normalPack.y);
    SimdReg Nz = LOAD_PS(normalPack.z);

    SimdReg diffuseTerm = MAX_PS(SET_ZERO_PS(), simdDotPack(Nx, Ny, Nz, Lx, Ly, Lz));

    //TODO: colors hardcoded
    SimdReg difR = MUL_PS(SET_PS1(0.6f), diffuseTerm);
    SimdReg difG = MUL_PS(SET_PS1(1.0f), diffuseTerm);
    SimdReg difB = MUL_PS(SET_PS1(0.0f), diffuseTerm);

    auto cameraPos = scene.camera->getPosition();
    SimdReg Vx = SUB_PS(SET_PS1(cameraPos.x), LOAD_PS(collisionPack.points.x));
    SimdReg Vy = SUB_PS(SET_PS1(cameraPos.y), LOAD_PS(collisionPack.points.y));
    SimdReg Vz = SUB_PS(SET_PS1(cameraPos.z), LOAD_PS(collisionPack.points.z));
    simdNormalizePack(Vx, Vy, Vz);

    SimdReg Hx = ADD_PS(Lx, Vx);
    SimdReg Hy = ADD_PS(Ly, Vy);
    SimdReg Hz = ADD_PS(Lz, Vz);
    simdNormalizePack(Hx, Hy, Hz);

    SimdReg specularTerm = MAX_PS(SET_ZERO_PS(), simdDotPack(Nx, Ny, Nz, Hx, Hy, Hz));
    specularTerm = simdPow(specularTerm, SET_PS1(50.0f));

    //TODO: colors hardcoded
    SimdReg specR = MUL_PS(SET_PS1(1.0f), specularTerm);
    SimdReg specG = MUL_PS(SET_PS1(1.0f), specularTerm);
    SimdReg specB = MUL_PS(SET_PS1(1.0f), specularTerm);

    STORE_PS(outColorPack.x, ADD_PS(difR, specR));
    STORE_PS(outColorPack.y, ADD_PS(difG, specG));
    STORE_PS(outColorPack.z, ADD_PS(difB, specB));
}

void RendererSimd::shadeSteps(const CollisionPack &collisionPack, ColorPack &outColorPack) {
    SimdReg steps = LOAD_PS(collisionPack.steps);
    SimdReg color = DIV_PS(steps, SET_PS1(static_cast<float>(renderSettings.rayMarchingSteps)));
    STORE_PS(outColorPack.x, color);
    STORE_PS(outColorPack.y, color);
    STORE_PS(outColorPack.z, color);
}
