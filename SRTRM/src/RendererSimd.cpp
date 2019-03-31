#include "RendererSimd.h"

#include <chrono>

#include <glm/glm.hpp>

#include "Scene.h"
#include "Window.h"


RendererSimd::RendererSimd(const RenderSettings &renderSettings, Scene &scene, Window &window) :
    renderSettings(renderSettings),
    scene(scene),
    window(window) {

    uint32 pixelCount = renderSettings.width * renderSettings.height;
    //Add some dummy data to be a multiple of SIMD_SIZE
    while(pixelCount % SIMD_SIZE != 0) {
        pixelCount++;
    }
    data = new float[pixelCount * 3u];

    //SFML requires RGBA :(
    byteData = new byte[renderSettings.width * renderSettings.height * 4u];
}

RendererSimd::~RendererSimd() {
    delete[] data;
    delete[] byteData;
}

void RendererSimd::startRenderLoop() {
    while(!window.isOpen())
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    while(window.isOpen()) {
        float dt = frameTimer.getElapsedSeconds();
        frameTimer.start();
        updateData(dt);
        renderFrame(dt);
        window.notifyUpdate(byteData, dt);
    }
}

void RendererSimd::updateData(float dt) {
    static float elapsed = 0.0f;
    elapsed += dt;

    scene.lightDir.x = dt * glm::sin(elapsed * 2.0f);
    scene.lightDir.z = dt * glm::cos(elapsed * 2.0f);
}

void RendererSimd::renderFrame(float dt) {
    uint32 pixelCount = renderSettings.width * renderSettings.height;

    float xStep = 1.0f / static_cast<float>(renderSettings.width);
    float yStep = 1.0f / static_cast<float>(renderSettings.height);

    RayPack rayPack;
    rayPack.origin = scene.camera.pos;

    CollisionPack collisionPack;

    //TODO: bad traversal order for ray coherency (rays along lines will likely collide with different objects)
    //but it's a good order for writing the results to memory
    for(uint32 px = 0; px < pixelCount; px += SIMD_SIZE) {

        uint32 x[SIMD_SIZE];
        uint32 invertedY[SIMD_SIZE];

        //TODO: is this performant enough?
        for(uint32 i = 0; i < SIMD_SIZE; ++i) {
            //Compute pixel coordinate of the current pixel
            x[i] = (px + i) % renderSettings.width;
            
            uint32 y = (px + i) / renderSettings.width;

            //The y=0 pixel should be the bottom of the image
            invertedY[i] = renderSettings.height - y - 1;
        }

        //Transform pixel coords to [0, 1] and then map [-1, 1]
        SimdRegi dirXi = LOAD_SI(reinterpret_cast<const SimdRegi*>(x));
        SimdReg dirX = CVT_I_PS(dirXi);
        dirX = ADD_PS(dirX, SET_PS1(0.5f));
        dirX = MUL_PS(dirX, SET_PS1(xStep));
        dirX = MUL_PS(dirX, SET_PS1(2.0f));
        dirX = SUB_PS(dirX, SET_PS1(1.0f));

        SimdRegi dirYi = LOAD_SI(reinterpret_cast<const SimdRegi*>(invertedY));
        SimdReg dirY = CVT_I_PS(dirYi);
        dirY = ADD_PS(dirY, SET_PS1(0.5f));
        dirY = MUL_PS(dirY, SET_PS1(yStep));
        dirY = MUL_PS(dirY, SET_PS1(2.0f));
        dirY = SUB_PS(dirY, SET_PS1(1.0f));

        SimdReg dirZ = SET_PS1(-1.0f);
        simdNormalizePack(dirX, dirY, dirZ);
        STORE_PS(rayPack.dirX, dirX);
        STORE_PS(rayPack.dirY, dirY);
        STORE_PS(rayPack.dirZ, dirZ);


        int collisionMask = raymarch(rayPack, collisionPack);
        float *ptr = data + px * 3;

        //TODO: is this performant enough?
        for(uint32 i = 0; i < SIMD_SIZE; ++i) {

            if(collisionMask & (1 << i)) {
                /**(ptr + (i * 3) + 0) = collisionPack.normalX[i] * 0.5f + 0.5f;
                *(ptr + (i * 3) + 1) = collisionPack.normalY[i] * 0.5f + 0.5f;
                *(ptr + (i * 3) + 2) = collisionPack.normalZ[i] * 0.5f + 0.5f;*/

                ColorPack colorPack;
                shade(collisionPack, colorPack);

                *(ptr + (i * 3) + 0) = colorPack.x[i];
                *(ptr + (i * 3) + 1) = colorPack.y[i];
                *(ptr + (i * 3) + 2) = colorPack.z[i];
            }
            else {
                *(ptr + (i * 3) + 0) = 0.5f;
                *(ptr + (i * 3) + 1) = 0.5f;
                *(ptr + (i * 3) + 2) = 0.5f;
            }
        }
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
}

int RendererSimd::raymarch(const RayPack &rayPack, CollisionPack &collisionPack) {
    
    SimdReg t = SET_ZERO_PS();
    SimdReg rayOriginX = SET_PS1(rayPack.origin.x);
    SimdReg rayOriginY = SET_PS1(rayPack.origin.y);
    SimdReg rayOriginZ = SET_PS1(rayPack.origin.z);

    const int ALL_ONES_MASK = (1 << SIMD_SIZE) - 1;
    int collidedMask = 0;

    collisionPack = {0};

    for(uint32 i = 0; i < renderSettings.rayMarchingSteps; ++i) {
       
        //Walk along the rays
        SimdReg pointX = ADD_PS(rayOriginX, MUL_PS(LOAD_PS(rayPack.dirX), t));
        SimdReg pointY = ADD_PS(rayOriginY, MUL_PS(LOAD_PS(rayPack.dirY), t));
        SimdReg pointZ = ADD_PS(rayOriginZ, MUL_PS(LOAD_PS(rayPack.dirZ), t));

        PointPack pointPack;
        STORE_PS(pointPack.x, pointX);
        STORE_PS(pointPack.y, pointY);
        STORE_PS(pointPack.z, pointZ);

        FloatPack distances;
        scene.sdf(pointPack, distances);

        SimdReg dists = LOAD_PS(distances);
        SimdReg epsilon = SET_PS1(renderSettings.rayMarchingEpsilon);
        SimdReg maskEpsilon = CMP_LT_PS(dists, epsilon);

        //If there is a collision (dist < epsilon) the mask will be != 0
        //Using a branch here because we really want to avoid computing normals when we can
        collidedMask = MOVE_MASK_PS(maskEpsilon);
        if(collidedMask) {
            STORE_PS(collisionPack.pointX, pointX);
            STORE_PS(collisionPack.pointY, pointY);
            STORE_PS(collisionPack.pointZ, pointZ);

            computeNormals(pointPack, collisionPack);

            //If every ray has collided finish early
            if(collidedMask == ALL_ONES_MASK) {
                return ALL_ONES_MASK;
            }
        }

        //Check if there is a ray that exceeded max distance
        SimdReg maxDist = SET_PS1(renderSettings.rayMarchingMaxDistance);
        SimdReg maskMax = CMP_GT_PS(dists, maxDist);
        int reachedMaxMask = MOVE_MASK_PS(maskEpsilon);

        //If every ray has exceeded maximum distance
        if(reachedMaxMask == ALL_ONES_MASK) {
            return 0;
        }

        //make sure to not move t for "finished" rays
        SimdReg combinedMask = OR_PS(maskEpsilon, maskMax);
        SimdReg maskedDists = NOT_AND_PS(combinedMask, dists);
        
        //Move t by the distances. Finished rays were masked out and will not change
        t = ADD_PS(t, maskedDists);
    }

    return collidedMask;
}

void RendererSimd::computeNormals(const PointPack &pointPack, CollisionPack &collisionPack) {
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

    STORE_PS(collisionPack.normalX, diffX);
    STORE_PS(collisionPack.normalY, diffY);
    STORE_PS(collisionPack.normalZ, diffZ);
}

void RendererSimd::shade(const CollisionPack &collisionPack, ColorPack &colorPack) {
    
    SimdReg Lx = SET_PS1(-scene.lightDir.x);
    SimdReg Ly = SET_PS1(-scene.lightDir.y);
    SimdReg Lz = SET_PS1(-scene.lightDir.z);
    simdNormalizePack(Lx, Ly, Lz);

    SimdReg Nx = LOAD_PS(collisionPack.normalX);
    SimdReg Ny = LOAD_PS(collisionPack.normalY);
    SimdReg Nz = LOAD_PS(collisionPack.normalZ);

    SimdReg diffuseTerm = MAX_PS(SET_ZERO_PS(), simdDotPack(Nx, Ny, Nz, Lx, Ly, Lz));

    //TODO: colors hardcoded
    SimdReg difR = MUL_PS(SET_PS1(0.6f), diffuseTerm);
    SimdReg difG = MUL_PS(SET_PS1(1.0f), diffuseTerm);
    SimdReg difB = MUL_PS(SET_PS1(0.0f), diffuseTerm);


    SimdReg Vx = SUB_PS(SET_PS1(scene.camera.pos.x), LOAD_PS(collisionPack.pointX));
    SimdReg Vy = SUB_PS(SET_PS1(scene.camera.pos.y), LOAD_PS(collisionPack.pointY));
    SimdReg Vz = SUB_PS(SET_PS1(scene.camera.pos.z), LOAD_PS(collisionPack.pointZ));
    simdNormalizePack(Vx, Vy, Vz);

    SimdReg Hx = ADD_PS(Lx, Vx);
    SimdReg Hy = ADD_PS(Ly, Vy);
    SimdReg Hz = ADD_PS(Lz, Vz);
    simdNormalizePack(Hx, Hy, Hz);

    //SimdReg specularTerm = MAX_PS(SET_ZERO_PS(), simdDotPack(Nx, Ny, Nz, Hx, Hy, Hz));

    //TODO: find a way to do pow with SIMD
    /*float specularHack[SIMD_SIZE];
    STORE_PS(specularHack, specularTerm);
    for(int i = 0; i < SIMD_SIZE; ++i) {
        specularHack[i] = glm::pow(specularHack[i], 50.0f);
    }
    specularTerm = LOAD_PS(specularHack);*/

    //TODO: colors hardcoded
    /*SimdReg specR = MUL_PS(SET_PS1(1.0f), specularTerm);
    SimdReg specG = MUL_PS(SET_PS1(1.0f), specularTerm);
    SimdReg specB = MUL_PS(SET_PS1(1.0f), specularTerm);

    STORE_PS(colorPack.x, ADD_PS(difR, specR));
    STORE_PS(colorPack.y, ADD_PS(difG, specG));
    STORE_PS(colorPack.z, ADD_PS(difB, specB));*/
    
    STORE_PS(colorPack.x, difR);
    STORE_PS(colorPack.y, difG);
    STORE_PS(colorPack.z, difB);
}