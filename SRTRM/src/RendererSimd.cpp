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

        //Normalize ray dirs
        SimdReg dirXSq = MUL_PS(dirX, dirX);
        SimdReg dirYSq = MUL_PS(dirY, dirY);
        SimdReg dirZSq = SET_PS1(1.0f); //z = -1
        SimdReg sum = ADD_PS(dirXSq, ADD_PS(dirYSq, dirZSq));
        SimdReg dirLengthInv = RSQRT_PS(sum); //1 / sqrt(sum)

        dirX = MUL_PS(dirLengthInv, dirX);
        dirY = MUL_PS(dirLengthInv, dirY);
        SimdReg dirZ = MUL_PS(dirLengthInv, SET_PS1(-1.0f));

        STORE_PS(rayPack.dirX, dirX);
        STORE_PS(rayPack.dirY, dirY);
        STORE_PS(rayPack.dirZ, dirZ);


        int collisionMask = raymarch(rayPack, collisionPack);
        float *ptr = data + px * 3;

        //TODO: is this performant enough?
        for(uint32 i = 0; i < SIMD_SIZE; ++i) {

            if(collisionMask & (1 << i)) {
                *(ptr + (i * 3) + 0) = 0.2f;
                *(ptr + (i * 3) + 1) = 1.0f;
                *(ptr + (i * 3) + 2) = 0.1f;

                //TODO
                //Color col = shade(collision.point, collision.normal);

                /**ptr = col.r;
                *(ptr + 1) = col.g;
                *(ptr + 2) = col.b;*/
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
            STORE_PS(collisionPack.pointX, BLENDV_PS(LOAD_PS(collisionPack.pointX), pointX, maskEpsilon));
            STORE_PS(collisionPack.pointY, BLENDV_PS(LOAD_PS(collisionPack.pointY), pointY, maskEpsilon));
            STORE_PS(collisionPack.pointZ, BLENDV_PS(LOAD_PS(collisionPack.pointZ), pointZ, maskEpsilon));

            //TODO: compute normals

            //If every ray has collided finish early
            if(collidedMask == ALL_ONES_MASK) {
                return ALL_ONES_MASK;
            }
        }

        //If there is a ray that exceeded max distance
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

/*glm::vec3 RendererSimd::computeNormal(const glm::vec3 &point) {
    const float E = 0.001f;

    glm::vec3 grad;
    grad.x = scene.sdf(point + glm::vec3(E, 0.0f, 0.0f)) - scene.sdf(point - glm::vec3(E, 0.0f, 0.0f));
    grad.y = scene.sdf(point + glm::vec3(0.0f, E, 0.0f)) - scene.sdf(point - glm::vec3(0.0f, E, 0.0f));
    grad.z = scene.sdf(point + glm::vec3(0.0f, 0.0f, E)) - scene.sdf(point - glm::vec3(0.0f, 0.0f, E));
    return glm::normalize(grad);
}

Color RendererSimd::shade(const glm::vec3 &point, const glm::vec3 &normal) {
    glm::vec3 L = glm::normalize(-scene.lightDir);
    glm::vec3 V = glm::normalize(scene.camera.pos - point);
    glm::vec3 H = glm::normalize(L + V);

    //TODO: colors hardcoded
    Color dif = Color(0.6f, 1.0f, 0.0f) * glm::max(0.0f, glm::dot(normal, L));
    Color spec = Color(1.0f) * glm::pow(glm::max(0.0f, glm::dot(normal, H)), 50.0f);
    return dif + spec;
}*/