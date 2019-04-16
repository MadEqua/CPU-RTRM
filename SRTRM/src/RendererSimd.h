#pragma once

#include "Types.h"
#include "Simd.h"
#include "RenderSettings.h"
#include "Timer.h"

#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

class Scene;
class Window;

class RendererSimd
{
public:
    RendererSimd(const RenderSettings &renderSettings, Scene &scene, Window &window);
    ~RendererSimd();

    void startRenderLoop();

private:
    void updateData(float dt);
    void renderTask();

    int raymarch(const RayPack &rayPack, CollisionPack &outCollisionPack);
    void computeNormals(const PointPack &pointPack, VectorPack &outNormalPack);
    void shadeBlinnPhong(const CollisionPack &collisionPack, ColorPack &outColorPack);
    void shadeSteps(const CollisionPack &collisionPack, ColorPack &outColorPack);

    const RenderSettings renderSettings;
    Scene &scene;
    Window &window;

    Timer frameTimer;

    const uint32 pixelCount;

    std::vector<std::thread> workerPool;
    std::atomic<int> nextPack;
    std::mutex mutexUpdate;
    std::condition_variable updateFinishedCv;
    bool updateFinished = false;
    std::mutex mutexRender;
    std::condition_variable renderFinishedCv;
    int workersFinished = 0;

    float *data;
    byte *byteData; //TODO : have this outside on a "converter" from float to byte. Tone mapping?

    //Lookup tables for converting pixel number to [x, y] coord.
    float *pixelToX;
    float *pixelToY;
};