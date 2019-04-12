#pragma once

#include "Types.h"
#include "Simd.h"
#include "RenderSettings.h"
#include "Timer.h"

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
    void renderFrame(float dt);

    int raymarch(const RayPack &rayPack, CollisionPack &outCollisionPack);
    void computeNormals(const PointPack &pointPack, VectorPack &outNormalPack);
    void shadeBlinnPhong(const CollisionPack &collisionPack, ColorPack &outColorPack);
    void shadeSteps(const CollisionPack &collisionPack, ColorPack &outColorPack);

    const RenderSettings renderSettings;
    Scene &scene;
    Window &window;

    Timer frameTimer;

    float *data;
    byte *byteData; //TODO : have this outside on a "converter" from float to byte. Tone mapping?

    //Lookup tables for converting pixel number to [x, y] coord.
    float *pixelToX;
    float *pixelToY;
};