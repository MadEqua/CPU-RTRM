#include "Window.h"
#include "RenderSettings.h"
#include "Types.h"
#include "Renderer.h"
#include "RendererSimd.h"
#include "Scene.h"


int main(int argc, char **argv) {

    RenderSettings renderSettings;
    renderSettings.width = 700;
    renderSettings.height = 700;
    renderSettings.rayMarchingSteps = 16;
    renderSettings.rayMarchingEpsilon = 0.01f;
    renderSettings.rayMarchingMaxDistance = 10.0f;

    Scene scene;
    Sphere s = {{0.0f, 0.0f, 0.0f}, 0.5f};
    scene.spheres.push_back(s);

    Sphere s1 = {{1.0f, 0.0f, 0.0f}, 0.4f};
    scene.spheres.push_back(s1);

    Sphere s2 = {{-1.0f, 0.0f, 0.0f}, 0.4f};
    scene.spheres.push_back(s2);
    
    Window window(renderSettings);
    //Renderer renderer(renderSettings, scene, window);
    RendererSimd renderer(renderSettings, scene, window);
    
    renderer.startRenderLoop();

    return 0;
}