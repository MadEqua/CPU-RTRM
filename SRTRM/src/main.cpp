#include "Window.h"
#include "RenderSettings.h"
#include "Types.h"
#include "Renderer.h"
#include "RendererSimd.h"
#include "Scene.h"
#include "Camera.h"


int main(int argc, char **argv) {

    RenderSettings renderSettings;
    renderSettings.width = 1000;
    renderSettings.height = renderSettings.width * (9.0f/16.0f);
    renderSettings.rayMarchingSteps = 32;
    renderSettings.rayMarchingEpsilon = 0.005f;
    renderSettings.rayMarchingMaxDistance = 100.0f;
    renderSettings.renderThreads = 4;

    Scene scene(new Camera(glm::vec3(0.0f, 1.0f, -2.0f), 90.0f, renderSettings.width, renderSettings.height));
    Sphere s = {{0.0f, -1000.0f, 0.0f}, 1000.0f};
    scene.spheres.push_back(s);

    Sphere s1 = {{0.5f, 0.5f, 0.0f}, 0.5f};
    scene.spheres.push_back(s1);

    Sphere s2 = {{-0.5f, 0.5f, 0.0f}, 0.5f};
    scene.spheres.push_back(s2);
    
    Window window(renderSettings.width, renderSettings.height);
    //Renderer renderer(renderSettings, scene, window);
    RendererSimd renderer(renderSettings, scene, window);

    renderer.startRenderLoop();

    return 0;
}