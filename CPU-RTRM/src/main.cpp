#include "Window.h"
#include "RenderSettings.h"
#include "Types.h"
#include "Renderer.h"
#include "Scene.h"
#include "Camera.h"


int main(int argc, char **argv) {

    RenderSettings renderSettings;
    renderSettings.width = 1000;
    renderSettings.height = renderSettings.width * (9.0f/16.0f);
    renderSettings.rayMarchingSteps = 64;
    renderSettings.rayMarchingEpsilon = 0.005f;
    renderSettings.rayMarchingMaxDistance = 100.0f;
    renderSettings.renderThreads = 4;

    Scene scene(new Camera(glm::vec3(0.0f, 2.0f, -2.0f), 60.0f, renderSettings.width, renderSettings.height));
    
    Window window(renderSettings.width, renderSettings.height);
    Renderer renderer(renderSettings, scene, window);

    renderer.startRenderLoop();

    return 0;
}