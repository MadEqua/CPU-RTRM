#pragma once

#include "Types.h"
#include "RenderSettings.h"

class Scene;
class Window;

class Renderer
{
public:
    Renderer(const RenderSettings &renderSettings, Scene &scene, Window &window);
    ~Renderer();

    void startRenderLoop();

private:
    void updateData(float dt);
    void renderFrame(float dt);

    bool raymarch(const Ray &ray, Collision &collision);
    glm::vec3 computeNormal(const glm::vec3 &point);
    Color shadeBlinnPhong(const glm::vec3 &point, const glm::vec3 &normal);

    const RenderSettings renderSettings;
    Scene &scene;
    Window &window;

    float *data;
    byte *byteData; //TODO : have this outside on a "converter" from float to byte. Tone mapping?
};