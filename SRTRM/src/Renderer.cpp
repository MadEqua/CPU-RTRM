#include "Renderer.h"

#include <chrono>

#include <glm/glm.hpp>

#include "Scene.h"
#include "Window.h"


Renderer::Renderer(const RenderSettings &renderSettings, Scene &scene, Window &window) :
    renderSettings(renderSettings),
    scene(scene),
    window(window) {

    uint32 pixelCount = renderSettings.width * renderSettings.height;
    data = new float[pixelCount * 3u];

    //SFML requires RGBA :(
    byteData = new byte[pixelCount * 4u];
}

Renderer::~Renderer() {
    delete[] data;
    delete[] byteData;
}

void Renderer::startRenderLoop() {
    using namespace std::chrono;

    time_point<steady_clock> start = steady_clock::now();
    time_point<steady_clock> end = steady_clock::now();

    while(!window.isOpen())
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

    while(window.isOpen()) {
        float dt = duration_cast<duration<float>>(end - start).count();
        start = steady_clock::now();
        updateData(dt);
        renderFrame(dt);
        window.notifyUpdate(byteData, dt);
        end = steady_clock::now();
    };
}

void Renderer::updateData(float dt) {
}

void Renderer::renderFrame(float dt) {
    uint32 pixelCount = renderSettings.width * renderSettings.height;

    float xStep = 1.0f / static_cast<float>(renderSettings.width);
    float yStep = 1.0f / static_cast<float>(renderSettings.height);

    Ray ray;
    ray.origin = scene.camera->getPosition();

    //TODO: bad traversal order for ray coherency (rays along lines will likely collide with different objects)
    //but it's a good order for writing the results to memory
    for(uint32 px = 0; px < pixelCount; ++px) {
        
        //Pixel coordinates from linear array
        uint32 x = px % renderSettings.width;
        uint32 y = px / renderSettings.width;
        
        //The y=0 pixel should be the bottom of the image
        uint32 invertedY = renderSettings.height - y - 1;

        //map [0, 1] to [-1, 1]
        float xDir = ((static_cast<float>(x) + 0.5f) * xStep) * 2.0f - 1.0f;
        float yDir = ((static_cast<float>(invertedY) + 0.5f) * yStep) * 2.0f - 1.0f;

        ray.dir = glm::normalize(glm::vec3(xDir, yDir, -1.0f));

        Collision collision;
        bool hit = raymarch(ray, collision);
        float *ptr = data + px * 3;

        if(hit) {
            Color col = shadeBlinnPhong(collision.point, collision.normal);

            *ptr = col.r;
            *(ptr + 1) = col.g;
            *(ptr + 2) = col.b;
        }
        else {
            *ptr = 0.5f;
            *(ptr + 1) = 0.5f;
            *(ptr + 2) = 0.5f;
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

bool Renderer::raymarch(const Ray &ray, Collision &collision) {
    float t = 0.0f;
    for(uint32 i = 0; i < renderSettings.rayMarchingSteps; ++i) {
        glm::vec3 point = ray.origin + t * ray.dir;
        float dist = scene.sdf(point);

        if(dist < renderSettings.rayMarchingEpsilon) {
            collision.point = point;
            collision.normal = computeNormal(point);
            return true;
        }

        if(dist > renderSettings.rayMarchingMaxDistance) {
            return false;
        }

        t += dist;
    }

    return false;
}

glm::vec3 Renderer::computeNormal(const glm::vec3 &point) {
    const float E = 0.001f;

    glm::vec3 grad;
    grad.x = scene.sdf(point + glm::vec3(E, 0.0f, 0.0f)) - scene.sdf(point - glm::vec3(E, 0.0f, 0.0f));
    grad.y = scene.sdf(point + glm::vec3(0.0f, E, 0.0f)) - scene.sdf(point - glm::vec3(0.0f, E, 0.0f));
    grad.z = scene.sdf(point + glm::vec3(0.0f, 0.0f, E)) - scene.sdf(point - glm::vec3(0.0f, 0.0f, E));
    return glm::normalize(grad);
}

Color Renderer::shadeBlinnPhong(const glm::vec3 &point, const glm::vec3 &normal) {
    glm::vec3 L = glm::normalize(-scene.lightDir);
    glm::vec3 V = glm::normalize(scene.camera->getPosition() - point);
    glm::vec3 H = glm::normalize(L + V);

    //TODO: colors hardcoded
    Color dif = Color(0.6f, 1.0f, 0.0f) * glm::max(0.0f, glm::dot(normal, L));
    Color spec = Color(1.0f) * glm::pow(glm::max(0.0f, glm::dot(normal, H)), 50.0f);
    return dif + spec;
}