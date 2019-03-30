#pragma once

#include <vector>
#include <glm/glm.hpp>


//TODO: test using SoA
struct Sphere {
    glm::vec3 pos;
    float radius;
};


//TODO: yeah...
struct Camera {
    glm::vec3 pos = {0.0f, 0.0f, 2.0f};
    glm::vec3 lookAt = {0.0f, 0.0f, 0.0f};
    glm::vec3 up = {0.0f, 1.0f, 0.0f};
};

struct Ray {
    glm::vec3 origin;
    glm::vec3 dir;
};

class Scene
{
public:
    std::vector<Sphere> spheres;
    Camera camera;
    glm::vec3 lightDir = {1.0f, -1.0f, 0.0f};

    float sdf(const glm::vec3 &pos) const;
};

