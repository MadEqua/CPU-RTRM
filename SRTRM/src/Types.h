#pragma once

#include <glm/glm.hpp>

using byte = unsigned char;
using uint32 = unsigned int;
using uint64 = unsigned long long int;

using Color = glm::vec3;

struct Ray {
    glm::vec3 origin;
    glm::vec3 dir;
};

struct Collision {
    glm::vec3 point;
    glm::vec3 normal;
};