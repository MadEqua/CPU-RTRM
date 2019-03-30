#include "Scene.h"

float Scene::sdf(const glm::vec3 &pos) const {
    float min = 999999.0f; //TODO better than this
    
    for(const auto &sphere : spheres) {
        float d = glm::distance(pos, sphere.pos) - sphere.radius;
        if(d < min)
            min = d;
    }
    return min;
}
