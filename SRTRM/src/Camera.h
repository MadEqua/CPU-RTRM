#pragma once

#include "Types.h"
#include "Simd.h"

#include <SFML/System/Vector2.hpp>

//Camera that rotates around the origin
class Camera
{
public:
    Camera(float initialPositionZ, float fovy, uint32 widthPx, uint32 heightPx);

    //Generates world space rays from pixel/screen coords
    void generateRayPack(const Point2Pack &pixelCoordsPack, RayPack &outRayPack) const;
    
    void update(float dt);

    glm::vec3 getPosition() const { return positionWorld; }

private:
    float initialPositionZ;
    float tanHalfFovy;
    uint32 widthPx, heightPx;
    float invWidth, invHeight;
    float aspectRatio;

    float yRot = 0.0f;
    float xRot = 0.0f;
    glm::vec3 positionWorld; //This can be always derived from rotation. It's stored for efficiency.

    sf::Vector2i lastCursorPos = {-1, -1};

    glm::mat3 cameraToWorldMatrix;
    void updateMatrix();

    //static float angleClamp(float r);
};

