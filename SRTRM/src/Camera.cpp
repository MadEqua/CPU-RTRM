#include "Camera.h"

#include <SFML/Window.hpp>

Camera::Camera(const glm::vec3 &initialPosition, float fovy, uint32 widthPx, uint32 heightPx) :
    initialPosition(initialPosition),
    tanHalfFovy(glm::tan(glm::radians(fovy * 0.5f))),
    widthPx(widthPx), heightPx(heightPx),
    invWidth(1.0f / static_cast<float>(widthPx)),
    invHeight(1.0f / static_cast<float>(heightPx)),
    aspectRatio(static_cast<float>(widthPx) / static_cast<float>(heightPx)) {
    updateMatrix();
}

void Camera::generateRayPack(const Point2Pack &pixelCoordsPack, RayPack &outRayPack) const {

    SimdReg pixelCoordX = LOAD_PS(pixelCoordsPack.x);
    SimdReg pixelCoordY = LOAD_PS(pixelCoordsPack.y);

    //Convert to [0, 1] range
    pixelCoordX = ADD_PS(pixelCoordX, SET_PS1(0.5f)); //use the pixel center
    pixelCoordX = MUL_PS(pixelCoordX, SET_PS1(invWidth));

    pixelCoordY = ADD_PS(pixelCoordY, SET_PS1(0.5f));
    pixelCoordY = MUL_PS(pixelCoordY, SET_PS1(invHeight));

    //Map coords to [-1, 1]
    pixelCoordX = MUL_PS(pixelCoordX, SET_PS1(2.0f));
    pixelCoordX = SUB_PS(pixelCoordX, SET_PS1(1.0f));

    pixelCoordY = MUL_PS(pixelCoordY, SET_PS1(2.0f));
    pixelCoordY = SUB_PS(pixelCoordY, SET_PS1(1.0f));

    //Adjust x to the aspect ratio
    pixelCoordX = MUL_PS(pixelCoordX, SET_PS1(aspectRatio));

    //Apply fovy factor
    pixelCoordX = MUL_PS(pixelCoordX, SET_PS1(tanHalfFovy));
    pixelCoordY = MUL_PS(pixelCoordY, SET_PS1(tanHalfFovy));

    //Convert camera space to world space
    SimdReg xBaseX = SET_PS1(cameraToWorldMatrix[0].x);
    SimdReg yBaseX = SET_PS1(cameraToWorldMatrix[1].x);
    SimdReg zBaseX = SET_PS1(cameraToWorldMatrix[2].x);
    SimdReg dirWorldX = ADD_PS(ADD_PS(MUL_PS(pixelCoordX, xBaseX), MUL_PS(pixelCoordY, yBaseX)), zBaseX);

    SimdReg xBaseY = SET_PS1(cameraToWorldMatrix[0].y);
    SimdReg yBaseY = SET_PS1(cameraToWorldMatrix[1].y);
    SimdReg zBaseY = SET_PS1(cameraToWorldMatrix[2].y);
    SimdReg dirWorldY = ADD_PS(ADD_PS(MUL_PS(pixelCoordX, xBaseY), MUL_PS(pixelCoordY, yBaseY)), zBaseY);

    SimdReg xBaseZ = SET_PS1(cameraToWorldMatrix[0].z);
    SimdReg yBaseZ = SET_PS1(cameraToWorldMatrix[1].z);
    SimdReg zBaseZ = SET_PS1(cameraToWorldMatrix[2].z);
    SimdReg dirWorldZ = ADD_PS(ADD_PS(MUL_PS(pixelCoordX, xBaseZ), MUL_PS(pixelCoordY, yBaseZ)), zBaseZ);

    simdNormalizePack(dirWorldX, dirWorldY, dirWorldZ);
    STORE_PS(outRayPack.directions.x, dirWorldX);
    STORE_PS(outRayPack.directions.y, dirWorldY);
    STORE_PS(outRayPack.directions.z, dirWorldZ);

    outRayPack.origin = positionWorld;
}

void Camera::update(float dt) {
    if(sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
        auto curCursorPos = sf::Mouse::getPosition();
        bool xMov = lastCursorPos.x != -1;
        bool yMov = lastCursorPos.y != -1;
        if(xMov || yMov) {
            auto diff = lastCursorPos - curCursorPos;
            if(xMov)
                yRot = glm::clamp(yRot - diff.x, -180.0f, 179.9f);
            /*if(yMov)
                xRot = glm::clamp(xRot + diff.y, 0.0f, 90.0f);*/
            updateMatrix();
        }
        lastCursorPos = curCursorPos;
    }
    else if(sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
        auto curCursorPos = sf::Mouse::getPosition();
        if(lastCursorPos.y != -1) {
            auto diff = lastCursorPos.y - curCursorPos.y;
            auto newPos = initialPosition.z + diff * 0.01f;
            if(glm::sign(newPos) == glm::sign(initialPosition.z)) {
                initialPosition.z = newPos;
                updateMatrix();
            }
        }
        lastCursorPos = curCursorPos;
    }
    else {
        lastCursorPos.x = -1;
        lastCursorPos.y = -1;
    }
}

void Camera::updateMatrix() {
    const glm::vec3 UP_VECTOR(0.0f, 1.0f, 0.0f);

    auto yRotRad = glm::radians(yRot);
    auto cosY = glm::cos(yRotRad);
    auto sinY = glm::sin(yRotRad);
    auto yRotMatrix = glm::mat3(cosY, 0.0f, -sinY,
                                0.0f, 1.0f, 0.0f,
                                sinY, 0.0f, cosY);

    auto xRotRad = glm::radians(xRot);
    auto cosX = glm::cos(xRotRad);
    auto sinX = glm::sin(xRotRad);
    auto xRotMatrix = glm::mat3(1.0f, 0.0f, 0.0f,
                                0.0f, cosX, -sinX,
                                0.0f, sinX, cosX);

    positionWorld = xRotMatrix * yRotMatrix * initialPosition;

    cameraToWorldMatrix[2] = glm::normalize(-positionWorld);
    cameraToWorldMatrix[0] = glm::normalize(glm::cross(UP_VECTOR, cameraToWorldMatrix[2]));
    cameraToWorldMatrix[1] = glm::normalize(glm::cross(cameraToWorldMatrix[2], cameraToWorldMatrix[0]));
}

/*float Camera::angleClamp(float r) {
    if(r < 0.0f) return 360.0f + r;
    else if(r > 360.0f) return r - 360.0f;
    else return r;
}*/