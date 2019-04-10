#include "Camera.h"

#include <SFML/Window.hpp>

Camera::Camera() {
    updateMatrix();
}

void Camera::generateRayPack(const Point2Pack &pixelCoordsPack, RayPack &outRayPack) const {

    SimdReg pixelCoordX = LOAD_PS(pixelCoordsPack.x);
    SimdReg pixelCoordY = LOAD_PS(pixelCoordsPack.y);

    //Map coords to [-1, 1]
    pixelCoordX = MUL_PS(pixelCoordX, SET_PS1(2.0f));
    pixelCoordX = SUB_PS(pixelCoordX, SET_PS1(1.0f));

    pixelCoordY = MUL_PS(pixelCoordY, SET_PS1(2.0f));
    pixelCoordY = SUB_PS(pixelCoordY, SET_PS1(1.0f));

    SimdReg dirCameraZ = SET_PS1(1.0f); //TODO: this should be derived, not hardcoded


    //Convert camera space to world space
    SimdReg xBaseX = SET_PS1(cameraToWorldMatrix[0].x);
    SimdReg yBaseX = SET_PS1(cameraToWorldMatrix[1].x);
    SimdReg zBaseX = SET_PS1(cameraToWorldMatrix[2].x);
    SimdReg dirWorldX = ADD_PS(ADD_PS(MUL_PS(pixelCoordX, xBaseX), MUL_PS(pixelCoordY, yBaseX)), MUL_PS(dirCameraZ, zBaseX));

    SimdReg xBaseY = SET_PS1(cameraToWorldMatrix[0].y);
    SimdReg yBaseY = SET_PS1(cameraToWorldMatrix[1].y);
    SimdReg zBaseY = SET_PS1(cameraToWorldMatrix[2].y);
    SimdReg dirWorldY = ADD_PS(ADD_PS(MUL_PS(pixelCoordX, xBaseY), MUL_PS(pixelCoordY, yBaseY)), MUL_PS(dirCameraZ, zBaseY));

    SimdReg xBaseZ = SET_PS1(cameraToWorldMatrix[0].z);
    SimdReg yBaseZ = SET_PS1(cameraToWorldMatrix[1].z);
    SimdReg zBaseZ = SET_PS1(cameraToWorldMatrix[2].z);
    SimdReg dirWorldZ = ADD_PS(ADD_PS(MUL_PS(pixelCoordX, xBaseZ), MUL_PS(pixelCoordY, yBaseZ)), MUL_PS(dirCameraZ, zBaseZ));

    simdNormalizePack(dirWorldX, dirWorldY, dirWorldZ);
    STORE_PS(outRayPack.dirX, dirWorldX);
    STORE_PS(outRayPack.dirY, dirWorldY);
    STORE_PS(outRayPack.dirZ, dirWorldZ);

    outRayPack.origin = positionWorld;
}

void Camera::update(float dt) {
    if(sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
        auto curCursorPos = sf::Mouse::getPosition();
        if(lastCursorPos.x != -1) {
            auto diff = lastCursorPos - curCursorPos;

            yRot -= diff.x;
            if(yRot < 0.0f) yRot = 360.0f + yRot;
            else if(yRot > 360.0f) yRot = yRot - 360.0f;

            updateMatrix();
        }
        lastCursorPos = curCursorPos;
    }
    else {
        lastCursorPos.x = -1;
    }
}

void Camera::updateMatrix() {
    const glm::vec3 ROTATION_0_POS(0.0f, 0.0f, -1.3f);
    const glm::vec3 UP_VECTOR(0.0f, 1.0f, 0.0f);

    auto yRotRad = glm::radians(yRot);
    auto cos = glm::cos(yRotRad);
    auto sin = glm::sin(yRotRad);
    auto yRotMatrix = glm::mat3(cos, 0.0f, -sin,
                                0.0f, 1.0f, 0.0f,
                                sin, 0.0f, cos);

    positionWorld = yRotMatrix * ROTATION_0_POS;

    cameraToWorldMatrix[2] = glm::normalize(-positionWorld);
    cameraToWorldMatrix[0] = glm::normalize(glm::cross(UP_VECTOR, cameraToWorldMatrix[2]));
    cameraToWorldMatrix[1] = glm::normalize(glm::cross(cameraToWorldMatrix[2], cameraToWorldMatrix[0]));
}