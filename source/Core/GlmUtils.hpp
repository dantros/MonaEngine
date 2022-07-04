#pragma once
#ifndef GLMUTILS_HPP
#define GLMUTILS_HPP

#include <vector>
#include <sstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Log.hpp"

namespace Mona {

    namespace glmUtils {
        inline glm::mat3 scaleToMat3(const glm::vec3& scale) {
            glm::mat3 mat = glm::identity<glm::mat3>();
            for (int i = 0; i < 3; i++) {
                mat[i][i] = scale[i];
            }
            return mat;
        }

        inline glm::mat4 scaleToMat4(const glm::vec3& scale) {
            return glm::scale(glm::identity<glm::mat4>(), scale);
        }

        inline glm::mat4 translationToMat4(const glm::vec3& translation) {
            return glm::translate(glm::identity<glm::mat4>(), translation);
        }

        inline glm::mat3 rotationToMat3(const glm::fquat& rotation) {
            return glm::toMat3(rotation);
        }

        inline glm::mat4 rotationToMat4(const glm::fquat& rotation) {
            return glm::toMat4(rotation);
        }

        inline glm::fquat rotationFromMat4(glm::mat4& transformMat) {
            glm::vec3 scale;
            glm::quat rotation;
            glm::vec3 translation;
            glm::vec3 skew;
            glm::vec4 perspective;
            glm::decompose(transformMat, scale, rotation, translation, skew, perspective);
            return rotation;
        }

        inline glm::vec3 translationFromMat4(glm::mat4& transformMat) {
            glm::vec3 scale;
            glm::quat rotation;
            glm::vec3 translation;
            glm::vec3 skew;
            glm::vec4 perspective;
            glm::decompose(transformMat, scale, rotation, translation, skew, perspective);
            return translation;
        }
    
    }
        
    
}


#endif