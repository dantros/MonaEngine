#pragma once
#ifndef GLMUTILS_HPP
#define GLMUTILS_HPP

#include <vector>
#include <sstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Log.hpp"

namespace Mona {

    namespace glmUtils {
        inline glm::mat3x3 scaleToMat3(const glm::vec3& scale) {
            glm::mat3x3 mat = glm::identity<glm::mat3x3>();
            for (int i = 0; i < 3; i++) {
                mat[i][i] = scale[i];
            }
            return mat;
        }

        inline glm::mat4x4 scaleToMat4(const glm::vec3& scale) {
            return glm::scale(glm::identity<glm::mat4x4>(), scale);
        }

        inline glm::mat4x4 translationToMat4(const glm::vec3& translation) {
            return glm::translate(glm::identity<glm::mat4x4>(), translation);
        }

        inline glm::mat3x3 rotationToMat3(const glm::fquat& rotation) {
            return glm::toMat3(rotation);
        }

        inline glm::mat4x4 rotationToMat4(const glm::fquat& rotation) {
            return glm::toMat4(rotation);
        }
        
        template <int len, typename T>
        inline glm::vec<len, T> stdVecToGlm(const std::vector<T>& stdVec) {
            glm::vec<len, T> glmVec;
            for (int i = 0; i < len; i++) {
                glmVec[i] = stdVec[i];
            }
            return glmVec;
        }
    
    
    }
        
    
}


#endif