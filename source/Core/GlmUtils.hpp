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

        template <int D>
        inline bool isApproxUniform(glm::vec<D, float> vec, float epsilon = 0.000001) {
            float baseValue = vec[0];
            for (int i = 1; i < D; i++) {
                if (glm::epsilonNotEqual(vec[i], baseValue, epsilon)) {
                    return false;
                }
            }
            return true;
        }

        template <int D>
        inline bool areApproxEqual(glm::vec<D, float> vec1, glm::vec<D, float> vec2, float epsilon= 0.000001) {
            for (int i = 0; i < D; i++) {
                if (glm::epsilonNotEqual(vec1[i], vec2[i], epsilon)) {
                    return false;
                }
            }
            return true;
        }
    

    
    }
        
    
}


#endif