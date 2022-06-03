#pragma once
#ifndef SPLINES_HPP
#define SPLINES_HPP
#include "glm/glm.hpp"
#include <vector>

namespace Mona{


    class BezierCurve {
        BezierCurve(int order, std::vector<glm::vec3> controlPoints);
        float BernsteinBP(int i, int n, float t);
        glm::vec3 evalCurve(float t);
        glm::vec3 getVelocity(float t);
        int m_order;
        std::vector<glm::vec3> m_controlPoints;
    };


    
}





#endif