#pragma once
#ifndef SPLINES_HPP
#define SPLINES_HPP
#include "glm/glm.hpp"
#include <vector>

namespace Mona{


    class BezierCurve {
        BezierCurve(int order, std::vector<glm::vec3> controlPoints, float minT=0, float maxT=1);
        float bernsteinBP(int i, int n, float t);
        glm::vec3 evalCurve(float t);
        glm::vec3 getVelocity(float t);
        float normalizeT(float t);
        int m_order;
        std::vector<glm::vec3> m_controlPoints;
        float m_minT;
        float m_maxT;
    };

    class CubicBezierSpline {
        CubicBezierSpline(std::vector<glm::vec3> splinePoints);
        std::vector<glm::vec3> m_controlPoints;
    };


    
}





#endif