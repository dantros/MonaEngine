#pragma once
#ifndef PARAMETRICCURVES_HPP
#define PARAMETRICCURVES_HPP
#include "glm/glm.hpp"
#include <glm/gtx/quaternion.hpp>
#include <vector>

namespace Mona{


    class BezierCurve {
        float bernsteinBP(int i, int n, float t);
        float normalizeT(float t);
        int m_order;
        std::vector<glm::vec3> m_controlPoints;
        float m_minT;
        float m_maxT;
    public:
        BezierCurve(int order, std::vector<glm::vec3> controlPoints, float minT = 0, float maxT = 1);
        BezierCurve() = default;
        glm::vec3 evalCurve(float t);
        glm::vec3 getVelocity(float t);
        int getOrder() { return m_order; }
        void rotate(glm::fquat rotation);
        void scale(glm::vec3 scaling);
        void translate(glm::vec3 translation);
        glm::vec2 getTRange() { return glm::vec2({ m_minT, m_maxT }); }
        bool inTRange(float t) { return m_minT <= t && t <= m_maxT; }
    };

    class BezierSpline {
    public:
        enum class Order {
            LINEAR,
            CUBIC
        };
    private:
        std::vector<BezierCurve> m_bezierCurves;
        float m_minT;
        float m_maxT;
        Order m_order;
    public:
        BezierSpline() = default;
        BezierSpline(std::vector<glm::vec3> splinePoints, std::vector<float> tValues, Order order);
        glm::vec3 evalSpline(float t);
        glm::vec3 getVelocity(float t);
        int getOrder();
        glm::vec2 getTRange() { return glm::vec2({ m_minT, m_maxT }); }
        int getSplinePointNum(float minT, float maxT);
        bool inTRange(float t) { return m_minT <= t && t <= m_maxT; }
        BezierSpline sample(float minT, float maxT, int innerSplinePointNumber);
    };


    
}





#endif