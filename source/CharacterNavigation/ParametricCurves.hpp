#pragma once
#ifndef PARAMETRICCURVES_HPP
#define PARAMETRICCURVES_HPP
#include "glm/glm.hpp"
#include <glm/gtx/quaternion.hpp>
#include <vector>

namespace Mona{

    // linearly interpolated curve
    template <typename T>
    class LIC {
    private:
        // puntos de la curva
        std::vector<T> m_curvePoints;
        // valores de t
        std::vector<float> m_tValues = { 1,0 };
    public:
        LIC() = default;
        LIC(std::vector<T> curvePoints, std::vector<float> tValues);
        glm::vec2 getTRange() { return glm::vec2({ m_tValues[0], m_tValues.back() }); }
        bool inTRange(float t) { return m_tValues[0] <= t && t <= m_tValues.back(); }
        const std::vector<float>& getTValues() const { return m_tValues; };
        int getPointNumber() const { return m_curvePoints.size(); }
        T evalCurve(float t);
        T getLeftHandVelocity(float t);
        T getRightHandVelocity(float t);
        void displacePointT(int pointIndex, float newT, float pointScalingRatio = 1);
        void setCurvePoint(int pointIndex, T newValue);
    };

    class DiscreteCurve {
    private:
        // puntos de la curva
        std::vector<glm::vec3> m_curvePoints;
        // valores de t
        std::vector<float> m_tValues;
    public:
        DiscreteCurve() = default;
        DiscreteCurve(std::vector<glm::vec3> curvePoints, std::vector<float> tValues);
        glm::vec2 getTRange();
        const std::vector<float>& getTValues() const { return m_tValues; };
        bool inTRange(float t);
        glm::vec3 evalCurve(int pointIndex);
        glm::vec3 getVelocity(int pointIndex);
        void displacePointT(int pointIndex, float newT, float positionScalingRatio=1);
        void setCurvePoint(int pointIndex, glm::vec3 newValue);
    };

    class BezierCurve {
        float bernsteinBP(int i, int n, float t);
        float normalizeT(float t);
        int m_order = -1;
        std::vector<glm::vec3> m_controlPoints;
        float m_minT = 1;
        float m_maxT = 0;
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
        float m_minT = 1;
        float m_maxT = 0;
        Order m_order;
    public:
        BezierSpline() = default;
        BezierSpline(std::vector<glm::vec3> splinePoints, std::vector<float> tValues, Order order = Order::CUBIC);
        glm::vec3 evalSpline(float t);
        glm::vec3 getVelocity(float t);
        int getOrder();
        glm::vec2 getTRange() { return glm::vec2({ m_minT, m_maxT }); }
        int getSplinePointNum(float minT, float maxT);
        bool inTRange(float t) { return m_minT <= t && t <= m_maxT; }
        DiscreteCurve sampleDiscrete(float minT, float maxT, int innerSplinePointNumber);
    };

    


    
}





#endif