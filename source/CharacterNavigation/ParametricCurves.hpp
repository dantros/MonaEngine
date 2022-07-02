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
        LIC getSubCurve(int minPointIndex, int maxPointIndex);
        void scale(T scaling);
        void translate(T translation);
    };   


    
}





#endif