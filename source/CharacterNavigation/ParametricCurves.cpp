#include "ParametricCurves.hpp"
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"
#include <math.h>

namespace Mona {

    template <typename T>
    LIC<T>::LIC(std::vector<T> curvePoints, std::vector<float> tValues) {
        MONA_ASSERT(1 < curvePoints.size(), "LIC: must provide at least two points.");
        MONA_ASSERT(curvePoints.size() == tValues.size(), "LIC: there must be exactly one tValue per spline point.");
        // chequeamos que los tValues vengan correctamente ordenados
        for (int i = 1; i < tValues.size(); i++) {
            if (!(tValues[i - 1] < tValues[i])) {
                MONA_LOG_ERROR("LIC: tValues must come in a strictly ascending order");
                return;
            }
        }
        m_curvePoints = curvePoints;
        m_tValues = tValues;
    }


    template <typename T>
    T LIC<T>::getLeftHandVelocity(float t) {
        MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
        T zero = m_curvePoints[0] - m_curvePoints[0];
        if (t == m_tValues[0]) { return zero }
        for (int i = 0; i < m_tValues.size(); i++) {
            if (m_tValues[i] == t) {
                return (m_curvePoints[i] - m_curvePoints[i - 1]) / (m_tValues[i] - m_tValues[i - 1]);
            }
            else if (m_tValues[i] < t && t < m_tValues[i + 1]) {
                return (m_curvePoints[i + 1] - m_curvePoints[i]) / (m_tValues[i + 1] - m_tValues[i]);
            }
        }
        return zero;
    }

    template <typename T>
    T LIC<T>::getRightHandVelocity(float t) {
        MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
        T zero = m_curvePoints[0] - m_curvePoints[0];
        if (t == m_tValues.back()) { return zero }
        for (int i = 0; i < m_tValues.size(); i++) {
            if (m_tValues[i] == t) {
                return (m_curvePoints[i + 1] - m_curvePoints[i]) / (m_tValues[i + 1] - m_tValues[i]);
            }
            else if (m_tValues[i] < t && t < m_tValues[i + 1]) {
                return (m_curvePoints[i + 1] - m_curvePoints[i]) / (m_tValues[i + 1] - m_tValues[i]);
            }
        }
        return zero;
    }

    template <typename T>
    T LIC<T>::evalCurve(float t) {
        MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
        T zero = m_curvePoints[0] - m_curvePoints[0];
        for (int i = 0; i < m_tValues.size() - 1; i++) {
            if (m_tValues[i] <= t && t <= m_tValues[i + 1]) {
                float fraction = funcUtils::getFraction(m_tValues[i], m_tValues[i + 1], t);
                return funcUtils::lerp(m_curvePoints[i], m_curvePoints[i + 1], fraction);
            }
        }
        return zero;
    }

    template <typename T>
    void LIC<T>::displacePointT(int pointIndex, float newT, float pointScalingRatio) {
        MONA_ASSERT(inTRange(t), "LIC: newT must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
        glm::vec2 tRange = getTRange();
        float oldT = m_tValues[pointIndex];
        float fractionBelow = funcUtils::getFraction(tRange[0], oldT, newT);
        float fractionAbove = funcUtils::getFraction(tRange[1], oldT, newT);
        for (int i = 0; i < pointIndex; i++) {
            tRange[i] = funcUtils::lerp(tRange[0], tRange[i], fractionBelow);
            m_curvePoints[i] = funcUtils::lerp(m_curvePoints[0], m_curvePoints[i], fractionBelow * positionScalingRatio);
        }
        for (int i = pointIndex; i < m_curvePoints.size(); i++) {
            tRange[i] = funcUtils::lerp(tRange[1], tRange[i], fractionAbove);
            m_curvePoints[i] = funcUtils::lerp(m_curvePoints.back(), m_curvePoints[i], fractionAbove * positionScalingRatio);
        }

    }
    template <typename T>
    void LIC<T>::setCurvePoint(int pointIndex, T newValue) {
        MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "LIC: input index must be within bounds");
        m_curvePoints[pointIndex] = newValue;
    }

    template <typename T>
    LIC<T> LIC<T>::getSubCurve(int minPointIndex, int maxPointIndex) {
        MONA_ASSERT(minPointIndex < maxPointIndex, "LIC: max point index must be greater than min point index.");
        std::vector<float> subCurveTValues(maxPointIndex - minPointIndex + 1);
        std::vector<T> subCurvePoints(maxPointIndex - minPointIndex + 1);
        for (int i = minPointIndex; i <= maxPointIndex; i++) {
            subCurveTValues[i] = m_tValues[i];
            subCurvePoints[i] = m_curvePoints[i];
        }
        return LIC(subCurvePoints, subCurveTValues);
    }

    template <typename T>
    void LIC<T>::scale(T scaling) {
        for (int i = 0; i < m_curvePoints.size(); i++) {
            m_curvePoints[i] *= scaling;
        }
    }

    template <typename T>
    void LIC<T>::translate(T translation) {
        for (int i = 0; i < m_curvePoints.size(); i++) {
            m_curvePoints[i] += translation;
        }
    }

    
}