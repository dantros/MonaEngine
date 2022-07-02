#include "ParametricCurves.hpp"
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"
#include <math.h>

namespace Mona {

    template <int D>
    LIC<D>::LIC(std::vector<glm::vec<D, float>> curvePoints, std::vector<float> tValues) {
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
        m_dimension = D;
    }


    template <int D>
    glm::vec<D, float> LIC<D>::getLeftHandVelocity(float t) {
        MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
        if (t == m_tValues[0]) { return glm::zero<glm::vec<D, float>>;}
        for (int i = 0; i < m_tValues.size(); i++) {
            if (m_tValues[i] == t) {
                return (m_curvePoints[i] - m_curvePoints[i - 1]) / (m_tValues[i] - m_tValues[i - 1]);
            }
            else if (m_tValues[i] < t && t < m_tValues[i + 1]) {
                return (m_curvePoints[i + 1] - m_curvePoints[i]) / (m_tValues[i + 1] - m_tValues[i]);
            }
        }
        return glm::zero<glm::vec<D, float>>;;
    }

    template <int D>
    glm::vec<D, float> LIC<D>::getRightHandVelocity(float t) {
        MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
        if (t == m_tValues.back()) { return glm::zero<glm::vec<D, float>>;}
        for (int i = 0; i < m_tValues.size(); i++) {
            if (m_tValues[i] == t) {
                return (m_curvePoints[i + 1] - m_curvePoints[i]) / (m_tValues[i + 1] - m_tValues[i]);
            }
            else if (m_tValues[i] < t && t < m_tValues[i + 1]) {
                return (m_curvePoints[i + 1] - m_curvePoints[i]) / (m_tValues[i + 1] - m_tValues[i]);
            }
        }
        return glm::zero<glm::vec<D, float>>;
    }

    template <int D>
    glm::vec<D, float> LIC<D>::evalCurve(float t) {
        MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
        for (int i = 0; i < m_tValues.size() - 1; i++) {
            if (m_tValues[i] <= t && t <= m_tValues[i + 1]) {
                float fraction = funcUtils::getFraction(m_tValues[i], m_tValues[i + 1], t);
                return funcUtils::lerp(m_curvePoints[i], m_curvePoints[i + 1], fraction);
            }
        }
        return glm::zero<glm::vec<D, float>>;
    }

    template <int D>
    void LIC<D>::displacePointT(int pointIndex, float newT, float pointScalingRatio) {
        MONA_ASSERT(inTRange(newT), "LIC: newT must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
        glm::vec2 tRange = getTRange();
        float oldT = m_tValues[pointIndex];
        float fractionBelow = funcUtils::getFraction(tRange[0], oldT, newT);
        float fractionAbove = funcUtils::getFraction(tRange[1], oldT, newT);
        for (int i = 0; i < pointIndex; i++) {
            tRange[i] = funcUtils::lerp(tRange[0], tRange[i], fractionBelow);
            m_curvePoints[i] = funcUtils::lerp(m_curvePoints[0], m_curvePoints[i], fractionBelow * pointScalingRatio);
        }
        for (int i = pointIndex; i < m_curvePoints.size(); i++) {
            tRange[i] = funcUtils::lerp(tRange[1], tRange[i], fractionAbove);
            m_curvePoints[i] = funcUtils::lerp(m_curvePoints.back(), m_curvePoints[i], fractionAbove * pointScalingRatio);
        }

    }
    template <int D>
    void LIC<D>::setCurvePoint(int pointIndex, glm::vec<D, float> newValue) {
        MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "LIC: input index must be within bounds");
        m_curvePoints[pointIndex] = newValue;
    }

    template <int D>
    LIC<D> LIC<D>::getSubCurve(int minPointIndex, int maxPointIndex) {
        MONA_ASSERT(minPointIndex < maxPointIndex, "LIC: max point index must be greater than min point index.");
        std::vector<float> subCurveTValues(maxPointIndex - minPointIndex + 1);
        std::vector<glm::vec<D, float>> subCurvePoints(maxPointIndex - minPointIndex + 1);
        for (int i = minPointIndex; i <= maxPointIndex; i++) {
            subCurveTValues[i] = m_tValues[i];
            subCurvePoints[i] = m_curvePoints[i];
        }
        return LIC(subCurvePoints, subCurveTValues);
    }

    template <int D>
    void LIC<D>::scale(glm::vec<D, float> scaling) {
        for (int i = 0; i < m_curvePoints.size(); i++) {
            m_curvePoints[i] *= scaling;
        }
    }

    template <int D>
    void LIC<D>::translate(glm::vec<D, float> translation) {
        for (int i = 0; i < m_curvePoints.size(); i++) {
            m_curvePoints[i] += translation;
        }
    }

    template <int D>
    void LIC<D>::rotate(glm::fquat rotation) {
        MONA_ASSERT(D == 3, "LIC: Quaternion rotation is only available for dimension 3 LICs.");
        for (int i = 0; i < m_curvePoints.size(); i++) {
            m_curvePoints[i] = glm::rotate(rotation, glm::vec4(m_curvePoints[i], 1));
        }
    }
    
}