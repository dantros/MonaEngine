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
    glm::vec<D, float> LIC<D>::getVelocity(float t) {
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
    glm::vec<D, float> LIC<D>::getAcceleration(int pointIndex) {
        MONA_ASSERT(0 < pointIndex && pointIndex < m_tValues.size() - 1), "LIC: pointIndex must be an inner point.";
        return (getVelocity(getTValue(pointIndex + 1)) - getVelocity(getTValue(pointIndex)))/(getTValue(pointIndex + 1) - getTValue(pointIndex));
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
    void LIC<D>::displacePointT(int pointIndex, float newT, bool scalePoints, float pointScalingRatio) {
        MONA_ASSERT(inTRange(newT), "LIC: newT must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
        glm::vec2 tRange = getTRange();
        float oldT = m_tValues[pointIndex];
        float fractionBelow = funcUtils::getFraction(tRange[0], oldT, newT);
        float fractionAbove = funcUtils::getFraction(tRange[1], oldT, newT);
        for (int i = 0; i < pointIndex; i++) {
            tRange[i] = funcUtils::lerp(tRange[0], tRange[i], fractionBelow);
            if (scalePoints) {
                m_curvePoints[i] = funcUtils::lerp(m_curvePoints[0], m_curvePoints[i], fractionBelow * pointScalingRatio);
            }
        }
        for (int i = pointIndex; i < m_curvePoints.size(); i++) {
            tRange[i] = funcUtils::lerp(tRange[1], tRange[i], fractionAbove);
            if (scalePoints) {
                m_curvePoints[i] = funcUtils::lerp(m_curvePoints.back(), m_curvePoints[i], fractionAbove * pointScalingRatio);
            }
        }
    }

    template <int D>
    void LIC<D>::setCurvePoint(int pointIndex, glm::vec<D, float> newValue) {
        MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "LIC: input index must be within bounds");
        m_curvePoints[pointIndex] = newValue;
    }

    template <int D>
    LIC<D> LIC<D>::sample(float minT, float maxT) {
        MONA_ASSERT(minT < maxT, "LIC: maxT must be greater than minT.");
        MONA_ASSERT(inTRange(minT) && inTRange(maxT), "LIC: Both minT and maxT must be in t range.")
        std::vector<float> sampleTValues;
        sampleTValues.reserve(m_tValues.size());
        std::vector<glm::vec<D, float>> samplePoints;
        samplePoints.reserve(m_tValues.size());
        int startInd = 0;
        for (int i = 0; i < m_tValues.size() - 1; i++) {
            if (m_tValues[i] <= minT < m_tValues[i + 1]) {
                sampleTValues.push_back(minT);
                samplePoints.push_back(evalCurve(minT));
                startInd = i + 1;
                break;
            }
        }
        for (int i = startInd; i < m_tValues.size() - 1; i++) {
            sampleTValues.push_back(m_tValues[i]);
            samplePoints.push_back(m_curvePoints[i]);
            if (m_tValues[i] < maxT <= m_tValues[i + 1]) {
                sampleTValues.push_back(maxT);
                samplePoints.push_back(evalCurve(maxT));
                break;
            }
        }
        return LIC(samplePoints, sampleTValues);
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

    void LIC<3>::rotate(glm::fquat rotation) {
        for (int i = 0; i < m_curvePoints.size(); i++) {
            m_curvePoints[i] = glm::rotate(rotation, glm::vec4(m_curvePoints[i], 1));
        }
    }

    template <int D>
    void LIC<D>::offsetTValues(float offset) {
        for (int i = 0; i < m_tValues.size(); i++) {
            m_tValues[i] += offset;
        }
    }

    template <int D>
    static LIC<D> LIC<D>::join(const LIC& curve1, const LIC& curve2, bool preserveLeft) {
        std::vector<float> jointTValues;
        jointTValues.reserve(curve1.m_curvePoints.size() + curve2.m_curvePoints.size());
        std::vector<glm::vec<D, float>> jointCurvePoints;
        jointCurvePoints.reserve(curve1.m_curvePoints.size() + curve2.m_curvePoints.size());
        if (preserveLeft) {
            jointTValues = curve1.m_tValues;
            jointCurvePoints = curve1.m_curvePoints;
            for (int i = 0; i < curve2.m_tValues.size(); i++) {
                if (curve1.m_tValues.back() < curve2.m_tValues[i]) {
                    jointTValues.insert(jointTValues.end(), curve2.m_tValues.begin() + i, curve2.m_tValues.end());
                    jointCurvePoints.insert(jointCurvePoints.end(), curve2.m_curvePoints.begin() + i, curve2.m_curvePoints.end());
                    break;
                }
            }
        }
        else {
            for (int i = 0; i < curve1.m_tValues.size(); i++) {
                if (curve2.m_tValues[0] <= curve1.m_tValues[i]) {
                    jointTValues.insert(jointTValues.end(), curve2.m_tValues);
                    jointCurvePoints.insert(jointCurvePoints.end(), curve2.m_curvePoints);
                    break;
                }
                jointTValues.push_back(curve1.m_tValues[i]);
                jointCurvePoints.push_back(curve1.m_curvePoints[i]);
            }
        }
        return LIC(jointCurvePoints, jointTValues);
    }

    template <int D>
    static LIC<D> LIC<D>::transition(const LIC& curve1, const LIC& curve2, float transitionT) {
        std::vector<float> transitionTValues;
        transitionTValues.reserve(curve1.m_curvePoints.size() + curve2.m_curvePoints.size());
        std::vector<glm::vec<D, float>> transitionCurvePoints;
        transitionCurvePoints.reserve(curve1.m_curvePoints.size() + curve2.m_curvePoints.size());
        for (int i = 0; i < curve1.m_tValues.size(); i++) {
            if (curve1.m_tValues[i] < transitionT) {
                transitionTValues.push_back(curve1.m_tValues[i]);
                transitionCurvePoints.push_back(curve1.m_curvePoints[i]);
            }
            else { break; }
        }
        for (int i = 0; i < curve2.m_tValues.size(); i++) {
            if ( transitionT <= curve2.m_tValues[i]) {
                transitionTValues.push_back(curve2.m_tValues[i]);
                transitionCurvePoints.push_back(curve2.m_curvePoints[i]);
            }
        }
        return LIC(transitionCurvePoints, transitionTValues);
    }

    template <int D>
    float LIC<D>::getTValue(int pointIndex) const {
        MONA_ASSERT(0 <= pointIndex && pointIndex < m_tValues.size(), "LIC: input index must be within bounds.");
        return m_tValues[pointIndex];
    };
    template <int D>
    glm::vec<D, float> LIC<D>::getCurvePoint(int pointIndex) const { 
        MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "LIC: input index must be within bounds.");
        return m_curvePoints[pointIndex]; 
    };

    template <int D>
    int LIC<D>::getClosestPointIndex(float tValue) const {
        if (tValue < m_tValues[0]) {
            return 0;
        }
        for (int i = 1; i < m_tValues.size() - 1; i++) {
            if (m_tValues[i] <= tValue && tValue <= m_tValues[i + 1]) {
                if ((m_tValues[i + 1] - tValue) <= (tValue - m_tValues[i]) {
                    return i+1;
                }
                else {
                    return i;
                }
            }
        }
        return m_tValues.size() - 1;
    }
    
}