#pragma once
#ifndef PARAMETRICCURVES_HPP
#define PARAMETRICCURVES_HPP

#include "glm/glm.hpp"
#include <glm/gtx/quaternion.hpp>
#include <vector>
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"
#include <math.h>

namespace Mona{

    template <int D>
    // linearly interpolated curve
    class LIC {
    private:
        // puntos de la curva
        std::vector<glm::vec<D, float>> m_curvePoints;
        // valores de t
        std::vector<float> m_tValues = { 1,0 };
        // dimension de los puntos
        int m_dimension = 0;
    public:
        glm::vec2 getTRange() { return glm::vec2({ m_tValues[0], m_tValues.back() }); }
        bool inTRange(float t) { return m_tValues[0] <= t && t <= m_tValues.back(); }
        bool inOpenTRange(float t) { return m_tValues[0] < t && t < m_tValues.back(); }
        bool inOpenRightTRange(float t) { return m_tValues[0] <= t && t < m_tValues.back(); }
        bool inOpenLeftTRange(float t) { return m_tValues[0] < t && t <= m_tValues.back(); }
        int getNumberOfPoints() const { return m_curvePoints.size(); }
        int getDimension() { return m_dimension; }
        glm::vec<D, float> getStart() { return m_curvePoints[0]; }
        glm::vec<D, float> getEnd() { return m_curvePoints.back(); }
        LIC() = default;
        LIC(std::vector<glm::vec<D, float>> curvePoints, std::vector<float> tValues) {
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

        glm::vec<D, float> getVelocity(float t) {
            MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
            if (t == m_tValues[0]) { return glm::vec<D, float>(0); }
            for (int i = 0; i < m_tValues.size(); i++) {
                if (m_tValues[i] == t) {
                    return (m_curvePoints[i] - m_curvePoints[i - 1]) / (m_tValues[i] - m_tValues[i - 1]);
                }
                else if (m_tValues[i] < t && t < m_tValues[i + 1]) {
                    return (m_curvePoints[i + 1] - m_curvePoints[i]) / (m_tValues[i + 1] - m_tValues[i]);
                }
            }
            return glm::vec<D, float>(0);;
        }

        glm::vec<D, float> getAcceleration(int pointIndex) {
            MONA_ASSERT(0 < pointIndex && pointIndex < m_tValues.size() - 1, "LIC: pointIndex must be an inner point.");
            return (getVelocity(getTValue(pointIndex + 1)) - getVelocity(getTValue(pointIndex))) / (getTValue(pointIndex + 1) - getTValue(pointIndex));
        }

        glm::vec<D, float> evalCurve(float t) {
            MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
            for (int i = 0; i < m_tValues.size() - 1; i++) {
                if (m_tValues[i] <= t && t <= m_tValues[i + 1]) {
                    float fraction = funcUtils::getFraction(m_tValues[i], m_tValues[i + 1], t);
                    return funcUtils::lerp(m_curvePoints[i], m_curvePoints[i + 1], fraction);
                }
            }
            return glm::vec<D, float>(0);
        }

        void displacePointT(int pointIndex, float newT, bool scalePoints = true, float pointScalingRatio = 1) {
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

        void setCurvePoint(int pointIndex, glm::vec<D, float> newValue) {
            MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "LIC: input index must be within bounds");
            m_curvePoints[pointIndex] = newValue;
        }

        LIC<D> sample(float minT, float maxT) {
            MONA_ASSERT(minT < maxT, "LIC: maxT must be greater than minT.");
            MONA_ASSERT(inTRange(minT) && inTRange(maxT), "LIC: Both minT and maxT must be in t range.")
                std::vector<float> sampleTValues;
            sampleTValues.reserve(m_tValues.size());
            std::vector<glm::vec<D, float>> samplePoints;
            samplePoints.reserve(m_tValues.size());
            int startInd = 0;
            for (int i = 0; i < m_tValues.size() - 1; i++) {
                if (m_tValues[i] <= minT  && minT < m_tValues[i + 1]) {
                    sampleTValues.push_back(minT);
                    samplePoints.push_back(evalCurve(minT));
                    startInd = i + 1;
                    break;
                }
            }
            for (int i = startInd; i < m_tValues.size() - 1; i++) {
                sampleTValues.push_back(m_tValues[i]);
                samplePoints.push_back(m_curvePoints[i]);
                if (m_tValues[i] < maxT && maxT <= m_tValues[i + 1]) {
                    sampleTValues.push_back(maxT);
                    samplePoints.push_back(evalCurve(maxT));
                    break;
                }
            }
            return LIC(samplePoints, sampleTValues);
        }

        void scale(glm::vec<D, float> scaling) {
            for (int i = 0; i < m_curvePoints.size(); i++) {
                m_curvePoints[i] *= scaling;
            }
        }

        void translate(glm::vec<D, float> translation) {
            for (int i = 0; i < m_curvePoints.size(); i++) {
                m_curvePoints[i] += translation;
            }
        }

        void rotate(glm::fquat rotation) {
            MONA_ASSERT(D == 3, "LIC: Rotation is only valid for dimension 3 LICs.");
            for (int i = 0; i < m_curvePoints.size(); i++) {
                m_curvePoints[i] = glm::rotate(rotation, glm::vec4(m_curvePoints[i], 1));
            }
        }

        void offsetTValues(float offset) {
            for (int i = 0; i < m_tValues.size(); i++) {
                m_tValues[i] += offset;
            }
        }

        static LIC<D> join(const LIC& curve1, const LIC& curve2, bool preserveLeft=true) {
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
                        jointTValues.insert(jointTValues.end(), curve2.m_tValues.begin(), curve2.m_tValues.end());
                        jointCurvePoints.insert(jointCurvePoints.end(), curve2.m_curvePoints.begin(), curve2.m_curvePoints.end());
                        break;
                    }
                    jointTValues.push_back(curve1.m_tValues[i]);
                    jointCurvePoints.push_back(curve1.m_curvePoints[i]);
                }
            }
            return LIC(jointCurvePoints, jointTValues);
        }

        static LIC<D> transition(const LIC& curve1, const LIC& curve2, float transitionT) {
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
                if (transitionT <= curve2.m_tValues[i]) {
                    transitionTValues.push_back(curve2.m_tValues[i]);
                    transitionCurvePoints.push_back(curve2.m_curvePoints[i]);
                }
            }
            return LIC(transitionCurvePoints, transitionTValues);
        }

        float getTValue(int pointIndex) const {
            MONA_ASSERT(0 <= pointIndex && pointIndex < m_tValues.size(), "LIC: input index must be within bounds.");
            return m_tValues[pointIndex];
        };

        glm::vec<D, float> getCurvePoint(int pointIndex) const {
            MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "LIC: input index must be within bounds.");
            return m_curvePoints[pointIndex];
        };


        int getClosestPointIndex(float tValue) const {
            if (tValue < m_tValues[0]) {
                return 0;
            }
            for (int i = 1; i < m_tValues.size() - 1; i++) {
                if (m_tValues[i] <= tValue && tValue <= m_tValues[i + 1]) {
                    if ((m_tValues[i + 1] - tValue) <= (tValue - m_tValues[i])) { return i + 1; }
                    else { return i; }
                }
            }
            return m_tValues.size() - 1;
        }

        int getPointIndex(float tValue, bool getNext=false) const {
            if (tValue < m_tValues[0]) {
                if (!getNext) {
                    MONA_LOG_ERROR("LIC: There is no point at t={0} or before", tValue);
                }
                return 0;
            }
            for (int i = 1; i < m_tValues.size() - 1; i++) {
                if (m_tValues[i] <= tValue && tValue <= m_tValues[i + 1]) {
                    if (tValue == m_tValues[i]) { return i; }
                    if (tValue == m_tValues[i + 1]) { return i + 1; }
                    if (getNext) { return i + 1; }
                    return i;
                }
            }
            if (getNext) {
                MONA_LOG_ERROR("LIC: There is no point at t={0} or after", tValue);
            }
            return m_tValues.size() - 1;
        }
    };
    
}





#endif