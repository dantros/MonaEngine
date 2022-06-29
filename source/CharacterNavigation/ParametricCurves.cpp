#include "ParametricCurves.hpp"
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"
#include <math.h>

namespace Mona{


    DiscreteCurve::DiscreteCurve(std::vector<glm::vec3> curvePoints, std::vector<float> tValues) {
        MONA_ASSERT(1 < curvePoints.size(), "DiscreteCurve: must provide at least two points.");
        // chequeamos que los tValues vengan correctamente ordenados
        for (int i = 1; i < tValues.size(); i++) {
            if (!(tValues[i - 1] < tValues[i])) {
                MONA_LOG_ERROR("DiscreteCurve: tValues must come in a strictly ascending order");
                return;
            }
        }
        MONA_ASSERT(curvePoints.size() == tValues.size(), "DiscreteCurve: there must be exactly one tValue per spline point.");
        m_curvePoints = curvePoints;
        m_tValues = tValues;
    }

    glm::vec2 DiscreteCurve::getTRange() {
        return glm::vec2(m_tValues[0], m_tValues.back());
    }

    bool DiscreteCurve::inTRange(float t) {
        auto tRange = getTRange();
        return tRange[0] <= t && t <= tRange[1];
    }

    glm::vec3 DiscreteCurve::getVelocity(int pointIndex) {
        MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "DiscreteCurve: input index must be within bounds");
        if (pointIndex == 0) { return glm::vec3(0); }
        return (m_curvePoints[pointIndex] - m_curvePoints[pointIndex - 1]) / (m_tValues[pointIndex] - m_tValues[pointIndex - 1]);
    }

    void DiscreteCurve::displacePointT(int pointIndex, float newT, bool scalePositions) {
        MONA_ASSERT(inTRange(newT), "DiscreteCurve: new t must be within original t bounds");
        glm::vec2 tRange = getTRange();
        float oldT = m_tValues[pointIndex];
        float fractionBelow = (newT - tRange[0]) / (oldT - tRange[0]);
        float fractionAbove = (newT - tRange[1]) / (oldT - tRange[1]);
        for (int i = 0; i < pointIndex; i++) {
            tRange[i] = funcUtils::lerp(tRange[0], tRange[i], fractionBelow);
            if (scalePositions) {
                m_curvePoints[i] = funcUtils::lerp(m_curvePoints[0], m_curvePoints[i], fractionBelow);
            }
        }
        for (int i = pointIndex; i < m_curvePoints.size(); i++) {
            tRange[i] = funcUtils::lerp(tRange[1], tRange[i], fractionAbove);
            if (scalePositions) {
                m_curvePoints[i] = funcUtils::lerp(m_curvePoints.back(), m_curvePoints[i], fractionAbove);
            }
        }
    }

    void DiscreteCurve::setCurvePoint(int pointIndex, glm::vec3 newValue) {
        MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "DiscreteCurve: input index must be within bounds");
        m_curvePoints[pointIndex] = newValue;
    }
    glm::vec3 DiscreteCurve::evalCurve(int pointIndex) {
        MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "DiscreteCurve: input index must be within bounds");
        return m_curvePoints[pointIndex];
    }




    // Solucion extraida de https://stackoverflow.com/questions/55421835/c-binomial-coefficient-is-too-slow
    // autor: BiagioF
    int BinomialCoefficient(const int n, const int k) {
        MONA_ASSERT(0<=k && k<=n, "BinomialCoefficient: it must be true that 0<=k<=n.");
        std::vector<int> aSolutions(k);
        aSolutions[0] = n - k + 1;
        for (int i = 1; i < k; ++i) {
            aSolutions[i] = aSolutions[i - 1] * (n - k + 1 + i) / (i + 1);
        }
        return aSolutions[k - 1];
    }

    BezierCurve::BezierCurve(int order, std::vector<glm::vec3> controlPoints,float minT, float maxT):
        m_order(order), m_controlPoints(controlPoints), m_minT(minT), m_maxT(maxT){
        MONA_ASSERT(order >= 1,
            "BezierCurve: Order must be at least 1.");
        MONA_ASSERT(controlPoints.size() == order + 1,
            "BezierCurve: Number of points provided does not fit the order. Points must be order plus 1.");
        MONA_ASSERT(m_minT < m_maxT, "BezierCurve: maxT must be greater than minT.");
    }

    float BezierCurve::bernsteinBP(int i, int n, float t) {
        return BinomialCoefficient(n, i) * std::pow(t, i) * std::pow((1 - t), (n - i));
    }
    float BezierCurve::normalizeT(float t) {
        if (!inTRange(t)) {
            MONA_LOG_ERROR("BezierCurve: t must be a value between {0} and {1}.", m_minT, m_maxT);
        }
        return (t - m_minT) / (m_maxT - m_minT);
    }
    glm::vec3 BezierCurve::evalCurve(float t) {
        glm::vec3 result = { 0,0,0 };
        int n = m_order;
        t = normalizeT(t);
        for (int i = 0; i <= n; i++) {
            result += bernsteinBP(i, n, t)*m_controlPoints[i];
        }
        return result;
    }
    glm::vec3 BezierCurve::getVelocity(float t) {
        glm::vec3 result = { 0,0,0 };
        int n = m_order;
        t = normalizeT(t);
        for (int i = 0; i <= n-1; i++) {
            result += bernsteinBP(i, n-1, t) * (m_controlPoints[i+1] - m_controlPoints[i]);
        }
        result *= n;
        return result;
    }

    void BezierCurve::rotate(glm::fquat rotation) {
        for (int i = 0; i < m_controlPoints.size(); i++) {
            m_controlPoints[i] = glm::rotate(rotation, glm::vec4(m_controlPoints[i], 1));
        }
    }
    void BezierCurve::scale(glm::vec3 scaling) {
        for (int i = 0; i < m_controlPoints.size(); i++) {
            m_controlPoints[i] *= scaling;
        }
    }
    void BezierCurve::translate(glm::vec3 translation) {
        for (int i = 0; i < m_controlPoints.size(); i++) {
            m_controlPoints[i] += translation;
        }
    }

    template <typename T>
    std::vector<T> triDiagonalMatrixSolver(std::vector<float> diagA, std::vector<float> diagB,
        std::vector<float> diagC, std::vector<T> dVector) {
        // n valores indexados de 0 a n-1
        int n = dVector.size();
        std::vector<T> xVector(n);
        float w;
        for (int i = 1; i <= n-1; i++) {
            w = diagA[i] / diagB[i - 1];
            diagB[i] = diagB[i] - w * diagC[i - 1];
            dVector[i] = dVector[i] - w * dVector[i - 1];
        }
        xVector[n - 1] = dVector[n - 1] / diagB[n - 1];
        for (int i = n-2; 0 <= i; i--) {
            xVector[i] = (dVector[i] - diagC[i] * xVector[i + 1])/diagB[i];
        }
        return xVector;
    }


    BezierSpline::BezierSpline(std::vector<glm::vec3> splinePoints, std::vector<float> tValues, Order order) {
        MONA_ASSERT(1 < splinePoints.size(), "BezierSpline: must provide at least two points.");
        m_minT = tValues[0];
        m_maxT = tValues.back();
        m_order = order;
        // chequeamos que los tValues vengan correctamente ordenados
        for (int i = 1; i < tValues.size(); i++) {
            MONA_ASSERT(!(tValues[i - 1] < tValues[i]), "BezierSpline: tValues must come in a strictly ascending order");
            return;
        }
        MONA_ASSERT(splinePoints.size() == tValues.size(), "BezierSpline: there must be exactly one tValue per spline point.");
        if (order == Order::LINEAR) {
            // creamos las curvas directamente
            m_bezierCurves = std::vector<BezierCurve>(tValues.size() - 1);
            for (int i = 0; i < tValues.size() - 1; i++) {
                m_bezierCurves[i] = BezierCurve(1, { splinePoints[i], splinePoints[i + 1] }, tValues[i], tValues[i + 1]);
            }
        }
        else if (order == Order::CUBIC) {
            // en splinePoints recibimos los puntos por los que pasara la curva, osea los extremos P0 y P4 de cada sub curva de bezier
            // generamos los puntos de control faltantes P1 y P2 para cada segmento
            // Se tienen n+1 puntos conocidos K , o knots, que son extremos de los segmentos
            // hay una ecuacion para cada uno de los n segmentos
            // 2P(1, 0) + P(1,1) = K(O) + 2K(1)
            // P(1,i-1) + 4P(1,i) + P(1, i+1) = 4K(i) + 2K(i+1)  i pertenece a [1, n-2]
            // 2P(1,n-2) + 7P(1,n-1) = 8K(n-1) + K(n)
            // primero se obtiene P1 para cada segmento
            int n = splinePoints.size() - 1;
            std::vector<float> diagA(n - 1);
            std::vector<float> diagB(n);
            std::vector<float> diagC(n - 1);
            std::vector<glm::vec3> dVector(n);
            diagB[0] = 2;
            diagC[0] = 1;
            dVector[0] = splinePoints[0] + 2.0f * splinePoints[1];
            for (int i = 1; i <= n - 2; i++) {
                diagA[i] = 1;
                diagB[i] = 4;
                diagC[i] = 1;
                dVector[i] = 4.0f * splinePoints[i] + 2.0f * splinePoints[i + 1];
            }
            diagA[n - 1] = 2;
            diagB[n - 1] = 7;
            dVector[n - 1] = 8.0f * splinePoints[n - 1] + 2.0f * splinePoints[n];

            std::vector<glm::vec3> p1Values(n);
            std::vector<glm::vec3> p2Values(n);
            p1Values = triDiagonalMatrixSolver<glm::vec3>(diagA, diagB, diagC, dVector);
            for (int i = 0; i <= n - 2; i++) {
                p2Values[i] = 2.0f * splinePoints[i] - p1Values[i];
            }
            p2Values[n - 1] = 0.5f * (splinePoints[n] + p1Values[n - 1]);

            // ahora podemos crear las curvas
            m_bezierCurves = std::vector<BezierCurve>(n);
            std::vector<glm::vec3> controlPoints;
            for (int i = 0; i < n; i++) {
                controlPoints = { splinePoints[i], p1Values[i], p2Values[i], splinePoints[i + 1] };
                m_bezierCurves[i] = BezierCurve(3, controlPoints, tValues[i], tValues[i + 1]);
            }
        }
    }

    glm::vec3 BezierSpline::evalSpline(float t) {
        for (int i = 0; i < m_bezierCurves.size(); i++) {
            if (m_bezierCurves[i].inTRange(t)) {
                return m_bezierCurves[i].evalCurve(t);
            }
        }
        MONA_LOG_ERROR("BezierSpline: t value is not in range");
        return glm::vec3(0);
    }

    glm::vec3 BezierSpline::getVelocity(float t) {
        for (int i = 0; i < m_bezierCurves.size(); i++) {
            if (m_bezierCurves[i].inTRange(t)) {
                return m_bezierCurves[i].getVelocity(t);
            }
        }
        MONA_LOG_ERROR("BezierSpline: t value is not in range");
        return glm::vec3(0);
    }

    int BezierSpline::getSplinePointNum(float minT, float maxT) {
        MONA_ASSERT(minT < maxT, "BezierSpline: maxT must be greater than minT.");
        int result = 0;
        int firstCurveIndex = 0;
        for (int i = 0; i < m_bezierCurves.size(); i++) {
            if (m_bezierCurves[i].inTRange(minT)) {
                firstCurveIndex = i;
                if (m_bezierCurves[i].getTRange()[0] == minT) {
                    result += 1;
                }
                break;
            }
        }
        for (int i = firstCurveIndex + 1; i < m_bezierCurves.size(); i++) {
            result += 1;
            if (m_bezierCurves[i].inTRange(maxT)) {
                if (m_bezierCurves[i].getTRange()[1] == maxT) {
                    result += 1;
                }
                break;
            }
        }
        return result;
    }

    DiscreteCurve BezierSpline::sampleDiscrete(float minT, float maxT, int innerSplinePointNumber) {
        MONA_ASSERT(minT < maxT, "BezierSpline: maxT must be greater than minT.");
        MONA_ASSERT(inTRange(minT) && inTRange(maxT), "BezierSpline: Both minT and maxT must be inside the spline's t range.");
        std::vector<glm::vec3> splinePoints(innerSplinePointNumber + 2);
        std::vector<float> tValues(innerSplinePointNumber + 2);
        for (int i = 0; i < tValues.size(); i++) {
            float fraction = i / (tValues.size() - 1);
            tValues[i] = funcUtils::lerp(minT, maxT, fraction);
            splinePoints[i] = evalSpline(tValues[i]);
        }
        return DiscreteCurve(splinePoints, tValues);
    }

    
}