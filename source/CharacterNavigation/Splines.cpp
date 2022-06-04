#include "Splines.hpp"
#include "../Core/Log.hpp"
#include <math.h>

namespace Mona{

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
        MONA_ASSERT(m_minT < m_maxT, "maxT must be greater than minT.");
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

    template <typename T>
    std::vector<T> triDiagonalMatrixSolver(std::vector<float> diagA, std::vector<float> diagB,
        std::vector<float> diagC, std::vector<T> dVector) {
        // n valores indexados de 0 a n-1
        int n = dVector.size();
        std::vector<T> xVector(n);
        int w;
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


    CubicBezierSpline::CubicBezierSpline(std::vector<glm::vec3> splinePoints, std::vector<float> tValues) {
        // en splinePoints recibimos los puntos por los que pasara la curva, osea los extremos P0 y P4 de cada sub curva de bezier
        // generamos los puntos de control faltantes P1 y P2 para cada segmento
        // Se tienen n+1 puntos conocidos K , o knots, que son extremos de los segmentos
        // hay una ecuacion para cada uno de los n segmentos
        // 2P(1, 0) + P(1,1) = K(O) + 2K(1)
        // P(1,i-1) + 4P(1,i) + P(1, i+1) = 4K(i) + 2K(i+1)  i pertenece a [1, n-2]
        // 2P(1,n-2) + 7P(1,n-1) = 8K(n-1) + K(n)
        // primero se obtiene P1 para cada segmento
        int n = splinePoints.size() - 1;
        std::vector<float> diagA(n-1);
        std::vector<float> diagB(n);
        std::vector<float> diagC(n-1);
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
        diagA[n-1] = 2;
        diagB[n-1] = 7;
        dVector[n-1] = 8.0f*splinePoints[n-1] + 2.0f * splinePoints[n];

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
    };


    
}