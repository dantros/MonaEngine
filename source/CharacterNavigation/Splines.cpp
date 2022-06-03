#include "Splines.hpp"
#include "../Core/Log.hpp"
#include <math.h>

namespace Mona{

    // Solucion extraida de https://stackoverflow.com/questions/55421835/c-binomial-coefficient-is-too-slow
    // autor: BiagioF
    int BinomialCoefficient(const int n, const int k) {
        std::vector<int> aSolutions(k);
        aSolutions[0] = n - k + 1;
        for (int i = 1; i < k; ++i) {
            aSolutions[i] = aSolutions[i - 1] * (n - k + 1 + i) / (i + 1);
        }
        return aSolutions[k - 1];
    }

    BezierCurve::BezierCurve(int order, std::vector<glm::vec3> controlPoints):m_order(order), m_controlPoints(controlPoints){
        MONA_ASSERT(order >= 1,
            "BezierCurve: Order must be at least 1.");
        MONA_ASSERT(controlPoints.size() == order + 1,
            "BezierCurve: Number of points provided does not fit the order. Points must be order plus 1.");
    }

    float BezierCurve::BernsteinBP(int i, int n, float t) {
        return BinomialCoefficient(n, i) * std::pow(t, i) * std::pow((1 - t), (n - i));
    }

    glm::vec3 BezierCurve::evalCurve(float t) {
        glm::vec3 result = { 0,0,0 };
        int n = m_order;
        for (int i = 0; i <= n; i++) {
            result += BernsteinBP(i, n, t)*m_controlPoints[i];
        }
        return result;
    }
    glm::vec3 BezierCurve::getVelocity(float t) {
        glm::vec3 result = { 0,0,0 };
        int n = m_order;
        for (int i = 0; i <= n-1; i++) {
            result += BernsteinBP(i, n-1, t) * (m_controlPoints[i+1] - m_controlPoints[i]);
        }
        result *= n;
        return result;
    }


    
}