#pragma once
#ifndef PARAMETRICCURVES_HPP
#define PARAMETRICCURVES_HPP

#include "glm/glm.hpp"
#include <glm/gtx/quaternion.hpp>
#include "glm/gtx/vector_angle.hpp"
#include <vector>
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"
#include "../Core/GlmUtils.hpp"
#include <math.h>
#include <numbers>

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
        float m_tEpsilon;
		glm::vec<D, float> getRightHandVelocity(float t) {
			MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
			if (m_tValues.back() <= t && t - m_tValues.back() <= m_tEpsilon) { return glm::vec<D, float>(0); }

			// si estamos en el entorno de un punto
			if (abs(m_tValues.back() - t) <= m_tEpsilon) {
				return (m_curvePoints[m_tValues.size() - 1] - m_curvePoints[m_tValues.size()-2]) / 
                    (m_tValues[m_tValues.size() - 1] - m_tValues[m_tValues.size() - 2]);
			}
			for (int i = 0; i < m_tValues.size() - 1; i++) {
				if (abs(m_tValues[i] - t) <= m_tEpsilon) {
                    return (m_curvePoints[i + 1] - m_curvePoints[i]) / (m_tValues[i + 1] - m_tValues[i]);
				}
			}

			// si no
			for (int i = 0; i < m_tValues.size() - 1; i++) {
				if (m_tValues[i] <= t && t <= m_tValues[i + 1]) {
					return (m_curvePoints[i + 1] - m_curvePoints[i]) / (m_tValues[i + 1] - m_tValues[i]);
				}
			}
			return glm::vec<D, float>(0);;
		}

        bool tValuesAreValid() {
			for (int i = 1; i < m_tValues.size(); i++) {
				if (m_tValues[i] - m_tValues[i - 1] <= 2 * m_tEpsilon) {
                    return false;
				}
			}
            return true;
        }

		bool correctTValues() {
			bool needCorrection = !tValuesAreValid();
			if (!needCorrection) {
				return true;
			}

			// correccion de valores
            // de izquierda a derecha
			std::vector<int> targetIndexesLeftToRight;
			for (int i = 1; i < getNumberOfPoints()-1; i++) {
				if (m_tValues[i] - m_tValues[i - 1] <= 2*m_tEpsilon) {
					targetIndexesLeftToRight.push_back(i);
				}
			}
			for (int i = targetIndexesLeftToRight.size() - 1; 0 <= i; i--) {
				int tIndex = targetIndexesLeftToRight[i];
				m_tValues[tIndex] = (m_tValues[tIndex] + m_tValues[tIndex + 1]) / 2.0f;
				m_curvePoints[tIndex] = (m_curvePoints[tIndex] + m_curvePoints[tIndex + 1]) / 2.0f;
			}
            // de derecha a izquierda
			std::vector<int> targetIndexesRightToLeft;
			for (int i = getNumberOfPoints() - 2; 0 <= i; i--) {
				if (m_tValues[i + 1] - m_tValues[i] <= 2*m_tEpsilon) {
					targetIndexesLeftToRight.push_back(i);
				}
			}
			for (int i = targetIndexesRightToLeft.size() - 1; 0 <= i; i--) {
				int tIndex = targetIndexesRightToLeft[i];
				m_tValues[tIndex] = (m_tValues[tIndex] + m_tValues[tIndex - 1]) / 2.0f;
				m_curvePoints[tIndex] = (m_curvePoints[tIndex] + m_curvePoints[tIndex - 1]) / 2.0f;
			}

			// chequeo final
            return tValuesAreValid();
		}
    public:
        glm::vec2 getTRange() const { return glm::vec2({ m_tValues[0], m_tValues.back() }); }
        bool inTRange(float t) const { return m_tValues[0]-m_tEpsilon <= t && t <= m_tValues.back()+m_tEpsilon; }
        int getNumberOfPoints() const { return m_curvePoints.size(); }
        int getDimension() const { return m_dimension; }
        float getTEpsilon() const { return m_tEpsilon; }
        glm::vec<D, float> getStart() { return m_curvePoints[0]; }
        glm::vec<D, float> getEnd() { return m_curvePoints.back(); }
        LIC() = default;
        LIC(std::vector<glm::vec<D, float>> curvePoints, std::vector<float> tValues, float tEpsilon = 0.0001) {
            MONA_ASSERT(1 < curvePoints.size(), "LIC: must provide at least two points.");
            MONA_ASSERT(curvePoints.size() == tValues.size(), "LIC: there must be exactly one tValue per spline point.");
            MONA_ASSERT(0 < tEpsilon, "LIC: tEpsilon must be greater than 0.");
            m_tEpsilon = tEpsilon;
            m_tValues = tValues;
            // chequeamos que los tValues vengan correctamente ordenados
            MONA_ASSERT(tValuesAreValid(), "LIC: tValues must come in a strictly ascending order and differ in more than 2*tEpsilon.");
            m_curvePoints = curvePoints;
            m_dimension = D;
        }

        glm::vec<D, float> getVelocity(float t, bool rightHandVelocity = false) {
            if (rightHandVelocity) {
                return getRightHandVelocity(t);
            }
            MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
            if (t <= m_tValues[0] && abs(m_tValues[0] - t) <= m_tEpsilon) { return glm::vec<D, float>(0); }
            // si estamos en el entorno de un punto
			if (abs(t - m_tValues[0]) <= m_tEpsilon) {
				return (m_curvePoints[1] - m_curvePoints[0]) / (m_tValues[1] - m_tValues[0]);
			}
            for (int i = 1; i < m_tValues.size(); i++) {
                if (abs(m_tValues[i] - t) <= m_tEpsilon) {
                    return (m_curvePoints[i] - m_curvePoints[i - 1]) / (m_tValues[i] - m_tValues[i - 1]);
                }
            }

            // si no
            for (int i = 0; i < m_tValues.size() - 1; i++) {
                if (m_tValues[i] <= t && t <= m_tValues[i + 1]) {
                    return (m_curvePoints[i + 1] - m_curvePoints[i]) / (m_tValues[i + 1] - m_tValues[i]);
                }                
            }
            return glm::vec<D, float>(0);
        }

        glm::vec<D, float> getPointVelocity(int pointIndex, bool rightHandVelocity = false) {
            MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "LIC: input index must be within bounds");
            if (!rightHandVelocity) {
                if (pointIndex == 0) {
                    return glm::vec3(0);
                }
                return (m_curvePoints[pointIndex] - m_curvePoints[pointIndex - 1]) / (m_tValues[pointIndex] - m_tValues[pointIndex - 1]);
            }
            else {
                if (pointIndex == m_tValues.size() - 1) {
                    return glm::vec3(0);
                }
                return (m_curvePoints[pointIndex + 1] - m_curvePoints[pointIndex]) / (m_tValues[pointIndex + 1] - m_tValues[pointIndex]);
            }        
        }

        glm::vec<D, float> getPointAcceleration(int pointIndex) {
            MONA_ASSERT(0 < pointIndex && pointIndex < m_tValues.size() - 1, "LIC: pointIndex must be an inner point.");
            return (getVelocity(getTValue(pointIndex + 1)) - getVelocity(getTValue(pointIndex))) / (getTValue(pointIndex + 1) - getTValue(pointIndex));
        }

        glm::vec<D, float> evalCurve(float t) {
            MONA_ASSERT(inTRange(t), "LIC: t must be a value between {0} and {1}.", m_tValues[0], m_tValues.back());
            // si estamos en el entorno de uno punto
            for (int i = 0; i < m_tValues.size(); i++) {
                if (abs(t - m_tValues[i]) <= m_tEpsilon) {
                    return m_curvePoints[i];
                }
            }
            // si no
            for (int i = 0; i < m_tValues.size() - 1; i++) {
                if (m_tValues[i] <= t && t <= m_tValues[i + 1]) {
                    float fraction = funcUtils::getFraction(m_tValues[i], m_tValues[i + 1], t);
                    return funcUtils::lerp(m_curvePoints[i], m_curvePoints[i + 1], fraction);
                }
            }
            return glm::vec<D, float>(0);
        }

        

        void displacePointT(int pointIndex, int lowIndex, int highIndex, float newT, 
            bool scalePoints = true, float pointScalingRatio = 1, int maxCorrectionSteps = 3) {
            MONA_ASSERT(lowIndex < highIndex && 0 <= lowIndex && highIndex < m_tValues.size(), "LIC: low and high index must be within bounds.");
            MONA_ASSERT(lowIndex <= pointIndex && pointIndex <= highIndex, "LIC: input point index must be within input bounds");
            float oldT = m_tValues[pointIndex];
            if (oldT == newT) { return; }
            if (pointIndex != lowIndex) {
                MONA_ASSERT(getTValue(lowIndex) < newT,
                    "LIC: If not the lower end, newT cannot subceed or match original low t bound.");
                float fractionBelow = funcUtils::getFraction(m_tValues[lowIndex], oldT, newT);
                for (int i = lowIndex + 1; i <= pointIndex; i++) {
                    m_tValues[i] = funcUtils::lerp(m_tValues[lowIndex], m_tValues[i], fractionBelow);
                    if (scalePoints) {
                        m_curvePoints[i] = funcUtils::lerp(m_curvePoints[lowIndex], m_curvePoints[i], fractionBelow * pointScalingRatio);
                    }
                }
            }

            if (pointIndex != highIndex) {
                MONA_ASSERT(newT < getTValue(highIndex),
                    "LIC: If not the higher end, newT cannot exceed or match original high t bound.");
                float fractionAbove = funcUtils::getFraction(m_tValues[highIndex], oldT, newT);
                for (int i = pointIndex; i < highIndex; i++) {
                    if (pointIndex < i || pointIndex == lowIndex) {
                        m_tValues[i] = funcUtils::lerp(m_tValues[highIndex], m_tValues[i], fractionAbove);
                        if (scalePoints) {
                            m_curvePoints[i] = funcUtils::lerp(m_curvePoints[highIndex], m_curvePoints[i], fractionAbove * pointScalingRatio);
                        }
                    }
                }
            }

            // correccion de valores
            bool corrected = tValuesAreValid();
            int correctionSteps = 0;
            while (!corrected && correctionSteps < maxCorrectionSteps) {
                corrected = correctTValues();
                correctionSteps += 1;
            }
            MONA_ASSERT(corrected, "LIC: tValues were pushed too close together.");                       
        }

        void setCurvePoint(int pointIndex, glm::vec<D, float> newValue) {
            MONA_ASSERT(0 <= pointIndex && pointIndex < m_curvePoints.size(), "LIC: input index must be within bounds");
            m_curvePoints[pointIndex] = newValue;
        }

        LIC<D> sample(float minT, float maxT) {
            MONA_ASSERT(m_tEpsilon*2 <= maxT - minT, "LIC: maxT must be greater than minT by at least 2*tEpsilon.");
            MONA_ASSERT(inTRange(minT) && inTRange(maxT), "LIC: Both minT and maxT must be in t range.");
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
            for (int i = startInd; i < m_tValues.size(); i++) {
                if (m_tEpsilon*2 < maxT - m_tValues[i]) {
                    if (m_tEpsilon*2 < m_tValues[i] - minT) {
						sampleTValues.push_back(m_tValues[i]);
						samplePoints.push_back(m_curvePoints[i]);
                    }					
                }
                else {
					sampleTValues.push_back(maxT);
					samplePoints.push_back(evalCurve(maxT));
					break;
                }
            }
            if (samplePoints.size() == 0 || samplePoints.size() == 1) {
                samplePoints = { evalCurve(minT), evalCurve(maxT) };
                sampleTValues = { minT, maxT };
            }
            return LIC(samplePoints, sampleTValues, m_tEpsilon);
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

        static LIC<D> join(const LIC& curve1, const LIC& curve2) {
            MONA_ASSERT(curve1.m_tEpsilon == curve2.m_tEpsilon, "LIC: both curves must have the same tEpsilon.");
            float epsilon = curve1.m_tEpsilon;
            std::vector<float> jointTValues;
            jointTValues.reserve(curve1.m_curvePoints.size() + curve2.m_curvePoints.size());
            std::vector<glm::vec<D, float>> jointCurvePoints;
            jointCurvePoints.reserve(curve1.m_curvePoints.size() + curve2.m_curvePoints.size());
			jointTValues = curve1.m_tValues;
			jointCurvePoints = curve1.m_curvePoints;
			for (int i = 0; i < curve2.m_tValues.size(); i++) {
				if (2*epsilon < curve2.m_tValues[i] - curve1.m_tValues.back()) {
					jointTValues.insert(jointTValues.end(), curve2.m_tValues.begin() + i, curve2.m_tValues.end());
					jointCurvePoints.insert(jointCurvePoints.end(), curve2.m_curvePoints.begin() + i, curve2.m_curvePoints.end());
					break;
				}
			}
			return LIC(jointCurvePoints, jointTValues, epsilon);
            
        }

        static LIC<D> transition(const LIC& curve1, const LIC& curve2, float transitionT) {
            MONA_ASSERT(curve1.inTRange(transitionT) && curve2.inTRange(transitionT), "LIC: transitionT must be within t ranges of both curves.");
            MONA_ASSERT(curve1.m_tEpsilon == curve2.m_tEpsilon, "LIC: both curves must have the same tEpsilon.");
            float epsilon = curve1.m_tEpsilon;
            std::vector<float> transitionTValues;
            transitionTValues.reserve(curve1.m_curvePoints.size() + curve2.m_curvePoints.size());
            std::vector<glm::vec<D, float>> transitionCurvePoints;
            transitionCurvePoints.reserve(curve1.m_curvePoints.size() + curve2.m_curvePoints.size());
            for (int i = 0; i < curve1.m_tValues.size(); i++) {
                if (curve1.m_tValues[i] < transitionT || abs(curve1.m_tValues[i]-transitionT)<=2*epsilon) {
                    transitionTValues.push_back(curve1.m_tValues[i]);
                    transitionCurvePoints.push_back(curve1.m_curvePoints[i]);
                }
                else { break; }
            }
            for (int i = 0; i < curve2.m_tValues.size(); i++) {
                if (transitionT <= curve2.m_tValues[i] && 2*epsilon < curve2.m_tValues[i] - transitionTValues.back()) {
                    transitionTValues.push_back(curve2.m_tValues[i]);
                    transitionCurvePoints.push_back(curve2.m_curvePoints[i]);
                }
            }
            if (transitionTValues.size() == 1) {
                float extraTValue = transitionTValues[0];
                funcUtils::epsilonAdjustment_add(extraTValue, 3*epsilon);
                transitionTValues.push_back(extraTValue);
                transitionCurvePoints.push_back(transitionCurvePoints[0]);
            }           
            return LIC(transitionCurvePoints, transitionTValues, epsilon);
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
                    if (abs(m_tValues[i + 1] - tValue) <= abs(tValue - m_tValues[i])) { return i + 1; }
                    else { return i; }
                }
            }
            return m_tValues.size() - 1;
        }

        static LIC<D> connect(LIC<D> curve1, LIC<D> curve2, bool connectAtFront=true) {
            MONA_ASSERT(curve1.m_tEpsilon == curve2.m_tEpsilon, "LIC: both curves must have the same tEpsilon.");
            float epsilon = curve1.m_tEpsilon;
            std::vector<float> connectedTValues = curve1.m_tValues;
            std::vector<glm::vec<D, float>> connectedCurvePoints = curve1.m_curvePoints;
            if (connectAtFront) {
                curve2.translate(-curve2.getStart() + curve1.getEnd());
                curve2.offsetTValues(-curve2.getTRange()[0] + curve1.getTRange()[1]);
                for (int i = 1; i < curve2.m_tValues.size(); i++) {
                    connectedTValues.push_back(curve2.m_tValues[i]);
                    connectedCurvePoints.push_back(curve2.m_curvePoints[i]);
                }
            }
            else {
                curve2.translate(-curve2.getEnd() + curve1.getStart());
                curve2.offsetTValues(-curve2.getTRange()[1] + curve1.getTRange()[0]);
                for (int i = curve2.m_tValues.size()-2; 0 <= i ; i--) {
                    connectedTValues.insert(connectedTValues.begin(), curve2.m_tValues[i]);
                    connectedCurvePoints.insert(connectedCurvePoints.begin(), curve2.m_curvePoints[i]);
                }
            }
            return LIC<D>(connectedCurvePoints, connectedTValues);
            
            
        }

		static LIC<D> connectPoint(LIC<D> curve, glm::vec<D, float> extraPoint, float tDiff) {
            MONA_ASSERT(0 < tDiff, "LIC: tDiff must be greater than 0.");
            float epsilon = curve.m_tEpsilon;
            glm::vec<D, float> modifiedPoint = extraPoint;
			if (tDiff <= 2 * epsilon) {
				funcUtils::epsilonAdjustment_add(tDiff, 3 * epsilon);
                glm::vec<D, float> transitionVel = curve.getVelocity(curve.getTRange()[1]);
                modifiedPoint = curve.m_curvePoints.back() + transitionVel * tDiff;
			}
            float extraPointTValue = curve.getTRange()[1] + tDiff;
			curve.m_curvePoints.push_back(modifiedPoint);
			curve.m_tValues.push_back(extraPointTValue);
            return curve;	
		}

        void fitStartAndDir(glm::vec3 newStart, glm::vec3 targetDirection, glm::vec3 upVector = {0,0,1}) {
            MONA_ASSERT(m_dimension == 3, "LIC: LIC must have a dimension equal to 3.");
            translate(-getStart());
			// rotarla para que quede en linea con las pos inicial y final
			glm::fquat targetRotation = glm::identity<glm::fquat>();
			glm::vec3 originalDirection = glm::normalize(getEnd() - getStart());
            float epsilon = 0.0001;
            float dotPr = glm::dot(originalDirection, targetDirection);
            if (abs(dotPr - 1) <=epsilon) {
                targetRotation = angleAxis(0.0f, upVector);
            }
            else if (abs(dotPr + 1) <= epsilon) {
                targetRotation = angleAxis((float)std::numbers::pi, upVector);
            }
            else {
				glm::vec3 crossVec = glm::cross(originalDirection, targetDirection);
				glm::vec3 rotAxis = glm::normalize(crossVec);
                float rotAngle = glm::orientedAngle(originalDirection, targetDirection, rotAxis);
                targetRotation = angleAxis(rotAngle, rotAxis);
            }
			rotate(targetRotation);
			translate(newStart);
		}

        void fitEnds(glm::vec3 newStart, glm::vec3 newEnd, glm::vec3 upVector = {0,0,1}) {
			MONA_ASSERT(m_dimension == 3, "LIC: LIC must have a dimension equal to 3.");
            if (0 < glm::length(newEnd - newStart)) {
				glm::vec3 targetDirection = glm::normalize(newEnd - newStart);
				fitStartAndDir(newStart, targetDirection, upVector);

				// escalarla para que llegue a newEnd
				float origLength = glm::distance(getStart(), getEnd());
				float targetLength = glm::distance(newStart, newEnd);
				scale(glm::vec3(targetLength / origLength));
            }			
			translate(-getStart() + newStart);
		}

		void debugPrintTValues() {
            std::cout << "tValues: " << std::endl;
            std::cout << funcUtils::vecToString(m_tValues) << std::endl;
		}


		void debugPrintCurvePoints() {
            MONA_ASSERT(m_dimension == 3, "LIC: only available for dim 3 LICs.");
			glmUtils::printColoredStdVector(m_curvePoints);

		}
    };
    
}





#endif