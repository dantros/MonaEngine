#pragma once
#ifndef GRADIENTDESCENT_HPP
#define GRADIENTDESCENT_HPP

#include <vector>
#include <functional>
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"

namespace Mona {

	template <typename dataT>
	class FunctionTerm { // representa un termino de la funcion objetivo
		template <typename dataT>
		friend class GradientDescent;
		dataT* m_dataPtr;
		float m_weight = 1.0f;
		// funcion que calcula el valor del termino dado un vector con los valores de las variables
		std::function<float(const std::vector<float>&, dataT*)> m_termFunction;
		// funcion que calcula el valor de la derivada del termino,
		// dado un vector con los valores de las variables y el indice de la variable
		// respecto a la cual se deriva
		std::function<float(const std::vector<float>&, int, dataT*)> m_termPartialDerivativeFunction;
		float calcTerm(const std::vector<float>& args) {
			return m_termFunction(args, m_dataPtr);
		}
		float calcTermPartialDerivative(const std::vector<float>& args, int varIndex) {
			return m_termPartialDerivativeFunction(args, varIndex, m_dataPtr);
		}
	public:
		FunctionTerm(std::function<float(const std::vector<float>&, dataT*)> termFunction,
			std::function<float(const std::vector<float>&, int, dataT*)> termPartialDerivativeFunction) :
			m_termFunction(termFunction), m_termPartialDerivativeFunction(termPartialDerivativeFunction) {
		}
	};
	
	template <typename dataT>
	class GradientDescent {
		std::vector<FunctionTerm<dataT>> m_terms;
		int m_argNum;
		dataT* m_dataPtr;
		std::function<void(std::vector<float>&, dataT*, std::vector<float>&)>  m_postDescentStepCustomBehaviour;
	public:
		GradientDescent() = default;
		GradientDescent(std::vector<FunctionTerm<dataT>> terms, int argNum, dataT* dataPtr,
			std::function<void(std::vector<float>&, dataT*, std::vector<float>&)>  postDescentStepCustomBehaviour) {
			MONA_ASSERT(terms.size() > 0, "Must provide at least one function term");
			m_argNum = argNum;
			m_terms = terms;
			m_postDescentStepCustomBehaviour = postDescentStepCustomBehaviour;
			m_dataPtr = dataPtr;
			for (int i = 0; i < m_terms.size(); i++) {
				m_terms[i].m_dataPtr = dataPtr;
			}
		};


		std::vector<float> computeGradient(const std::vector<float>& args) {
			MONA_ASSERT(args.size() == m_argNum, "GradientDescent: number of args does not match argNum value");
			std::vector<float> gradient(m_argNum, 0.0f);
			for (int i = 0; i < m_terms.size(); i++) {
				for (int j = 0; j < m_argNum; j++) {
					if (m_terms[i].m_weight != 0) {
						gradient[j] += m_terms[i].m_weight * m_terms[i].calcTermPartialDerivative(args, j);
					}					
				}
			}
			return gradient;
		}
		std::vector<float> computeArgsMin(float descentRate, int maxIterations, float targetArgDelta,
			const std::vector<float>& initialArgs, bool useSGDM=true, bool softenSteps = true) {
			MONA_ASSERT(initialArgs.size() == m_argNum, "GradientDescent: number of args does not match argNum value");
			std::vector<float> args = initialArgs;
			std::vector<float> gradient;
			std::vector<float> argsRawDelta(args.size());
			std::vector<bool> continueDescent(args.size(), true);
			int stepNum = 0;
			while (stepNum < maxIterations && funcUtils::conditionVector_OR(continueDescent)) {
				gradient = computeGradient(args);
				for (int i = 0; i < args.size(); i++) {
					if (stepNum < 0 && softenSteps) {
						if (argsRawDelta[i] * 10 <= gradient[i]) {
							gradient[i] = argsRawDelta[i] * 10;
						}
					}					
					if (!useSGDM || stepNum == 0) {
						argsRawDelta[i] = gradient[i];
					}
					else {
						argsRawDelta[i] = 0.9 * argsRawDelta[i] + 0.1 * gradient[i];
					}
					float argDelta = descentRate * argsRawDelta[i];
					args[i] -= argDelta;
				}
				m_postDescentStepCustomBehaviour(args, m_dataPtr, argsRawDelta);
				for (int i = 0; i < args.size(); i++) {
					continueDescent[i] = targetArgDelta < abs(descentRate*argsRawDelta[i]);
				}
				stepNum += 1;
			}
			return args;
		};


		float computeFunctionValue(const std::vector<float>& args) {
			MONA_ASSERT(args.size() == m_argNum, "GradientDescent: number of args does not match argNum value");
			float functionValue = 0;
			for (int i = 0; i < m_terms.size(); i++) {
				if (m_terms[i].m_weight != 0) {
					functionValue += m_terms[i].m_weight * m_terms[i].calcTerm(args);
				}				
			}
			return functionValue;
		};

		void  setArgNum(int argNum) {
			m_argNum = argNum;
		};

		int getTermNum() {
			return m_terms.size();
		}

		void setTermWeight(int termIndex, float weight){
			m_terms[termIndex].m_weight = weight;
		}
	};	
};




#endif