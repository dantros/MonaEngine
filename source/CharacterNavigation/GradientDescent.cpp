#include "GradientDescent.hpp"
#include "../Core/Log.hpp"


namespace Mona {
	
	template <typename dataT>
	FunctionTerm<dataT>::FunctionTerm(std::function<float(const std::vector<float>&, dataT*)> termFunction,
		std::function<float(const std::vector<float>&, int, dataT*)> termPartialDerivativeFunction) :
		m_termFunction(termFunction), m_termPartialDerivativeFunction(termPartialDerivativeFunction) {
	}
	template <typename dataT>
	float FunctionTerm<dataT>::calcTerm(const std::vector<float>& args) {
		return m_termFunction(args, m_dataPtr);
	}
	template <typename dataT>
	float FunctionTerm<dataT>::calcTermPartialDerivative(const std::vector<float>& args, int varIndex) {
		return m_termPartialDerivativeFunction(args, varIndex, m_dataPtr);
	}

	template <typename dataT>
	GradientDescent<dataT>::GradientDescent(std::vector<FunctionTerm<dataT>> terms, int argNum, dataT* dataPtr,
		std::function<void(std::vector<float>&, dataT*)>  postDescentStepCustomBehaviour) {
		MONA_ASSERT(terms.size() > 0, "Must provide at least one function term");
		m_argNum = argNum;
		m_terms = terms;
		m_postDescentStepCustomBehaviour = postDescentStepCustomBehaviour;
		m_dataPtr = dataPtr;
		for (int i = 0; i < m_terms.size(); i++) {
			m_terms[i].m_dataPtr = dataPtr;
		}
	}

	template <typename dataT>
	std::vector<float> GradientDescent<dataT>::computeGradient(const std::vector<float>& args) {
		MONA_ASSERT(args.size() == m_argNum, "GradientDescent: number of args does not match argNum value");
		std::vector<float> gradient(m_argNum);
		for (int i = 0; i < m_argNum; i++) {
			gradient[i] = 0;
		}
		for (int i = 0; i < m_terms.size(); i++) {
			for (int j = 0; j < m_argNum; j++) {
				gradient[j] += m_terms[i].calcTermPartialDerivative(args, j);
			}
		}
		return gradient;
	}

	template <typename dataT>
	std::vector<float> GradientDescent<dataT>::computeArgsMin(float descentRate, int maxIterations, const std::vector<float>& initialArgs) {
		MONA_ASSERT(initialArgs.size() == m_argNum, "GradientDescent: number of args does not match argNum value");
		std::vector<float> args = initialArgs;
		std::vector<float> gradient;
		while (0 < maxIterations) {
			gradient = computeGradient(args);
			for (int i = 0; i < args.size(); i++) {
				args[i] -= descentRate * gradient[i];
			}
			m_postDescentStepCustomBehaviour(args, m_dataPtr);
			maxIterations -= 1;
		}
		return args;
	}

	template <typename dataT>
	float GradientDescent<dataT>::computeFunctionValue(const std::vector<float>& args) {
		MONA_ASSERT(args.size() == m_argNum, "GradientDescent: number of args does not match argNum value");
		float functionValue = 0;
		for (int i = 0; i < m_terms.size(); i++) {
			functionValue += m_terms[i].calcTerm(args);
		}
		return functionValue;
	}

	template <typename dataT>
	void  GradientDescent<dataT>::setArgNum(int argNum) {
		m_argNum = argNum;
	}
}