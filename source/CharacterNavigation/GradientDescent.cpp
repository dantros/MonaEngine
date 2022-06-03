#include "GradientDescent.hpp"
#include "../Core/Log.hpp"


namespace Mona {
	
	template <typename dataT>
	FunctionTerm<dataT>::FunctionTerm(std::function<float(std::vector<float>, dataT* dataPtr)> termFunction,
		std::function<float(std::vector<float>, int, dataT* dataPtr)> termPartialDerivativeFunction,
		int argNum, dataT* dataPtr) :
		m_termFunction(termFunction), m_termPartialDerivativeFunction(termPartialDerivativeFunction),
		m_argNum(argNum), m_dataPtr(dataPtr) {
	}
	template <typename dataT>
	float FunctionTerm<dataT>::calcTerm(std::vector<float> args) {
		if (args.size() != m_argNum) {
			MONA_LOG_ERROR("FunctionTerm: number of args does not match term argNum value");
			return -1;
		}
		return m_termFunction(args, m_dataPtr);
	}
	template <typename dataT>
	float FunctionTerm<dataT>::calcTermPartialDerivative(std::vector<float> args, int varIndex) {
		if (args.size() != m_argNum) {
			MONA_LOG_ERROR("FunctionTerm: number of args does not match term argNum value");
			return -1;
		}
		return m_termPartialDerivativeFunction(args, varIndex, m_dataPtr);
	}

	template <typename dataT>
	GradientDescent<dataT>::GradientDescent(std::vector<FunctionTerm<dataT>> terms) {
		MONA_ASSERT(terms.size() > 0, "Must provide at least one function term");
		m_argNum = terms[0].getArgNum();
		for (int i = 1; i < terms.size(); i++) {
			if (terms[i].getArgNum() != m_argNum) {
				MONA_LOG_ERROR("GradientDescent: All terms must have the same number of arguments. Term {0} had {1}", i, terms[i].getArgNum());
				return;
			}
		}
		m_terms = terms;
	}

	template <typename dataT>
	std::vector<float> GradientDescent<dataT>::computeGradient(std::vector<float> args) {
		std::vector<float> gradient(m_argNum);
		for (int i = 0; i < m_argNum; i++) {
			gradient[i] = 0.0f;
		}
		for (int i = 0; i < m_terms.size(); i++) {
			for (int j = 0; j < m_argNum; j++) {
				gradient[j] += m_terms[i].calcTermPartialDerivative(args, j);
			}
		}
		return gradient;
	}

	template <typename dataT>
	std::vector<float> GradientDescent<dataT>::computeArgsMin(float descentRate, int maxIterations, std::vector<float> initialArgs) {
		std::vector<float> args = initialArgs;
		std::vector<float> gradient;
		while (maxIterations < 0) {
			gradient = computeGradient(args);
			for (int i = 0; i < m_argNum; i++) {
				args[i] -= descentRate * gradient[i];
			}
			maxIterations -= 1;
		}
		return args;
	}

	template <typename dataT>
	float GradientDescent<dataT>::computeFunctionValue(std::vector<float> args) {
		float functionValue = 0;
		for (int i = 0; i < m_terms.size(); i++) {
			functionValue += m_terms[i].calcTerm(args);
		}
		return functionValue;
	}

}