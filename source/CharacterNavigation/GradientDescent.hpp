#pragma once
#ifndef GRADIENTDESCENT_HPP
#define GRADIENTDESCENT_HPP

#include <vector>
#include <functional>

namespace Mona {

	template <typename dataT>
	class FunctionTerm { // representa un termino de la funcion objetivo
		template <typename dataT>
		friend class GradientDescent;
		dataT* m_dataPtr;
		// funcion que calcula el valor del termino dado un vector con los valores de las variables
		std::function<float(const std::vector<float>&, dataT*)> m_termFunction;
		// funcion que calcula el valor de la derivada del termino,
		// dado un vector con los valores de las variables y el indice de la variable
		// respecto a la cual se deriva
		std::function<float(const std::vector<float>&, int, dataT*)> m_termPartialDerivativeFunction;
		float calcTerm(const std::vector<float>& args);
		float calcTermPartialDerivative(const std::vector<float>& args, int varIndex);
	public:
		FunctionTerm(std::function<float(const std::vector<float>&, dataT*)> termFunction,
			std::function<float(const std::vector<float>&, int, dataT*)> termPartialDerivativeFunction);
	};
	
	template <typename dataT>
	class GradientDescent {
		std::vector<FunctionTerm<dataT>> m_terms;
		int m_argNum;
		dataT* m_dataPtr;
		std::function<void(std::vector<float>&, dataT*)>  m_postDescentStepCustomBehaviour;
	public:
		GradientDescent(std::vector<FunctionTerm<dataT>> terms, int argNum, dataT* dataPtr, std::function<void(std::vector<float>&, dataT*)>  postDescentStepCustomBehaviour);
		GradientDescent() = default;
		std::vector<float> computeGradient(const std::vector<float>& args);
		std::vector<float> computeArgsMin(float descentRate, int maxIterations, const std::vector<float>& initialArgs);
		float computeFunctionValue(const std::vector<float>& args);
		void setArgNum(int argNum);
	};	

	
};




#endif