#pragma once
#ifndef GRADIENTDESCENT_HPP
#define GRADIENTDESCENT_HPP

#include <vector>
#include <functional>

namespace Mona {
	template <typename dataT>
	class FunctionTerm { // representa un termino de la funcion objetivo
		int m_argNum;
		dataT* m_dataPtr;
	public:
		FunctionTerm(std::function<float(std::vector<float>, dataT* dataPtr)> termFunction,
			std::function<float(std::vector<float>, int, dataT* dataPtr)> termPartialDerivativeFunction,
			int argNum, dataT* dataPtr);
		// funcion que calcula el valor del termino dado un vector con los valores de las variables
		std::function<float(std::vector<float>, dataT* dataPtr)> m_termFunction;
		// funcion que calcula el valor de la derivada del termino,
		// dado un vector con los valores de las variables y el indice de la variable
		// respecto a la cual se deriva
		std::function<float(std::vector<float>, int, dataT* dataPtr)> m_termPartialDerivativeFunction;
		float calcTerm(std::vector<float> args);
		float calcTermPartialDerivative(std::vector<float> args, int varIndex);
		int getArgNum() const { return m_argNum; }
	};
	
	template <typename dataT>
	class GradientDescent {
		GradientDescent(std::vector<FunctionTerm<dataT>> terms);
		std::vector<FunctionTerm<dataT>> m_terms;
		int m_argNum;
		std::vector<float> computeGradient(std::vector<float> args);
		std::vector<float> computeArgsMin(float descentRate, int maxIterations, std::vector<float> initialArgs);
		float computeFunctionValue(std::vector<float> args);
	};	

	
};




#endif