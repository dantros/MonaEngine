#pragma once
#ifndef GRADIENTDESCENT_HPP
#define GRADIENTDESCENT_HPP

#include <vector>
#include <functional>

namespace Mona {
	template <typename outT, typename argT, typename dataT>
	struct FunctionTerm { // representa un termino de la funcion objetivo
		// funcion que calcula el valor del termino dado un vector con los valores de las variables
		std::function<outT(std::vector<argT>, dataT* dataPtr)> termValue;
		// funcion que calcula el valor de la derivada del termino,
		// dado un vector con los valores de las variables y el indice de la variable
		// respecto a la cual se deriva
		std::function<outT(std::vector<argT>, int, dataT* dataPtr)> termDerivativeValue;
	};
	
	template <typename outT, typename argT, typename dataT>
	class GradientDescent {
		std::vector<argT> m_initialArgs;
		std::vector<FunctionTerm<outT, argT, dataT>> m_terms;
		std::vector<argT> computeArgsMin();
		outT getFunctionValue(std::vector<argT> args);
		void setInitialArgs(std::vector<argT> initialArgs);
	};	

	
};




#endif