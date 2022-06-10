#pragma once
#ifndef GRADIENTDESCENT_HPP
#define GRADIENTDESCENT_HPP

#include <vector>
#include <functional>
#include <Eigen/Dense>

namespace Mona {

	typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorX;
	typedef Eigen::Matrix<float, 3, 1> Vector3;

	template <typename dataT>
	class FunctionTerm { // representa un termino de la funcion objetivo
		template <typename dataT>
		friend class GradientDescent;
		dataT* m_dataPtr;
		// funcion que calcula el valor del termino dado un vector con los valores de las variables
		std::function<float(const VectorX&, dataT*)> m_termFunction;
		// funcion que calcula el valor de la derivada del termino,
		// dado un vector con los valores de las variables y el indice de la variable
		// respecto a la cual se deriva
		std::function<float(const VectorX&, int, dataT*)> m_termPartialDerivativeFunction;
		float calcTerm(const VectorX& args);
		float calcTermPartialDerivative(const VectorX& args, int varIndex);
	public:
		FunctionTerm(std::function<float(const VectorX&, dataT*)> termFunction,
			std::function<float(const VectorX&, int, dataT*)> termPartialDerivativeFunction);
	};
	
	template <typename dataT>
	class GradientDescent {
		std::vector<FunctionTerm<dataT>> m_terms;
		int m_argNum;
		dataT* m_dataPtr;
		std::function<void(VectorX&, dataT*)>  m_postDescentStepCustomBehaviour;
	public:
		GradientDescent(std::vector<FunctionTerm<dataT>> terms, int argNum, dataT* dataPtr, std::function<void(VectorX&, dataT*)>  postDescentStepCustomBehaviour);
		GradientDescent() = default;
		VectorX computeGradient(const VectorX& args);
		VectorX computeArgsMin(float descentRate, int maxIterations, const VectorX& initialArgs);
		float computeFunctionValue(const VectorX& args);
		void setArgNum(int argNum);
	};	

	
};




#endif