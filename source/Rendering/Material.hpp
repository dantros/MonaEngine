#pragma once
#ifndef MATERIAL_HPP
#define MATERIAL_HPP
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glad/glad.h>
#include "ShaderProgram.hpp"
namespace Mona {
	enum class MaterialType {
		FlatColor,
		Textured,
		MaterialTypeCount
	};
	class Material {
	public:
		Material(uint32_t shaderID) : m_shaderID(shaderID) {}
		virtual ~Material() = default;
		void SetUniforms(const glm::mat4& perspectiveMatrix, const glm::mat4& viewMatrix, const glm::mat4& modelMatrix) {
			glUseProgram(m_shaderID);
			const glm::mat4 mvpMatrix = perspectiveMatrix * viewMatrix * modelMatrix;
			const glm::mat4 modelInverseTransposeMatrix = glm::transpose(glm::inverse(modelMatrix));
			glUniformMatrix4fv(ShaderProgram::MvpMatrixShaderLocation, 1, GL_FALSE, glm::value_ptr(mvpMatrix));
			glUniformMatrix4fv(ShaderProgram::ModelInverseTransposeMatrixShaderLocation, 1, GL_FALSE, glm::value_ptr(modelInverseTransposeMatrix));
			SetMaterialUniforms();
		}
		virtual void SetMaterialUniforms() = 0;
	protected:
		uint32_t m_shaderID;
	};
}
#endif