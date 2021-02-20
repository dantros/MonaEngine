#pragma once
#ifndef MATERIAL_HPP
#define MATERIAL_HPP
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glad/glad.h>
#include "ShaderProgram.hpp"
namespace Mona {
	enum class MaterialType {
		UnlitFlat,
		UnlitTextured,
		DiffuseFlat,
		DiffuseTextured,
		PBRFlat,
		PBRTextured,
		MaterialTypeCount
	};

	class Material {
	public:
		Material(const ShaderProgram& shaderProgram, bool isForSkinning) : m_shaderID(shaderProgram.GetProgramID()), m_isForSkinning(isForSkinning) {}
		virtual ~Material() = default;
		void SetUniforms(const glm::mat4& perspectiveMatrix,
			const glm::mat4& viewMatrix,
			const glm::mat4& modelMatrix,
			const glm::vec3& cameraPosition) {

			//Se Configura la información compartida por todos los materiales (Matrices y posicion camara).
			glUseProgram(m_shaderID);
			const glm::mat4 mvpMatrix = perspectiveMatrix * viewMatrix * modelMatrix;
			const glm::mat4 modelInverseTransposeMatrix = glm::transpose(glm::inverse(modelMatrix));
			glUniformMatrix4fv(ShaderProgram::MvpMatrixShaderLocation, 1, GL_FALSE, glm::value_ptr(mvpMatrix));
			glUniformMatrix4fv(ShaderProgram::ModelMatrixShaderLocation, 1, GL_FALSE, glm::value_ptr(modelMatrix));
			glUniformMatrix4fv(ShaderProgram::ModelInverseTransposeMatrixShaderLocation, 1, GL_FALSE, glm::value_ptr(modelInverseTransposeMatrix));
			//Llamado a función virtual que implemental los materiales.
			SetMaterialUniforms(cameraPosition);
		}
		virtual void SetMaterialUniforms(const glm::vec3& cameraPosition) = 0;
		bool IsForSkinning() const { return m_isForSkinning; }
	protected:
		bool m_isForSkinning;
		uint32_t m_shaderID;
	};
}
#endif