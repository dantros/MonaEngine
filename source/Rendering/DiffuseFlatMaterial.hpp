#pragma once
#ifndef DIFFUSEFLATMATERIAL_HPP
#define DIFFUSEFLATMATERIAL_HPP
#include "Material.hpp"
#include <glm/glm.hpp>
namespace Mona {
	class DiffuseFlatMaterial : public Material {
	public:
 
		DiffuseFlatMaterial(const ShaderProgram& shaderProgram, bool isForSkinning) : Material(shaderProgram, isForSkinning), m_diffuseColor(glm::vec3(1.0f)) {}
		virtual void SetMaterialUniforms(const glm::vec3& cameraPosition) {
			glUniform3fv(ShaderProgram::DiffuseColorShaderLocation, 1, glm::value_ptr(m_diffuseColor));
		}
		const glm::vec3& GetDiffuseColor() const { return m_diffuseColor; }
		void SetDiffuseColor(const glm::vec3& color) { m_diffuseColor = color; }
	private:
		glm::vec3 m_diffuseColor;
	};
}
#endif