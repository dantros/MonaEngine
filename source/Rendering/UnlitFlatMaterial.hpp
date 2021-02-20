#pragma once
#ifndef UNLITFLATMATERIAL_HPP
#define UNLITFLATMATERIAL_HPP
#include "Material.hpp"
#include <glm/glm.hpp>
namespace Mona {
	class UnlitFlatMaterial : public Material {
	public:
 
		UnlitFlatMaterial(const ShaderProgram& shaderProgram, bool isForSkinning) : Material(shaderProgram, isForSkinning), m_color(glm::vec3(1.0f)) {}
		virtual void SetMaterialUniforms(const glm::vec3& cameraPosition) {
			glUniform3fv(ShaderProgram::UnlitColorShaderLocation, 1, glm::value_ptr(m_color));
		}
		const glm::vec3& GetColor() const { return m_color; }
		void SetColor(const glm::vec3& color) { m_color = color; }
	private:
		glm::vec3 m_color;
	};
}
#endif