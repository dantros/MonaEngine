#pragma once
#ifndef FLATCOLORMATERIAL_HPP
#define FLATCOLORMATERIAL_HPP
#include "Material.hpp"
#include <glm/glm.hpp>
namespace Mona {
	class FlatColorMaterial : public Material {
	public:
 
		FlatColorMaterial(uint32_t shaderID) : Material(shaderID), m_flatColor(glm::vec3(1.0f)) {}
		virtual void SetMaterialUniforms() {
			glUniform3fv(ShaderProgram::FlatColorShaderLocation, 1, glm::value_ptr(m_flatColor));
		}
		const glm::vec3& GetColor() const { return m_flatColor; }
		void SetColor(const glm::vec3& color) { m_flatColor = color; }
	private:
		glm::vec3 m_flatColor;
	};
}
#endif