#pragma once
#ifndef TEXTUREMATERIAL_HPP
#define TEXTUREMATERIAL_HPP
#include <memory>
#include "Texture.hpp"
#include "Material.hpp"
#include <glm/glm.hpp>

namespace Mona {
	class TextureMaterial : public Material {
	public:

		TextureMaterial(uint32_t shaderID) : Material(shaderID), m_diffuseTexture(nullptr), m_colorIntensity(glm::vec3(1.0f)) {
			glUseProgram(m_shaderID);
			glUniform1i(ShaderProgram::DiffuseTextureSamplerShaderLocation, ShaderProgram::DiffuseTextureUnit);
		}
		const glm::vec3& GetColorIntensity() const { return m_colorIntensity; }
		void SetColorIntensity(const glm::vec3& colorIntensity) { m_colorIntensity = colorIntensity; }
		std::shared_ptr<Texture> GetDiffuseTexture() const { return m_diffuseTexture; }
		void SetDiffuseTexture(std::shared_ptr<Texture> diffuseTexture) { m_diffuseTexture = diffuseTexture; }
		virtual void SetMaterialUniforms() {
			glBindTextureUnit(ShaderProgram::DiffuseTextureUnit, m_diffuseTexture->GetID());
			glUniform3fv(ShaderProgram::ColorIntensityShaderLocation, 1, glm::value_ptr(m_colorIntensity));
		}
	private:
		std::shared_ptr<Texture> m_diffuseTexture;
		glm::vec3 m_colorIntensity;
	};
}
#endif