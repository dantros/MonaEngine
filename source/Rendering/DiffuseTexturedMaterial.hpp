#pragma once
#ifndef DIFFUSETEXTUREDMATERIAL_HPP
#define DIFFUSETEXTUREDMATERIAL_HPP
#include <memory>
#include "../Core/Log.hpp"
#include "Texture.hpp"
#include "Material.hpp"
#include <glm/glm.hpp>

namespace Mona {
	class DiffuseTexturedMaterial : public Material {
	public:

		DiffuseTexturedMaterial(const ShaderProgram& shaderProgram, bool isForSkinning) : Material(shaderProgram, isForSkinning), m_diffuseTexture(nullptr), m_materialTint(glm::vec3(1.0f)) {
			//Dado que las ubicaiones de las texturas nunca cambian solo se configura al momento de construcción
			glUseProgram(m_shaderID);
			glUniform1i(ShaderProgram::DiffuseTextureSamplerShaderLocation, ShaderProgram::DiffuseTextureUnit);
		}
		const glm::vec3& GetMaterialTint() const { return m_materialTint; }
		void SetMaterialTint(const glm::vec3& tint) { m_materialTint = tint; }
		std::shared_ptr<Texture> GetDiffuseTexture() const { return m_diffuseTexture; }
		void SetDiffuseTexture(std::shared_ptr<Texture> diffuseTexture) { m_diffuseTexture = diffuseTexture; }
		virtual void SetMaterialUniforms(const glm::vec3& cameraPosition) {
			MONA_ASSERT(m_diffuseTexture != nullptr, "Material Error: Texture must be not nullptr for rendering to be posible");
			glBindTextureUnit(ShaderProgram::DiffuseTextureUnit, m_diffuseTexture->GetID());
			glUniform3fv(ShaderProgram::MaterialTintShaderLocation, 1, glm::value_ptr(m_materialTint));
		}
	private:
		std::shared_ptr<Texture> m_diffuseTexture;
		glm::vec3 m_materialTint;
	};
}
#endif