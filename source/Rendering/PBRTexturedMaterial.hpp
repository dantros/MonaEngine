#pragma once
#ifndef PBRTEXTUREDMATERIAL_HPP
#define PBRTEXTUREDMATERIAL_HPP
#include <memory>
#include "Texture.hpp"
#include "Material.hpp"
#include "../Core/Log.hpp"
#include <glm/glm.hpp>

namespace Mona {
	class PBRTexturedMaterial : public Material {
	public:
		PBRTexturedMaterial(const ShaderProgram& shaderProgram, bool isForSkinning) : 
			Material(shaderProgram, isForSkinning),
			m_albedoTexture(nullptr),
			m_normalMapTexture(nullptr),
			m_metallicTexture(nullptr),
			m_roughnessTexture(nullptr),
			m_ambientOcclusionTexture(nullptr),
			m_materialTint(glm::vec3(1.0f)) {
			//Dado que las ubicaiones de las texturas nunca cambian solo se configura al momento de construcción
			glUseProgram(m_shaderID);
			glUniform1i(ShaderProgram::AlbedoTextureSamplerShaderLocation, ShaderProgram::AlbedoTextureUnit);
			glUniform1i(ShaderProgram::NormalMapSamplerShaderLocation, ShaderProgram::NormalMapTextureUnit);
			glUniform1i(ShaderProgram::MetallicSamplerShaderLocation, ShaderProgram::MetallicTextureUnit);
			glUniform1i(ShaderProgram::RoughnessSamplerShaderLocation, ShaderProgram::RoughnessTextureUnit);
			glUniform1i(ShaderProgram::AmbientOcclusionSamplerShaderLocation, ShaderProgram::AmbientOcclusionTextureUnit);
		}
		const glm::vec3& GetMaterialTint() const { return m_materialTint; }
		void SetMaterialTint(const glm::vec3& tint) { m_materialTint = tint; }
		std::shared_ptr<Texture> GetAlbedoTexture() const { return m_albedoTexture; }
		std::shared_ptr<Texture> GetNormalMapTextire() const { return m_normalMapTexture; }
		std::shared_ptr<Texture> GetMetallicTexture() const { return m_metallicTexture; }
		std::shared_ptr<Texture> GetRoughnessTexture() const { return m_roughnessTexture; }
		std::shared_ptr<Texture> GetAmbienOcclusionTexture() const { return m_ambientOcclusionTexture; }
		void SetAlbedoTexture(std::shared_ptr<Texture> albedoTexture) { m_albedoTexture = albedoTexture; }
		void SetNormalMapTexture(std::shared_ptr<Texture> normalMapTexture) { m_normalMapTexture = normalMapTexture; }
		void SetMetallicTexture(std::shared_ptr<Texture> metallicTexture) { m_metallicTexture = metallicTexture; }
		void SetRoughnessTexture(std::shared_ptr<Texture> roughnessTexture) { m_roughnessTexture = roughnessTexture; }
		void SetAmbientOcclusionTexture(std::shared_ptr<Texture> ambientOcclusionTexture) { m_ambientOcclusionTexture = ambientOcclusionTexture; }

		virtual void SetMaterialUniforms(const glm::vec3& cameraPosition) {
			MONA_ASSERT(m_albedoTexture != nullptr, "Material Error: Texture must be not nullptr for rendering to be posible");
			MONA_ASSERT(m_normalMapTexture != nullptr, "Material Error: Texture must be not nullptr for rendering to be posible");
			MONA_ASSERT(m_metallicTexture != nullptr, "Material Error: Texture must be not nullptr for rendering to be posible");
			MONA_ASSERT(m_roughnessTexture != nullptr, "Material Error: Texture must be not nullptr for rendering to be posible");
			MONA_ASSERT(m_ambientOcclusionTexture != nullptr, "Material Error: Texture must be not nullptr for rendering to be posible");
			glBindTextureUnit(ShaderProgram::AlbedoTextureUnit, m_albedoTexture->GetID());
			glBindTextureUnit(ShaderProgram::NormalMapTextureUnit, m_normalMapTexture->GetID());
			glBindTextureUnit(ShaderProgram::MetallicTextureUnit, m_metallicTexture->GetID());
			glBindTextureUnit(ShaderProgram::RoughnessTextureUnit, m_roughnessTexture->GetID());
			glBindTextureUnit(ShaderProgram::AmbientOcclusionTextureUnit, m_ambientOcclusionTexture->GetID());
			glUniform3fv(ShaderProgram::MaterialTintShaderLocation, 1, glm::value_ptr(m_materialTint));
			glUniform3fv(ShaderProgram::CameraPositionShaderLocation, 1, glm::value_ptr(cameraPosition));
		}
	private:
		std::shared_ptr<Texture> m_albedoTexture;
		std::shared_ptr<Texture> m_normalMapTexture;
		std::shared_ptr<Texture> m_metallicTexture;
		std::shared_ptr<Texture> m_roughnessTexture;
		std::shared_ptr<Texture> m_ambientOcclusionTexture;
		glm::vec3 m_materialTint;
	};
}
#endif