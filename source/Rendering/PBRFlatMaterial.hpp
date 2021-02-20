#pragma once
#ifndef PBRFLATMATERIAL_HPP
#define PBRFLATMATERIAL_HPP
#include <memory>
#include "Material.hpp"
#include <glm/glm.hpp>

namespace Mona {
	class PBRFlatMaterial : public Material {
	public:
		PBRFlatMaterial(const ShaderProgram& shaderProgram, bool isForSkinning) : 
			Material(shaderProgram, isForSkinning),
			m_albedo(glm::vec3(1.0f)),
			m_metallic(0.0f),
			m_roughness(0.5f),
			m_ambientOcclusion(1.0f)
		{}
		
		void SetAlbedo(const glm::vec3& albedo) { m_albedo = albedo; }
		void SetMetallic(float metallic) { m_metallic = metallic; }
		void SetRoughnes(float roughness) { m_roughness = roughness; }
		void SetAmbientOcclusion(float ambientOcclusion) { m_ambientOcclusion = ambientOcclusion; }
		const glm::vec3& GetAlbedo() const { return m_albedo; }
		float GetMetallic() const { return m_metallic; }
		float GetRoughness() const { return m_roughness; }
		float GetAmbientOcclusion() const { return m_ambientOcclusion; }
		virtual void SetMaterialUniforms(const glm::vec3& cameraPosition) {
			glUniform3fv(ShaderProgram::AlbedoShaderLocation, 1, glm::value_ptr(m_albedo));
			glUniform1f(ShaderProgram::MetallicShaderLocation, m_metallic);
			glUniform1f(ShaderProgram::RoughnessShaderLocation, m_roughness);
			glUniform1f(ShaderProgram::AmbientOcclusionShaderLocation, m_ambientOcclusion);
			glUniform3fv(ShaderProgram::CameraPositionShaderLocation, 1, glm::value_ptr(cameraPosition));
		}
	private:
		glm::vec3 m_albedo;
		float m_metallic;
		float m_roughness;
		float m_ambientOcclusion;

	};
}
#endif