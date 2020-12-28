#pragma once
#ifndef SHADERPROGRAM_H
#define SHADERPROGRAM_H
#include <cstdint>
#include <filesystem>
#include <string>
#include <glm/glm.hpp>
namespace Mona {
	class ShaderProgram {
	public:
		static constexpr int MvpMatrixShaderLocation = 0;
		static constexpr int ModelMatrixShaderLocation = 1;
		static constexpr int ModelInverseTransposeMatrixShaderLocation = 2;
		static constexpr int UnlitColorShaderLocation = 3;
		static constexpr int UnlitColorTextureSamplerShaderLocation = 3;
		static constexpr int UnlitColorTextureUnit = 0;
		static constexpr int DiffuseColorShaderLocation = 3;
		static constexpr int DiffuseTextureSamplerShaderLocation = 3;
		static constexpr int DiffuseTextureUnit = 0;
		static constexpr int AlbedoShaderLocation = 3;
		static constexpr int AlbedoTextureSamplerShaderLocation = 3;
		static constexpr int AlbedoTextureUnit = 0;
		static constexpr int NormalMapSamplerShaderLocation = 5;
		static constexpr int NormalMapTextureUnit = 1;
		static constexpr int MetallicShaderLocation = 6;
		static constexpr int MetallicSamplerShaderLocation = 6;
		static constexpr int MetallicTextureUnit = 2;
		static constexpr int RoughnessShaderLocation = 7;
		static constexpr int RoughnessSamplerShaderLocation = 7;
		static constexpr int RoughnessTextureUnit = 3;
		static constexpr int AmbientOcclusionShaderLocation = 8;
		static constexpr int AmbientOcclusionSamplerShaderLocation = 8;
		static constexpr int AmbientOcclusionTextureUnit = 4;
		static constexpr int MaterialTintShaderLocation = 4;
		static constexpr int LightsUniformBlockBinding = 0;
		static constexpr int CameraPositionShaderLocation = 9;
		static constexpr int BoneTransformShaderLocation = 10;


		ShaderProgram(const std::filesystem::path& vertexShaderPath,
			const std::filesystem::path& pixelShaderPath) noexcept;
		ShaderProgram() : m_programID(0) {}
		ShaderProgram& operator=(ShaderProgram const &program) = delete;
		ShaderProgram(ShaderProgram const& program) = delete;
		ShaderProgram(ShaderProgram&& a) noexcept;
		ShaderProgram& operator=(ShaderProgram&& a) noexcept; 
		uint32_t GetProgramID() const noexcept { return m_programID; }
		~ShaderProgram();

	private:
		std::string LoadCode(const std::filesystem::path& shaderPath) const noexcept;
		unsigned int CompileShader(const std::string& code, const std::filesystem::path& shaderPath, unsigned int type) const noexcept;
		void LinkProgram(unsigned int vertex, unsigned int pixel) noexcept;
		void PreProcessCode(std::string& code);
		uint32_t m_programID;
	};
}
#endif