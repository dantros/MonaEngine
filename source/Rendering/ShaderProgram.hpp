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
		static constexpr int ModelInverseTransposeMatrixShaderLocation = 1;
		//static constexpr int ViewMatrixShaderLocation = 1;
		//static constexpr int ModelMatrixShaderLocation = 2;
		static constexpr int FlatColorShaderLocation = 3;
		static constexpr int DiffuseTextureUnit = 3;
		static constexpr int DiffuseTextureSamplerShaderLocation = 3;
		static constexpr int ColorIntensityShaderLocation = 4;

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
		uint32_t m_programID;
	};
}
#endif