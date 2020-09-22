#pragma once
#ifndef SHADERPROGRAM_H
#define SHADERPROGRAM_H
#include <cstdint>
#include <filesystem>
#include <string>
namespace Mona {
	class ShaderProgram {
	public:
		ShaderProgram(const std::filesystem::path& vertexShaderPath,
			const std::filesystem::path& pixelShaderPath) noexcept;
		ShaderProgram() : m_programID(0) {}
		ShaderProgram& operator=(ShaderProgram const &program) = delete;
		ShaderProgram(ShaderProgram const& program) = delete;
		ShaderProgram(ShaderProgram&& a) noexcept;
		ShaderProgram& operator=(ShaderProgram&& a) noexcept; 

		void UseProgram() const noexcept;
		~ShaderProgram();

	private:
		std::string LoadCode(const std::filesystem::path& shaderPath) const noexcept;
		unsigned int CompileShader(const std::string& code, const std::filesystem::path& shaderPath, unsigned int type) const noexcept;
		void LinkProgram(unsigned int vertex, unsigned int pixel) noexcept;
		uint32_t m_programID;
	};
}
#endif