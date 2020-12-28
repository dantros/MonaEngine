#include "ShaderProgram.hpp"
#include "Renderer.hpp"
#include "../Core/Log.hpp"
#include <glad/glad.h>
#include <sstream>
#include <fstream>
#include <glm/gtc/type_ptr.hpp>

namespace Mona {



	ShaderProgram::ShaderProgram(const std::filesystem::path& vertexShaderPath, const std::filesystem::path& pixelShaderPath) noexcept
	{
		
		m_programID = 0;
		//Se carga un string con todo el codigo de ambos shaders
		std::string vertexShaderCode = LoadCode(vertexShaderPath);
		std::string pixelShaderCode = LoadCode(pixelShaderPath);

		//Remplazo de constantes
		PreProcessCode(vertexShaderCode);
		PreProcessCode(pixelShaderCode);

		if (vertexShaderCode.length() == 0 || pixelShaderCode.length() == 0)
			return;
		//Intento de Compilar ambos shaders
		unsigned int vertex = CompileShader(vertexShaderCode, vertexShaderPath, GL_VERTEX_SHADER);
		unsigned int pixel = CompileShader(pixelShaderCode, pixelShaderPath, GL_FRAGMENT_SHADER);
		
		if (vertex && pixel)
		{
			//En caso de exito linkeamos el programa
			LinkProgram(vertex, pixel);
		}


	}




	ShaderProgram::ShaderProgram(ShaderProgram&& a) noexcept : m_programID(a.m_programID) {
		a.m_programID = 0;
	}


	ShaderProgram&  ShaderProgram::operator=(ShaderProgram&& a) noexcept
	{
		if (&a == this)
			return *this;
		if (m_programID)
			glDeleteProgram(m_programID);
		m_programID = a.m_programID;
		a.m_programID = 0;
		return *this;
		
	}

	std::string ShaderProgram::LoadCode(const std::filesystem::path& shaderPath) const noexcept
	{
		std::ifstream shaderFile(shaderPath);
		if (!shaderFile.good())
		{
			MONA_LOG_INFO("Shader Error: Couldn't open file at {0}", shaderPath.string());
			return std::string();
		}
		std::stringstream shaderStream;
		shaderStream << shaderFile.rdbuf();
		shaderFile.close();
		return shaderStream.str();
	}

	unsigned int ShaderProgram::CompileShader(const std::string& code, const std::filesystem::path& shaderPath, unsigned int type) const noexcept
	{

		unsigned int shader = glCreateShader(type);
		const char* cCode = code.c_str();
		glShaderSource(shader, 1, &cCode, 0);
		glCompileShader(shader);
		GLint isCompiled = 0;
		glGetShaderiv(shader, GL_COMPILE_STATUS, &isCompiled);
		//Chequeo de errores de compilación
		if (isCompiled == GL_FALSE)
		{
			GLint maxLength = 0;
			glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &maxLength);
			std::vector<GLchar> errorLog(maxLength);
			glGetShaderInfoLog(shader, maxLength, &maxLength, &errorLog[0]);
			glDeleteShader(shader);
			MONA_LOG_ERROR("ShaderProgram Error: {0}", errorLog.data());
			MONA_LOG_ERROR("File Location: {0}", shaderPath.string());
			MONA_ASSERT(false, "");
			return 0;
		}
		return shader;

	}

	void ShaderProgram::LinkProgram(unsigned int vertex, unsigned int pixel) noexcept
	{
		unsigned int program = glCreateProgram();
		glAttachShader(program, vertex);
		glAttachShader(program, pixel);
		glLinkProgram(program);
		GLint isLinked = 0;
		glGetProgramiv(program, GL_LINK_STATUS, (int*)&isLinked);
		//Chequeo de error de linkeo
		if (isLinked == GL_FALSE)
		{
			GLint maxLength = 0;
			glGetProgramiv(program, GL_INFO_LOG_LENGTH, &maxLength);
			std::vector<GLchar> infoLog(maxLength);
			glGetProgramInfoLog(program, maxLength, &maxLength, &infoLog[0]);
			glDeleteProgram(program);
			glDeleteShader(vertex);
			glDeleteShader(pixel);

			MONA_LOG_ERROR("Shader Linking Error: {0}", infoLog.data());
			MONA_ASSERT(false, "");
			return;
		}

		glDetachShader(program, vertex);
		glDetachShader(program, pixel);
		m_programID = program;
	}

	void ShaderProgram::PreProcessCode(std::string& code)
	{
		struct ShaderConstant {
			std::string key;
			std::string value;
		};
		std::array<ShaderConstant,4> constants = {{
			{"${MAX_DIRECTIONAL_LIGHTS}", std::to_string(Renderer::NUM_HALF_MAX_DIRECTIONAL_LIGHTS * 2)},
			{"${MAX_SPOT_LIGHTS}", std::to_string(Renderer::NUM_HALF_MAX_SPOT_LIGHTS * 2)} ,
			{"${MAX_POINT_LIGHTS}", std::to_string(Renderer::NUM_HALF_MAX_POINT_LIGHTS * 2)},
			{"${MAX_BONES}", std::to_string(Renderer::NUM_MAX_BONES)}} };
		
		for (ShaderConstant& c : constants) {
			size_t pos = 0;
			while ((pos = code.find(c.key, pos)) != std::string::npos) 
			{
				code.replace(pos, c.key.length(), c.value);
				pos += c.value.length();
			}
		}

	}

	ShaderProgram::~ShaderProgram()
	{
		if(m_programID)
			glDeleteProgram(m_programID);
	}

}