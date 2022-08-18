#include "IKNavigationDebugDraw.hpp"
#include <assert.h>
#include "../Core/Log.hpp"
#include <glm/gtc/type_ptr.hpp>
#include <glad/glad.h>
namespace Mona {

	void IKNavigationDebugDraw::StartUp() noexcept {
		dd::initialize(this);
	}
	void IKNavigationDebugDraw::ShutDown() noexcept {
		dd::shutdown();
	}

	// ** Funcionalidad extraida de uno de los archivos de ejemplo de la libreria externa debug-draw **

	static void checkGLError(const char* file, const int line)
	{
		GLenum err;
		while ((err = glGetError()) != GL_NO_ERROR)
		{
			MONA_LOG_ERROR("IKNavigationDebugDraw: GL_ERROR");
		}
	}


	void IKNavigationDebugDraw::drawLineList(const dd::DrawVertex* lines, int count, bool depthEnabled)
	{
		assert(lines != nullptr);
		assert(count > 0 && count <= DEBUG_DRAW_VERTEX_BUFFER_SIZE);

		glBindVertexArray(linePointVAO);
		glUseProgram(linePointProgram);

		glUniformMatrix4fv(linePointProgram_MvpMatrixLocation,
			1, GL_FALSE, glm::value_ptr(mvpMatrix));

		if (depthEnabled)
		{
			glEnable(GL_DEPTH_TEST);
		}
		else
		{
			glDisable(GL_DEPTH_TEST);
		}

		// NOTE: Could also use glBufferData to take advantage of the buffer orphaning trick...
		glBindBuffer(GL_ARRAY_BUFFER, linePointVBO);
		glBufferSubData(GL_ARRAY_BUFFER, 0, count * sizeof(dd::DrawVertex), lines);

		// Issue the draw call:
		glDrawArrays(GL_LINES, 0, count);

		glUseProgram(0);
		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		checkGLError(__FILE__, __LINE__);
	}

	IKNavigationDebugDraw::IKNavigationDebugDraw()
		: mvpMatrix(glm::identity<glm::mat4>())
		, linePointProgram(0)
		, linePointProgram_MvpMatrixLocation(-1)
		, linePointVAO(0)
		, linePointVBO(0)
	{
		// Default OpenGL states:
		glEnable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);

		// This has to be enabled since the point drawing shader will use gl_PointSize.
		glEnable(GL_PROGRAM_POINT_SIZE);

		setupShaderPrograms();
		setupVertexBuffers();
	}

	IKNavigationDebugDraw::~IKNavigationDebugDraw()	{
		glDeleteProgram(linePointProgram);

		glDeleteVertexArrays(1, &linePointVAO);
		glDeleteBuffers(1, &linePointVBO);
	}

	void IKNavigationDebugDraw::setupVertexBuffers() {

		//
	   // Lines/points vertex buffer:
	   //
		{
			glGenVertexArrays(1, &linePointVAO);
			glGenBuffers(1, &linePointVBO);
			checkGLError(__FILE__, __LINE__);

			glBindVertexArray(linePointVAO);
			glBindBuffer(GL_ARRAY_BUFFER, linePointVBO);

			// RenderInterface will never be called with a batch larger than
			// DEBUG_DRAW_VERTEX_BUFFER_SIZE vertexes, so we can allocate the same amount here.
			glBufferData(GL_ARRAY_BUFFER, DEBUG_DRAW_VERTEX_BUFFER_SIZE * sizeof(dd::DrawVertex), nullptr, GL_STREAM_DRAW);
			checkGLError(__FILE__, __LINE__);

			// Set the vertex format expected by 3D points and lines:
			std::size_t offset = 0;

			glEnableVertexAttribArray(0); // in_Position (vec3)
			glVertexAttribPointer(
				/* index     = */ 0,
				/* size      = */ 3,
				/* type      = */ GL_FLOAT,
				/* normalize = */ GL_FALSE,
				/* stride    = */ sizeof(dd::DrawVertex),
				/* offset    = */ reinterpret_cast<void*>(offset));
			offset += sizeof(float) * 3;

			glEnableVertexAttribArray(1); // in_ColorPointSize (vec4)
			glVertexAttribPointer(
				/* index     = */ 1,
				/* size      = */ 4,
				/* type      = */ GL_FLOAT,
				/* normalize = */ GL_FALSE,
				/* stride    = */ sizeof(dd::DrawVertex),
				/* offset    = */ reinterpret_cast<void*>(offset));

			checkGLError(__FILE__, __LINE__);

			// VAOs can be a pain in the neck if left enabled...
			glBindVertexArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		}
	}

	void IKNavigationDebugDraw::setupShaderPrograms()
	{
		//
		// Line/point drawing shader:
		//
		{
			GLuint linePointVS = glCreateShader(GL_VERTEX_SHADER);
			glShaderSource(linePointVS, 1, &linePointVertShaderSrc, nullptr);
			compileShader(linePointVS);

			GLint linePointFS = glCreateShader(GL_FRAGMENT_SHADER);
			glShaderSource(linePointFS, 1, &linePointFragShaderSrc, nullptr);
			compileShader(linePointFS);

			linePointProgram = glCreateProgram();
			glAttachShader(linePointProgram, linePointVS);
			glAttachShader(linePointProgram, linePointFS);

			glBindAttribLocation(linePointProgram, 0, "in_Position");
			glBindAttribLocation(linePointProgram, 1, "in_ColorPointSize");
			linkProgram(linePointProgram);

			linePointProgram_MvpMatrixLocation = glGetUniformLocation(linePointProgram, "u_MvpMatrix");
			if (linePointProgram_MvpMatrixLocation < 0)
			{
				MONA_LOG_ERROR("IKNavigationDebugDraw: Unable to get u_MvpMatrix uniform location!");
			}
			checkGLError(__FILE__, __LINE__);
		}
	}

	const char* IKNavigationDebugDraw::linePointVertShaderSrc = "\n"
		"#version 150\n"
		"\n"
		"in vec3 in_Position;\n"
		"in vec4 in_ColorPointSize;\n"
		"\n"
		"out vec4 v_Color;\n"
		"uniform mat4 u_MvpMatrix;\n"
		"\n"
		"void main()\n"
		"{\n"
		"    gl_Position  = u_MvpMatrix * vec4(in_Position, 1.0);\n"
		"    gl_PointSize = in_ColorPointSize.w;\n"
		"    v_Color      = vec4(in_ColorPointSize.xyz, 1.0);\n"
		"}\n";


	const char* IKNavigationDebugDraw::linePointFragShaderSrc = "\n"
		"#version 150\n"
		"\n"
		"in  vec4 v_Color;\n"
		"out vec4 out_FragColor;\n"
		"\n"
		"void main()\n"
		"{\n"
		"    out_FragColor = v_Color;\n"
		"}\n";

	void IKNavigationDebugDraw::compileShader(const uint32_t shader)
	{
		glCompileShader(shader);
		checkGLError(__FILE__, __LINE__);

		GLint status;
		glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
		checkGLError(__FILE__, __LINE__);

		if (status == GL_FALSE)
		{
			GLchar strInfoLog[1024] = { 0 };
			glGetShaderInfoLog(shader, sizeof(strInfoLog) - 1, nullptr, strInfoLog);
			MONA_LOG_ERROR("IKNavigationDebugDraw: Shader compiler errors : {0}", strInfoLog);
		}
	}

	void IKNavigationDebugDraw::linkProgram(const uint32_t program)
	{
		glLinkProgram(program);
		checkGLError(__FILE__, __LINE__);

		GLint status;
		glGetProgramiv(program, GL_LINK_STATUS, &status);
		checkGLError(__FILE__, __LINE__);

		if (status == GL_FALSE)
		{
			GLchar strInfoLog[1024] = { 0 };
			glGetProgramInfoLog(program, sizeof(strInfoLog) - 1, nullptr, strInfoLog);
			MONA_LOG_ERROR("IKNavigationDebugDraw: Program linker errors : {0}", strInfoLog);
		}
	}

}