#include "Texture.hpp"

#include <stb_image.h>
#include "../Core/Log.hpp"
#include <glad/glad.h>
namespace Mona {

	GLenum WrapEnumToOpenGLEnum(WrapMode wrapMode) {
		switch (wrapMode)
		{
		case Mona::WrapMode::Repeat:
			return GL_REPEAT;
			break;
		case Mona::WrapMode::ClampToEdge:
			return GL_CLAMP_TO_EDGE;
			break;
		case Mona::WrapMode::MirroedRepeat:
			return GL_MIRRORED_REPEAT;
			break;
		default:
			return GL_REPEAT;
			break;
		}
	}
	GLenum MagnificationFilterEnumToOpenGLEnum(TextureMagnificationFilter filter) {
		switch (filter)
		{
		case Mona::TextureMagnificationFilter::Nearest:
			return GL_NEAREST;
			break;
		case Mona::TextureMagnificationFilter::Linear:
			return GL_LINEAR;
			break;
		default:
			return GL_LINEAR;
			break;
		}
	}

	GLenum MinificationFilterEnumToOpenGLEnum(TextureMinificationFilter filter) {
		switch (filter)
		{
		case Mona::TextureMinificationFilter::Nearest:
			return GL_NEAREST;
			break;
		case Mona::TextureMinificationFilter::Linear:
			return GL_LINEAR;
			break;
		case Mona::TextureMinificationFilter::NearestMimapNearest:
			return GL_NEAREST_MIPMAP_NEAREST;
			break;
		case Mona::TextureMinificationFilter::NearestMipmapLinear:
			return GL_NEAREST_MIPMAP_LINEAR;
			break;
		case Mona::TextureMinificationFilter::LinearMipmapNearest:
			return GL_LINEAR_MIPMAP_NEAREST;
			break;
		case Mona::TextureMinificationFilter::LinearMipmapLinear:
			return GL_LINEAR_MIPMAP_LINEAR;
			break;
		default:
			return GL_NEAREST_MIPMAP_LINEAR;
			break;
		}
	}
	
	void Texture::SetSWrapMode(WrapMode wrapMode) noexcept{
		glTextureParameteri(m_ID,GL_TEXTURE_WRAP_S, WrapEnumToOpenGLEnum(wrapMode));
	}

	void Texture::SetTWrapMode(WrapMode wrapMode) noexcept {
		glTextureParameteri(m_ID, GL_TEXTURE_WRAP_T, WrapEnumToOpenGLEnum(wrapMode));
	}

	void Texture::SetMagnificationFilter(TextureMagnificationFilter magFilter) noexcept {
		glTextureParameteri(m_ID, GL_TEXTURE_MAG_FILTER, MagnificationFilterEnumToOpenGLEnum(magFilter));
		
	}

	void Texture::SetMinificationFilter(TextureMinificationFilter minFilter) noexcept {
		glTextureParameteri(m_ID, GL_TEXTURE_MIN_FILTER, MinificationFilterEnumToOpenGLEnum(minFilter));
	}

	Texture::~Texture() {
		if (m_ID)
			ClearData();
	}

	void Texture::ClearData() noexcept {
		MONA_ASSERT(m_ID, "Texture Error: Trying to clear data from already freed texture.");
		glDeleteTextures(1, &m_ID);
		m_ID = 0;
	}

	Texture::Texture(const std::string& stringFilePath,
		TextureMagnificationFilter magFilter,
		TextureMinificationFilter minFilter,
		WrapMode sWrapMode,
		WrapMode tWrapMode,
		bool genMipmaps) :
		m_ID(0),
		m_width(0),
		m_height(0),
		m_channels(0)
	{

		int width, height, channels;
		//Se carga los datos de la imagen usando stb
		stbi_uc* data = stbi_load(stringFilePath.c_str(), &width, &height, &channels, 0);
		if (!data) {
			MONA_LOG_ERROR("Texture Error: Failed to load texture from {0} file.", stringFilePath);
			stbi_image_free(data);
			return;
		}
		GLenum internalFormat = 0;
		GLenum dataFormat = 0;
		if (channels == 1)
		{
			internalFormat = GL_R8;
			dataFormat = GL_RED;
		}
		else if (channels == 4)
		{
			internalFormat = GL_RGBA8;
			dataFormat = GL_RGBA;
		}
		else if (channels == 3)
		{
			internalFormat = GL_RGB8;
			dataFormat = GL_RGB;
		}

		if (!(internalFormat & dataFormat)) {
			MONA_LOG_ERROR("Texture Error: Texture format not supported.", stringFilePath);
			stbi_image_free(data);
			return;
		}

		//Se pasa los datos de CPU a GPU usando OpenGL
		glCreateTextures(GL_TEXTURE_2D, 1, &m_ID);
		glTextureStorage2D(m_ID, 1, internalFormat, width, height);
		glTextureParameteri(m_ID, GL_TEXTURE_WRAP_S, WrapEnumToOpenGLEnum(sWrapMode));
		glTextureParameteri(m_ID, GL_TEXTURE_WRAP_T, WrapEnumToOpenGLEnum(tWrapMode));
		glTextureParameteri(m_ID, GL_TEXTURE_MAG_FILTER, MagnificationFilterEnumToOpenGLEnum(magFilter));
		glTextureParameteri(m_ID, GL_TEXTURE_MIN_FILTER, MinificationFilterEnumToOpenGLEnum(minFilter));
		glTextureSubImage2D(m_ID, 0, 0, 0, width, height, dataFormat, GL_UNSIGNED_BYTE, data);
		if (genMipmaps) {
			glGenerateTextureMipmap(m_ID);
		}
		m_channels = channels;
		m_width = width;
		m_height = height;
		stbi_image_free(data);
	}

}