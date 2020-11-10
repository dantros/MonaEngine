#pragma once
#ifndef TEXTURE_HPP
#define TEXTURE_HPP
#include <cstdint>
#include <string>
namespace Mona {
	enum class TextureMagnificationFilter {
		Nearest,
		Linear
	};
	enum class TextureMinificationFilter {
		Nearest,
		Linear,
		NearestMimapNearest,
		NearestMipmapLinear,
		LinearMipmapNearest,
		LinearMipmapLinear
	};


	enum class WrapMode {
		Repeat,
		ClampToEdge,
		MirroedRepeat
	};
	class Texture {
	public:
		friend class TextureManager;
		uint32_t GetWidth() const { return m_width; }
		uint32_t GetHeight() const { return m_height; }
		uint32_t GetID() const { return m_ID; }
		void SetSWrapMode(WrapMode wrapMode) noexcept;
		void SetTWrapMode(WrapMode wrapMode) noexcept;
		void SetMagnificationFilter(TextureMagnificationFilter magFilter) noexcept;
		void SetMinificationFilter(TextureMinificationFilter minFilter) noexcept;
		~Texture();
	private:
		Texture(const std::string& stringFilePath,
			TextureMagnificationFilter magFilter = TextureMagnificationFilter::Linear,
			TextureMinificationFilter minFilter = TextureMinificationFilter::LinearMipmapLinear,
			WrapMode sWrapMode = WrapMode::Repeat,
			WrapMode tWrapMode = WrapMode::Repeat,
			bool genMipmaps = false);
		void ClearData() noexcept;
		uint32_t m_ID;
		uint32_t m_width;
		uint32_t m_height;
		uint32_t m_channels;
	};
}
#endif