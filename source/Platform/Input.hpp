#pragma once
#ifndef INPUT_HPP
#define INPUT_HPP
#include <glm/glm.hpp>
#include <memory>
#include "KeyCodes.hpp"

namespace Mona
{
	class Input {
		enum class CursorType{Hidden, Disabled, Normal};
	public:
		Input();
		~Input();
		Input(const Input& input) = delete;
		Input& operator=(const Input& input) = delete;
		//TODO(Byron) Maybe move StartUp/Update to private and make caller friend class
		void Update();
		bool IsKeyPressed(int keycode) const noexcept;
		bool IsMouseButtonPressed(int button) const noexcept;
		glm::dvec2 GetMousePosition() const noexcept;
		void SetCursorType(CursorType type);
	private:
		class Impl;
		std::unique_ptr<Impl> p_Impl;
	};
}

#endif