#pragma once
#ifndef INPUT_HPP
#define INPUT_HPP
#include <glm/glm.hpp>
#include <memory>
#include "KeyCodes.hpp"

namespace Mona
{
	class Engine;
	class EventManager;
	class Input {
		enum class CursorType{Hidden, Disabled, Normal};
	public:
		friend class Engine;
		Input(const Input& input) = delete;
		Input& operator=(const Input& input) = delete;
		//TODO(Byron) Maybe move StartUp/Update to private and make caller friend class
		bool IsKeyPressed(int keycode) const noexcept;
		bool IsMouseButtonPressed(int button) const noexcept;
		glm::dvec2 GetMousePosition() const noexcept;
		glm::dvec2 GetMouseWheelOffset() const noexcept;
		void SetCursorType(CursorType type) noexcept;
	private:
		Input();
		~Input();
		void Update() noexcept;
		void StartUp(EventManager* eventManager) noexcept;
		class Impl;
		std::unique_ptr<Impl> p_Impl;
	};
}

#endif