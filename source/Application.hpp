#pragma once
#ifndef APPLICATION_HPP
#define APPLICATION_HPP
#include <memory>
namespace Mona {
	class Window;
	class Input;
	class World;
	class Application {
	public:
		friend class World;
		virtual void UserStartUp(World &world) noexcept = 0;
		virtual void UserShutDown(World& world) noexcept = 0;
		virtual void UserUpdate(World& world, float timestep)  noexcept = 0;
		virtual ~Application() = default;
	private:
		void StartUp(World& world) noexcept;

	};
}
#endif


