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
		/*
		* Funcion virtual que el usuario del motor DEBE implementar al heredar de esta clase. Este metodo es llamado durante el proceso de
		* inicio del motor.
		*/
		virtual void UserStartUp(World &world) noexcept = 0;
		
		/*
		* Funcion virtual que el usuario del motor implementar al heredar de esta clase. Este metodo es llamado durante el proceso de cierre
		* del motor.
		*/
		virtual void UserShutDown(World& world) noexcept = 0;
		
		/*
		* Funcion virtual que el usuario del motor implementar al heredar de esta clase. Este metodo es llamado cada iteración del main loop del motor,
		* es aqui donde el usuario deberia correr logica general de su aplicación.
		*/
		virtual void UserUpdate(World& world, float timestep)  noexcept = 0;
		virtual ~Application() = default;
	private:
		void StartUp(World& world) noexcept;

	};
}
#endif


