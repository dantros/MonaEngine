#pragma once
#ifndef APPLICATION_HPP
#define APPLICATION_HPP

namespace Mona {
	class Application {
	public:
		virtual void StartUp() noexcept = 0;
		virtual void ShutDown() noexcept = 0;
		virtual void Update(float timeStep) noexcept = 0;
		virtual ~Application() = default;

	};
}
#endif


