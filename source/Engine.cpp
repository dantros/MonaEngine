#include "Engine.hpp"
#include "Core/Common.hpp"
#include "Core/Log.hpp"
#include "Core/Config.hpp"

namespace Mona {
	
	void Engine::StartUp(std::unique_ptr<Application> app) noexcept
	{
		Mona::Config& config = Mona::Config::GetInstance();
		config.readFile("config.cfg");
		MONA_ASSERT(app != nullptr, "Engine StartUp Error: Must provide not null Application pointer");
		m_world.StartUp(std::move(app));
		
	}

	void Engine::ShutDown() noexcept
	{
		m_world.ShutDown();


	}

	void Engine::StartMainLoop() noexcept
	{
		m_world.StartMainLoop();
	}

}