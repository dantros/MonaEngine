#include "EventManager.hpp"

namespace Mona
{
	
	void EventManager::ShutDown() noexcept
	{
		m_observers.clear();
	}

}

