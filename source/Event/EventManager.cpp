#include "EventManager.hpp"
namespace Mona
{
	SubscriptionHandle::~SubscriptionHandle() {
		if (m_eventManager != nullptr && m_eventManager->IsSubcriptionHandleValid(*this)) {
			m_eventManager->Unsubscribe(*this);
		}
	}

	ObserverList::ObserverList() :
		m_firstFreeIndex(s_maxEntries),
		m_lastFreeIndex(s_maxEntries),
		m_freeIndicesCount(0)
	{}
	void EventManager::ShutDown() noexcept
	{
		for (auto& observerList : m_observerLists)
			observerList.ShutDown();
	}


	void ObserverList::Unsubscribe(const SubscriptionHandle& handle) noexcept
	{
		auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "EventManager Error: handle index out of bounds");
		MONA_ASSERT(handle.m_generation == m_handleEntries[index].generation, "EventManager Error: Trying to destroy from invalid handle");
		MONA_ASSERT(m_handleEntries[index].active == true, "EventManager Error: Trying to destroy from inactive handle");
		auto& handleEntry = m_handleEntries[index];
		if (handleEntry.index < m_eventHandlers.size() - 1)
		{
			auto handleEntryIndex = m_handleEntryIndices.back();
			m_eventHandlers[handleEntry.index] = std::move(m_eventHandlers.back());
			m_handleEntries[handleEntryIndex].index = handleEntry.index;
		}

		m_eventHandlers.pop_back();
		m_handleEntryIndices.pop_back();

		if (m_lastFreeIndex != s_maxEntries)
			m_handleEntries[m_lastFreeIndex].index = index;
		else
			m_firstFreeIndex = index;

		handleEntry.active = false;
		handleEntry.prevIndex = m_lastFreeIndex;
		handleEntry.index = s_maxEntries;
		m_lastFreeIndex = index;
		++m_freeIndicesCount;
	}

	void ObserverList::Publish(const Event& e) noexcept
	{
		for (const auto& eventHandler : m_eventHandlers)
		{
			eventHandler(e);
		}
	}

	void ObserverList::ShutDown() noexcept
	{
		m_eventHandlers.clear();
		m_handleEntryIndices.clear();
		m_handleEntries.clear();
		m_firstFreeIndex = s_maxEntries;
		m_lastFreeIndex = s_maxEntries;
		m_freeIndicesCount = 0;
	}

}

