set(MONA_HEADERS 
				Core/Common.hpp 
				Core/Log.hpp
				Core/Config.hpp
				Core/RootDirectory.hpp
				Platform/Window.hpp
				Platform/Input.hpp
				Platform/KeyCodes.hpp
				Event/EventManager.hpp
				Event/Events.hpp
				Engine.hpp
				Application.hpp
				World/GameObject.hpp
				World/GameObjectManager.hpp
				World/Detail/GameObjectManager.hpp
				World/Component.hpp
				World/ComponentHandle.hpp
				World/ComponentManager.hpp
				World/Detail/ComponentManager.hpp
				World/World.hpp)
set(MONA_SOURCES 
				Core/Log.cpp
				Core/Config.cpp
				Event/EventManager.cpp
				Platform/Window.cpp
				Platform/Input.cpp
				Engine.cpp
				Application.cpp
				World/GameObjectManager.cpp)

add_library(MonaEngine STATIC ${MONA_SOURCES} ${MONA_HEADERS} MonaEngine.hpp)
target_include_directories(MonaEngine PRIVATE ${THIRD_PARTY_INCLUDE_DIRECTORIES} MONA_INCLUDE_DIRECTORY)
target_link_libraries(MonaEngine PRIVATE ${THIRD_PARTY_LIBRARIES})
set_property(TARGET MonaEngine PROPERTY CXX_STANDARD 20)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${MONA_SOURCES} ${MONA_HEADERS})

