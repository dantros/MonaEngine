set(CMAKE_LEGACY_CYGWIN_WIN32 OFF)
cmake_minimum_required(VERSION 3.15)
project(MonaEngine C CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(OpenGL REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(THIRD_PARTY_INCLUDE_DIRECTORIES
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glad/include"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glfw-3.3.2/include"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/spdlog/include"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glm-0.9.9.8"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/imgui-1.78"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/bullet3-2.88/src"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/openal-soft/include"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/dr_wav"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/assimp-5.1.6/include"
	"${CMAKE_CURRENT_BINARY_DIR}/thirdParty/assimp-5.1.6/include"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/stb"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/console-color"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/debug-draw"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/whereami2cpp/source"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/json/include"
	"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/entt/single_include"
)

set(MONA_INCLUDE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/source")
if(NOT ${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
	set(THIRD_PARTY_INCLUDE_DIRECTORIES ${THIRD_PARTY_INCLUDE_DIRECTORIES} PARENT_SCOPE)
	set(MONA_INCLUDE_DIRECTORY ${MONA_INCLUDE_DIRECTORY} PARENT_SCOPE)
endif()
list (APPEND BULLET_LIBRARIES BulletDynamics)
list (APPEND BULLET_LIBRARIES BulletCollision)
list (APPEND BULLET_LIBRARIES LinearMath)

if(MSVC)
	set(THIRD_PARTY_LIBRARIES glad glfw ${OPENGL_LIBRARIES} ImGui ${BULLET_LIBRARIES} OpenAL dr_wav assimp stb console-color debug-draw whereami2cpp)
else()
	set(THIRD_PARTY_LIBRARIES glad glfw ${OPENGL_LIBRARIES} ImGui ${BULLET_LIBRARIES} stdc++fs OpenAL dr_wav assimp stb console-color debug-draw whereami2cpp)
endif(MSVC)

add_subdirectory(thirdParty)
add_subdirectory(source)
add_subdirectory(tests)

option(MONA_BUILD_EXAMPLES "Build the examples?" OFF)
if (MONA_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()

include(GNUInstallDirs)
install(TARGETS MonaEngine
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		COMPONENT MonaEngine_Runtime
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
		COMPONENT MonaEngine_Include
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		COMPONENT MonaEngine_Runtime
		NAMELINK_COMPONENT MonaEngine_Development
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		COMPONENT MonaEngine_Development
)

# Install process
install(
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/EngineAssets
	DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

if (${MONA_BUILD_EXAMPLES})
	install(TARGETS
		Example0_MinimalSetup
		Example1_WindowInputOperations
		Example2_Breakout
		Example3_AnimationAudio
		Example4_IKBasic
		Example5_IKMultipleTerrains
		Example6_IKMultipleCharacters
		Example7_ECSExample
		Example8_ECSBreakout

		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
			COMPONENT MonaEngineExamples_Runtime
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
			COMPONENT MonaEngineExamples_Runtime
			NAMELINK_COMPONENT MonaEngineExamples_Development
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
			COMPONENT MonaEngineExamples_Development
	)

	install(
		DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/examples/Assets
		DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
	)
endif()
