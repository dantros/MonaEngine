set(CMAKE_LEGACY_CYGWIN_WIN32 OFF)
cmake_minimum_required(VERSION 3.15)
project(MonaEngine C CXX)
find_package(OpenGL REQUIRED)
set(THIRD_PARTY_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glad/include"
									"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glfw-3.3.2/include"
									"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/spdlog-1.6.0/include"
									"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glm-0.9.9.8"
									"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/imgui-1.78"
									"${BULLET_PHYSICS_SOURCE_DIR}/src")
set(MONA_INCLUDE_DIRECTORY "${CMAKE_SOURCE_DIR}/source")
list (APPEND BULLET_LIBRARIES LinearMath)
list (APPEND BULLET_LIBRARIES Bullet3Common)
list (APPEND BULLET_LIBRARIES BulletInverseDynamics)
list (APPEND BULLET_LIBRARIES BulletCollision)
list (APPEND BULLET_LIBRARIES BulletDynamics)
list (APPEND BULLET_LIBRARIES BulletSoftBody)
set(THIRD_PARTY_LIBRARIES glad glfw ${OPENGL_LIBRARIES} ImGui ${BULLET_LIBRARIES})
configure_file(CMakeConfigFiles/RootDirectory.hpp.in "${CMAKE_SOURCE_DIR}/source/Core/RootDirectory.hpp")
add_subdirectory(thirdParty)
add_subdirectory(source)
add_subdirectory(tests)
add_subdirectory(examples)
