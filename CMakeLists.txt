set(CMAKE_LEGACY_CYGWIN_WIN32 OFF)
cmake_minimum_required(VERSION 3.15)
project(MonaEngine C CXX)
find_package(OpenGL REQUIRED)
set(THIRD_PARTY_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glad/include"
									"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glfw-3.3.2/include"
									"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/spdlog-1.6.0/include"
									"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glm-0.9.9.8")
set(THIRD_PARTY_LIBRARIES glad glfw ${OPENGL_LIBRARIES})
add_subdirectory(thirdParty)
add_subdirectory(source)
