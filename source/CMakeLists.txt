find_package(OpenGL REQUIRED)
set(THIRD_PARTY_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glad/include"
									"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glfw-3.3.2/include"
									"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/spdlog-1.6.0/include"
									"${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/glm-0.9.9.8")
set(THIRD_PARTY_LIBRARIES glad glfw ${OPENGL_LIBRARIES})
add_subdirectory(thirdParty)
add_executable(Sandbox main.cpp)
target_include_directories(Sandbox PRIVATE ${THIRD_PARTY_INCLUDE_DIRECTORIES})
target_link_libraries(Sandbox PRIVATE ${THIRD_PARTY_LIBRARIES})


