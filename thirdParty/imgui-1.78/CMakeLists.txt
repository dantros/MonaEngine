
file(GLOB imguifiles
		"imgui*.cpp"
		"*.h")
add_library(ImGui STATIC ${imguifiles})
target_include_directories(ImGui PRIVATE ${THIRD_PARTY_INCLUDE_DIRECTORIES})
