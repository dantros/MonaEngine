function(Add_Test TARGETNAME FILENAME)
	add_executable(${TARGETNAME} ${FILENAME})
	set_property(TARGET ${TARGETNAME} PROPERTY CXX_STANDARD 20)
	set_property(TARGET ${TARGETNAME} PROPERTY FOLDER Tests)
	target_link_libraries(${TARGETNAME} PRIVATE MonaEngine)
	target_include_directories(${TARGETNAME} PRIVATE ${MONA_INCLUDE_DIRECTORY} ${THIRD_PARTY_INCLUDE_DIRECTORIES})
	add_custom_command(TARGET ${TARGETNAME} POST_BUILD        
		COMMAND ${CMAKE_COMMAND} -E copy_if_different 
        $<TARGET_FILE:OpenAL> $<TARGET_FILE_DIR:${TARGETNAME}>)

endfunction(Add_Test)

Add_Test(test_fbx test_fbx.cpp)
Add_Test(test_bvh test_bvh.cpp)
Add_Test(test_terrain test_terrain.cpp)
Add_Test(test_terrainRender test_terrainRender.cpp)