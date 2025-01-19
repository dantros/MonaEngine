function(Add_Example TARGETNAME FILENAME)
	add_executable(${TARGETNAME} ${FILENAME})
	set_property(TARGET ${TARGETNAME} PROPERTY CXX_STANDARD 20)
	set_property(TARGET ${TARGETNAME} PROPERTY FOLDER Examples)
	target_link_libraries(${TARGETNAME} PRIVATE MonaEngine)
	target_include_directories(${TARGETNAME} PRIVATE ${MONA_INCLUDE_DIRECTORY} ${THIRD_PARTY_INCLUDE_DIRECTORIES})
	add_custom_command(TARGET ${TARGETNAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different 
        $<TARGET_FILE:OpenAL> $<TARGET_FILE_DIR:${TARGETNAME}>)
	add_custom_command(TARGET ${TARGETNAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different 
        config.json $<TARGET_FILE_DIR:${TARGETNAME}>)
endfunction(Add_Example)

# Temporarily disabled as they do not work
#Add_Example(Example0_MinimalSetup MinimalSetup.cpp)
#Add_Example(Example1_WindowInputOperations WindowInputExample.cpp)
