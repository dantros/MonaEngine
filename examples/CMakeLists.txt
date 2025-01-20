
function(Add_Example TARGETNAME FILENAME)

	add_executable(${TARGETNAME} ${FILENAME})
	set_property(TARGET ${TARGETNAME} PROPERTY CXX_STANDARD 20)
	set_property(TARGET ${TARGETNAME} PROPERTY FOLDER Examples)
	target_link_libraries(${TARGETNAME} PRIVATE MonaEngine)
	target_include_directories(${TARGETNAME} PRIVATE ${MONA_INCLUDE_DIRECTORY} ${THIRD_PARTY_INCLUDE_DIRECTORIES})

	# Making OpenAL available when executing the program from Visual Studio
	add_custom_command(TARGET ${TARGETNAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different 
        $<TARGET_FILE:OpenAL> $<TARGET_FILE_DIR:${TARGETNAME}>)

	# Setting up the configuration file to use the asset directories from the source code, this is only for development.
	set(APPLICATION_ASSETS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Assets)
	set(ENGINE_ASSETS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../EngineAssets)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../config.json.in config.json)
	add_custom_command(TARGET ${TARGETNAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different 
        config.json $<TARGET_FILE_DIR:${TARGETNAME}>)

endfunction(Add_Example)

Add_Example(HelloMonaEngine HelloMonaEngine.cpp)
