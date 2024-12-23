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

# Setting up the configuration file to use the asset directories from the source code, this is only for development.
set(APPLICATION_ASSETS_DIR ${CMAKE_SOURCE_DIR}/examples/Assets)
set(ENGINE_ASSETS_DIR ${CMAKE_SOURCE_DIR}/EngineAssets)
configure_file(${CMAKE_SOURCE_DIR}/config.json.in config.json)

Add_Example(Example0_MinimalSetup MinimalSetup.cpp)
Add_Example(Example1_WindowInputOperations WindowInputExample.cpp)
Add_Example(Example2_Breakout Breakout.cpp)
Add_Example(Example3_AnimationAudio AnimationAudioExample.cpp)
Add_Example(Example4_IKBasic IKBasicExample.cpp)
Add_Example(Example5_IKMultipleTerrains IKMultipleTerrainsExample.cpp)
Add_Example(Example6_IKMultipleCharacters IKMultipleCharactersExample.cpp)
Add_Example(Example7_ECSExample ECSExample.cpp)
Add_Example(Example8_ECSBreakout ECSBreakout.cpp)
