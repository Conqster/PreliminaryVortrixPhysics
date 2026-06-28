cmake_minimum_required(VERSION 3.18)

#generate project
project(Vortrix VERSION 1.0 LANGUAGES CXX)

option(ENABLE_ALL_WARNINGS "Enable all warning" ON)
# When turning this option on, the library will be compiled with debug symbols
option(GENERATE_DEBUG_SYMBOLS "Generate debug symbols" ON)
# When turning this on, the library will be compiled with C++ exceptions enabled.
# This adds some overhead and Vortrix doesn't use exceptions so by default it is off.
option(CPP_EXCEPTIONS_ENABLED "Enable C++ exceptions" OFF)

option(VX_PROFILE_SIM "Profile simulation informations" ON)
option(VX_PROFILE_BROAD "Profile Broadphase" ON)
option(VX_PROFILE_NARROW "Profile Narrowphase" ON)
option(VX_PROFILE_SOLVER "Profile Constraint Solver" ON)


if(MSVC)
	set(CMAKE_CONFIGURATION_TYPES "Debug;Development;Release;Distribution;ReleaseASan" CACHE STRING "" FORCE)
	

	#/EHsc for std::vector
	#Base flags
	set(VX_GLOBAL_FLAGS "/Zc:__cplusplus /MP /nologo /Zc:inline /fp:fast /fp:except- /Gm- /GR- /EHsc")
	
	set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /Zc:sizedDealloc") 

	
	if(ENABLE_ALL_WARNINGS)
		#  a lot i do not know, about compiler warning 
		#could not use flags /Wall /WX
		#as it break when treating all warnings as Error 
		#
		#in future need to look into suppressing some complier warning 
		#for now keep all warning and as warnings
		#set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /Wall /WX") 
		set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /Wall") 
	endif()
	
	#if(GENERATE_DEBUG_SYMBOLS)
	#	set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /Zi")
	#endif()
	
	#if (NOT CPP_EXCEPTIONS_ENABLED)
		# Remove any existing compiler flag that enables exceptions
	#	string(REPLACE "/EHsc" "" VX_GLOBAL_FLAGS ${CMAKE_CXX_FLAGS})

		# Disable warning about STL and compiler-generated types using noexcept when exceptions are disabled
	#	set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /wd4577")
	#else()
		# Enable exceptions
	#	set(VX_GLOBAL_FLAGS "${VX_GLOBAL_FLAGS} /EHsc")
	#endif()
	
	#common optimisation block
	set(PHYSICS_OPTS "${VX_GLOBAL_FLAGS} /O2 /Oi /Ot /DNDEBUG")
	
	set(CMAKE_CXX_FLAGS_DEBUG "/Zi /MP /nologo /Wall /MP /GS /fp:fast /fp:except- /Gm- /RTC1" CACHE STRING "" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_DEBUG "/DEBUG:FULL " CACHE STRING "" FORCE)
		 
	#Development (Release speed + uProf/VTune profiling)
	set(CMAKE_CXX_FLAGS_DEVELOPMENT "${PHYSICS_OPTS} /Zi /GS" CACHE STRING "" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_DEVELOPMENT "/DEBUG /OPT:REF /OPT:ICF" CACHE STRING "" FORCE)
	
	#Release Speed with no symbol but some profiling benchmarking support
	set(CMAKE_CXX_FLAGS_RELEASE "${PHYSICS_OPTS} /GS- /Gy /GL" CACHE STRING "" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/OPT:REF /OPT:ICF" CACHE STRING "" FORCE)
	
	#Distribution with Link Time Code Gen
	set(CMAKE_CXX_FLAGS_DISTRIBUTION "${PHYSICS_OPTS} /GS- /Gy /GL" CACHE STRING "" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_DISTRIBUTION "/LTCG /OPT:REF /OPT:ICF" CACHE STRING "" FORCE)
	
	#Release Speed with no symbol but some profiling benchmarking support
	set(CMAKE_CXX_FLAGS_RELEASEASAN "${PHYSICS_OPTS} /fsanitize=address /Zi" CACHE STRING "" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_RELEASEASAN  "/DEBUG" CACHE STRING "" FORCE)

endif()



#ISO c++ 17 standard required for features like (std::hash, std::unorderedmap, std::filesystem, etc)
#target_compile_features(crrt_compiler_flags INTERFACE cxx_std_17) 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(VX_DIR ${CMAKE_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules")

option(USE_SSE "Enable SSE Optimisation" ON)
option(USE_AVX2 "Enable AVX2 & FMA Optimisation" ON) #only use for fmadd... at the moment
#later
#option(BUILD_TESTS "build unit tests" ON)
#option(BUILD_PLAYGROUND "build playground" ON)


#later redirect this to via CMakeModules as include(VortrixPhysics)
set(VX_SRC_DIR ${VX_DIR}/Vortrix)
file(GLOB_RECURSE VX_PHY_FILES CONFIGURE_DEPENDS
										${VX_SRC_DIR}/*.h
										${VX_SRC_DIR}/*.inl
										${VX_SRC_DIR}/*.cpp
										${VX_SRC_DIR}/Vortrix.natvis)
						  
#for code organisation 
source_group(TREE ${VX_SRC_DIR} PREFIX "Vortrix" FILES ${VX_PHY_FILES})

#sample framework 
set(VX_SAMPLE_APP ${VX_DIR}/SampleFramework)
file(GLOB_RECURSE VX_APP_FILES CONFIGURE_DEPENDS
										${VX_SAMPLE_APP}/*.h
										${VX_SAMPLE_APP}/*.inl
										${VX_SAMPLE_APP}/*.cpp)
#for code organisation 
source_group(TREE ${VX_SAMPLE_APP} PREFIX "SampleFramework" FILES ${VX_APP_FILES})

#required imgui code
set(VX_EXTERNAL_IMGUI_DIR ${VX_DIR}/external/imgui)
file(GLOB_RECURSE IMGUI_FILES 
					${VX_EXTERNAL_IMGUI_DIR}/*.h 
					${VX_EXTERNAL_IMGUI_DIR}/*.c 
					${VX_EXTERNAL_IMGUI_DIR}/*.cpp)
					
					
#LATER have options for as .exe or .lib
set(VX_EXE ON CACHE_BOOL)
if(VX_EXE)
	message(STATUS "Vortrix will be built as an executable")
else()
	message(STATUS "Vortrix will be built as an static library")
endif()

set(VX_PROGRAM_FILES ${VX_PHY_FILES} ${VX_APP_FILES} ${IMGUI_FILES})


##IMPORTANT THINGS TO INCLUDES
#output directory 
#intermidiate directory

#set(OS_ARCH 64) #word
#set(VX_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/${CMAKE_BUILD_TYPE}/${CMAKE_SYSTEM_NAME}${OS_ARCH}")
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${VX_OUTPUT_DIR}/static libs")
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${VX_OUTPUT_DIR}")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${VX_OUTPUT_DIR}")

#set(CMAKE_INTERMEDIATE_DIR_STRATEGY ${CMAKE_SOURCE_DIR}/build)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
#foreach(OUTPUT_CONFIG ${CMAKE_CONFIGURATION_TYPES})
	#string(TOUPPER ${OUTPUT_CONFIG} CONFIG)
	#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} "${CMAKE_BINARY_DIR}/bin/${OUTPUT_CONFIG}")
	#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} "${CMAKE_BINARY_DIR}/bin/${OUTPUT_CONFIG}")
	#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} "${CMAKE_BINARY_DIR}/lib/${OUTPUT_CONFIG}")
#endforeach()


add_executable(${PROJECT_NAME} ${VX_PROGRAM_FILES})
#work around for in function const 
#target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE
	$<$<CONFIG:Debug>:VX_DEBUG>
	$<$<CONFIG:Development>:VX_DEV>
	$<$<CONFIG:Release>:VX_REL>
	$<$<CONFIG:Distribution>:VX_DIST>
	$<$<CONFIG:ReleaseASan>:VX_REL_ASAN>
	)
	
	
#outputs Debug; Release etc 	
target_compile_definitions(${PROJECT_NAME} PRIVATE 
	VX_BUILD_NAME="$<CONFIG>"
)

#string print outputs DEBUG; DEV, REL etc
target_compile_definitions(${PROJECT_NAME} PRIVATE
	$<$<CONFIG:Debug>:VX_BUILD_STR="DEBUG">
	$<$<CONFIG:Development>:VX_BUILD_STR="DEV">
	$<$<CONFIG:Release>:VX_BUILD_STR="REL">
	$<$<CONFIG:Distribution>:VX_BUILD_STR="DIST">
	$<$<CONFIG:ReleaseASan>:VX_BUILD_STR="REL_ASAN">
)
	
#Traget instruction extension
if(MSVC)
	if(USE_AVX2)
		message(STATUS "[${PROJECT_NAME}] Compiling with AVX2 hardware optimisations")
		target_compile_options(${PROJECT_NAME} PRIVATE "/arch:AVX2")
	endif()
endif()
	
## flags options
# VX_DISABLE_FORCE_INLINE



#additional include directory 
#	project directory	
# 	/external
#	/vortrix
# 	/external/glfw/include
# 	/external/glew/glew-2.1.0/include
# 	/SampleFramework <-- actual app fused in physics lib for now 

target_include_directories(${PROJECT_NAME} PUBLIC
												${PROJECT_SOURCE_DIR}
												${PROJECT_SOURCE_DIR}/external
												${VX_SRC_DIR}
												${PROJECT_SOURCE_DIR}/external/glfw/include
												${PROJECT_SOURCE_DIR}/external/glew/glew-2.1.0/include
												${PROJECT_SOURCE_DIR}/SampleFramework)
												
												

#Linker 
#Additional Library directory
#	glew
# 	glfw
# Additional dependencies
#	opengl32.lib
#	glfw3.lib	
#	glew32.lib

#could use "link_directories" 
target_link_directories(${PROJECT_NAME} PUBLIC
											${PROJECT_SOURCE_DIR}/external/glew/glew-2.1.0/lib/Release/x64
											${PROJECT_SOURCE_DIR}/external/glfw/lib-vc2022)

target_link_libraries(${PROJECT_NAME} PUBLIC 
											opengl32.lib
											glfw3.lib
											glew32.lib)
											
					
target_compile_definitions(${PROJECT_NAME} PRIVATE
					PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
					
if(VX_PROFILE_NARROW)
	target_compile_definitions(${PROJECT_NAME} PUBLIC VX_PROFILE_NARROW)
endif()
if(VX_PROFILE_BROAD)
	target_compile_definitions(${PROJECT_NAME} PUBLIC VX_PROFILE_BROAD)
endif()
if(VX_PROFILE_SIM)
	target_compile_definitions(${PROJECT_NAME} PUBLIC VX_PROFILE_SIM)
endif()

option(VX_USE_TRACY_PROFILER "Enable Tracy Profiler" ON)
if(VX_USE_TRACY_PROFILER)

	set(TRACY_ZIP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/external/TracyProfiler.zip")
	set(TRACY_TARGET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external")
	
	
	if(NOT EXISTS "${TRACY_TARGET_DIR}/TracyProfiler")
		message(STATUS "Extracting ${TRACY_ZIP_SRC}...")
		
		file(ARCHIVE_EXTRACT
			INPUT "${TRACY_ZIP_SRC}"
			DESTINATION "${TRACY_TARGET_DIR}"
			)
			
		# need to include TracyClient.cpp
		target_sources(${PROJECT_NAME} PRIVATE ${TRACY_TARGET_DIR}/TracyProfiler/TracyClient.cpp)
	else()
		message(STATUS "Folder already extracted, skipping.")
	endif()
	
	# unzip trace profiler folder
	target_compile_definitions(${PROJECT_NAME} PUBLIC VX_ENABLE_TRACY)
	target_compile_definitions(${PROJECT_NAME} PUBLIC TRACY_ENABLE)
endif()



set_target_properties(${PROJECT_NAME} PROPERTIES 
						VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
						VS_STARTUP_PROJECT TRUE)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})


#Build Events - Post-Build Event
#Command Line
#	xcopy /y /e /d "$(SolutionDir)assets" "$(OutDir)assets\"
#	xcopy /y /d "$(SolutionDir)external\glew\glew-2.1.0\bin\Release\x64\*.dll" "$(OutDir)"


get_filename_component(MSVC_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
set(ASAN_DLL "${MSVC_DIR}/clang_rt.asan_dynamic-x86_64.dll")

#copy require data to output directory
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	#assets 
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	"${PROJECT_SOURCE_DIR}/assets/"
	"$<TARGET_FILE_DIR:${PROJECT_NAME}>/assets"
	
	
	#dlls
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
	"${PROJECT_SOURCE_DIR}/external/glew/glew-2.1.0/bin/Release/x64/glew32.dll"
	"$<TARGET_FILE_DIR:${PROJECT_NAME}>"
	
	
	
	#copy Asan dll
	COMMAND 
	$<$<CONFIG:ReleaseASan>:${CMAKE_COMMAND}>
	$<$<CONFIG:ReleaseASan>:-E>
	$<$<CONFIG:ReleaseASan>:copy_if_different>
	$<$<CONFIG:ReleaseASan>:${ASAN_DLL}>
	$<$<CONFIG:ReleaseASan>:$<TARGET_FILE_DIR:${PROJECT_NAME}>>
	
	COMMENT "Depolying assets and DLLs to output directory"
	)


