#ISO c++ 17 standard required for features like (std::hash, std::unorderedmap, std::filesystem, etc)
#target_compile_features(crrt_compiler_flags INTERFACE cxx_std_17) 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



#files 
set(VX_SRC_DIR ${VX_ROOT_DIR}/Vortrix)
file(GLOB_RECURSE VX_PHY_FILES CONFIGURE_DEPENDS
										${VX_SRC_DIR}/*.h
										${VX_SRC_DIR}/*.inl
										${VX_SRC_DIR}/*.cpp
										${VX_SRC_DIR}/Vortrix.natvis)
										
#for code organisation 
source_group(TREE ${VX_SRC_DIR} PREFIX "Vortrix" FILES ${VX_PHY_FILES})


if(VX_BUILD_SHARED_LIBS)
	add_library(Vortrix SHARED ${VX_PHY_FILES})
else()
	add_library(Vortrix STATIC ${VX_PHY_FILES})
endif()


#target instruction extension
if(USE_AVX2)
	message(STATUS "[Vortrix] Compiling with AVX2 hardware optimisations")
	target_compile_options(Vortrix PRIVATE "/arch:AVX2")
endif()


if(PROFILE_NARROW)
	target_compile_definitions(Vortrix PUBLIC VX_PROFILE_NARROW)
endif()
if(PROFILE_BROAD)
	target_compile_definitions(Vortrix PUBLIC VX_PROFILE_BROAD)
endif()
if(PROFILE_SIM)
	target_compile_definitions(Vortrix PUBLIC VX_PROFILE_SIM)
endif()


#additional include directory 
#	project directory	
# 	/external
#	/vortrix
target_include_directories(Vortrix PUBLIC ${VX_ROOT_DIR})

# for now add trace profile to vortrix as public; making easy for easy ref		
if(USE_TRACY_PROFILER)

	set(TRACY_ZIP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/external/TracyProfiler.zip")
	set(TRACY_TARGET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external")
	
	
	if(NOT EXISTS "${TRACY_TARGET_DIR}/TracyProfiler")
		message(STATUS "Extracting ${TRACY_ZIP_SRC}...")
		
		file(ARCHIVE_EXTRACT
			INPUT "${TRACY_ZIP_SRC}"
			DESTINATION "${TRACY_TARGET_DIR}"
			)
			
	else()
		message(STATUS "Folder already extracted, skipping.")
	endif()
	
	# need to include TracyClient.cpp
	target_sources(Vortrix PRIVATE ${TRACY_TARGET_DIR}/TracyProfiler/TracyClient.cpp)
	source_group(TREE "${TRACY_TARGET_DIR}/TracyProfiler" PREFIX "TracyClient" FILES "${TRACY_TARGET_DIR}/TracyProfiler/TracyClient.cpp")
	
	# unzip trace profiler folder
	target_compile_definitions(Vortrix PUBLIC VX_ENABLE_TRACY)
	target_compile_definitions(Vortrix PUBLIC TRACY_ENABLE)
endif()
												
												