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
												
												