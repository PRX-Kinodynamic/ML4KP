## External libraries of DirtMP
# include_directories("${PROJECT_SOURCE_DIR}/src/prx/external")

file(GLOB_RECURSE SRC_PRX_LIB_EXTERNAL  ./*.cpp)

add_library(${PROJECT_NAME}_external ${SRC_PRX_LIB_EXTERNAL})

target_include_directories(${PROJECT_NAME}_external PUBLIC ".")

# Silencing all warning for the external (non-prx) code
set_target_properties(${PROJECT_NAME}_external PROPERTIES COMPILE_FLAGS "-w")

if(BUILD_FOR_TORCH AND DEFINED ENV{Torch_DIR})
	target_link_libraries(${PROJECT_NAME}_external ${TORCH_LIBRARIES})
endif()