cmake_minimum_required(VERSION 3.5)
project(DirtMP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(BUILD_FOR_PYTHON ON CACHE BOOL "Set ON/TRUE for building Python Bindings")
set(BUILD_FOR_BULLET ON CACHE BOOL "Set ON/TRUE for building for PyBullet")
set(BUILD_FOR_TORCH  ON CACHE BOOL "Set ON/TRUE for building using Torch")
set(BUILD_TESTS		 ON CACHE BOOL "Set ON/TRUE for building tests")

# TODO: go back to c++11?
add_definitions(-O3 -std=c++14 -fPIC -g )
if(APPLE)
	set(CMAKE_CXX_STANDARD 14)
else()
	add_definitions(-lstdc++fs)
	link_libraries(stdc++fs)
endif()
find_package(Boost COMPONENTS system filesystem unit_test_framework  REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
# link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})
link_libraries(
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	# ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
	)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories("${PROJECT_SOURCE_DIR}/src")
link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})

if(BUILD_FOR_TORCH)
	if(DEFINED ENV{Torch_DIR})
        find_package(Torch REQUIRED)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
    else()
		message("Please download libtorch C++, place in external folder and setup the enviornmental variable Torch_DIR accordingly.")
        add_definitions(TORCH_NOT_BUILT)
    endif()
else()
    add_definitions(-DTORCH_NOT_BUILT)
endif()


if(BUILD_FOR_BULLET)
	include_directories(src/prx/bullet_sim)
	add_subdirectory(src/prx/bullet_sim)
else()
	message("Not building for bullet")
    add_definitions(-DBULLET_NOT_BUILT)
endif()

include_directories(src/prx/external)
include_directories(src/prx/visualization)
include_directories(src/prx/planning)
include_directories(src/prx/utilities)
include_directories(src/prx/simulation)

# TODO: Make a shared libraries.
add_subdirectory(src/prx/planning)
add_subdirectory(src/prx/external)
add_subdirectory(src/prx/utilities)
add_subdirectory(src/prx/simulation)
add_subdirectory(src/prx/visualization)

if (BUILD_FOR_PYTHON)
	add_subdirectory(src/pyprx)
endif()

link_directories(${PROJECT_SOURCE_DIR}/lib)

	# ${PROJECT_SOURCE_DIR}/tests/*.cpp
file(GLOB_RECURSE EXAMPLE_FILES
	${PROJECT_SOURCE_DIR}/examples/*.cpp)
foreach(f ${EXAMPLE_FILES})
GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
add_executable(${fname} ${f})
target_link_libraries(${fname} 
		${PROJECT_NAME}_external 
		${PROJECT_NAME}_utilities 
		${PROJECT_NAME}_simulation 
		${PROJECT_NAME}_planning 	
		${PROJECT_NAME}_visualization 
		)
	if(BUILD_FOR_BULLET)
		target_link_libraries(${fname} ${PROJECT_NAME}_bullet_sim )
	endif()
endforeach()


file(GLOB_RECURSE EXECUTABLE_FILES
	${PROJECT_SOURCE_DIR}/executables/*.cpp)
	# ${PROJECT_SOURCE_DIR}/src/prx/experiments/*.cpp)

foreach(f ${EXECUTABLE_FILES})
GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
add_executable(${fname} ${f})
target_link_libraries(${fname}
		${PROJECT_NAME}_external 
		${PROJECT_NAME}_utilities 
		${PROJECT_NAME}_simulation 
		${PROJECT_NAME}_planning 	
		${PROJECT_NAME}_visualization 
		)
	if(BUILD_FOR_BULLET)
		target_link_libraries(${fname} ${PROJECT_NAME}_bullet_sim )
	endif()
	if (BUILD_FOR_TORCH)
		target_link_libraries(${fname} ${TORCH_LIBRARIES})
	endif()
endforeach()

if(BUILD_TESTS)
	message("Building tests")
	ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
	enable_testing()
	file(GLOB_RECURSE TESTS_FILES
		${PROJECT_SOURCE_DIR}/tests/simulation/*.cpp
		${PROJECT_SOURCE_DIR}/tests/planning/*.cpp
		${PROJECT_SOURCE_DIR}/tests/utilities/*.cpp
		)

	foreach(f ${TESTS_FILES})
		GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
		add_executable(${fname} ${f})
		target_link_libraries(${fname} ${PROJECT_NAME}_external )
		target_link_libraries(${fname} ${PROJECT_NAME}_utilities )
		target_link_libraries(${fname} ${PROJECT_NAME}_visualization )
		target_link_libraries(${fname} ${PROJECT_NAME}_simulation )
		target_link_libraries(${fname} ${PROJECT_NAME}_planning )
		if(BUILD_FOR_BULLET)
			target_link_libraries(${fname} ${PROJECT_NAME}_bullet_sim )
		endif()
		target_link_libraries(${fname} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
		add_test(NAME ${fname}_test COMMAND ${fname})
	endforeach()
endif()
