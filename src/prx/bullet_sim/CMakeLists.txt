## Bullet_sim submodule of DirtMP

find_package(Bullet QUIET)

if (Bullet_FOUND)
	set(BULLET_PHYSICS_PATH "${Bullet_DIR}")
	message("Using system Bullet")
else()
	if (DEFINED ENV{BULLET_PHYSICS_PATH})
		add_definitions(-DBULLET_PHYSICS_PATH="$ENV{BULLET_PHYSICS_PATH}")
		message("Using custom Bullet")
		set(BULLET_INCLUDE_DIR "${BULLET_PHYSICS_PATH}/include/bullet")
	else()
		message("[${CMAKE_PROJECT_NAME}] Bullet NOT FOUND and 'BULLET_PHYSICS_PATH' not set. See README on how to properly setup Bullet.")
		return()
	endif()
endif()

include_directories(${BULLET_INCLUDE_DIR})
message("Building with Bullet Physics")
if(APPLE)
	FIND_LIBRARY(COCOA_LIBRARY Cocoa)
	FIND_PACKAGE(OpenGL)
	include_directories(${OPENGL_INCLUDE_DIR})
	find_package(GLUT)
	include_directories(${GLUT_INCLUDE_DIR})
endif()
# TODO: There must be a better way of including bullet...
# include_directories("${BULLET_PHYSICS_PATH}/include/bullet")
# include("${PROJECT_SOURCE_DIR}/lib/cmake/bullet/BulletConfig.cmake")

file(GLOB_RECURSE SRC_PRX_LIB_BULLET_SIM  ./*.cpp)

add_library(${PROJECT_NAME}_bullet_sim ${SRC_PRX_LIB_BULLET_SIM})

target_compile_definitions(${PROJECT_NAME}_bullet_sim PUBLIC -DBULLET_PHYSICS_PATH="$ENV{BULLET_PHYSICS_PATH}")

target_include_directories(${PROJECT_NAME}_bullet_sim PUBLIC "." 
		${BULLET_INCLUDE_DIR}
		)

target_link_libraries(${PROJECT_NAME}_bullet_sim 
	BulletRoboticsGUI BulletExampleBrowserLib BulletRobotics 
	BulletFileLoader BulletWorldImporter BulletSoftBody 
	BulletDynamics BulletCollision BulletInverseDynamicsUtils 
	BulletInverseDynamics LinearMath OpenGLWindow gwen BussIK
	Bullet3Common
	)

target_link_libraries(${PROJECT_NAME}_bullet_sim pthread ${CMAKE_DL_LIBS})

if(APPLE)
	#TODO: check for other systems
	target_link_libraries(${PROJECT_NAME}_bullet_sim
		# BulletRoboticsGUI BulletExampleBrowserLib BulletRobotics BulletFileLoader
		# BulletWorldImporter BulletInverseDynamicsUtils  
		OpenGLWindow gwen BussIK  
		${OPENGL_LIBRARIES} 
		${GLUT_LIBRARIES}
		)
endif()
	# target_link_libraries(${PROJECT_NAME}     
	# 	pthread ${CMAKE_DL_LIBS})