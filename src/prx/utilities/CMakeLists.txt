## Utilities library of DirtMP
find_package(YAMLCPP NAMES Yaml-cpp YamlCpp)
include_directories(${YAMLCPP_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS})

file(GLOB_RECURSE SRC_PRX_LIB_UTILITIES  ./*.cpp)

add_library(${PROJECT_NAME}_utilities ${SRC_PRX_LIB_UTILITIES})

target_include_directories(${PROJECT_NAME}_utilities PUBLIC ".")

set_target_properties(${PROJECT_NAME}_utilities PROPERTIES COMPILE_FLAGS "-Wno-deprecated")

target_link_libraries(${PROJECT_NAME}_utilities ${PROJECT_NAME}_external )
target_link_libraries(${PROJECT_NAME}_utilities ${YAMLCPP_LIBRARY} ${YAML_CPP_LIBRARIES} )
