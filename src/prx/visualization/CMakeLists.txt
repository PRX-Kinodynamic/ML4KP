## Utilities library of DirtMP

file(GLOB_RECURSE SRC_PRX_LIB_VISUALIZATION  ./*.cpp)

add_library(${PROJECT_NAME}_visualization ${SRC_PRX_LIB_VISUALIZATION})

target_include_directories(${PROJECT_NAME}_visualization PUBLIC ".")
target_link_libraries(${PROJECT_NAME}_visualization ${PROJECT_NAME}_simulation )
target_link_libraries(${PROJECT_NAME}_visualization ${PROJECT_NAME}_utilities )
