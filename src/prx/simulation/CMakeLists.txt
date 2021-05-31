## Utilities library of DirtMP

file(GLOB_RECURSE SRC_PRX_LIB_SIMULATION  ./*.cpp)

add_library(${PROJECT_NAME}_simulation ${SRC_PRX_LIB_SIMULATION})

target_include_directories(${PROJECT_NAME}_simulation PUBLIC ".")