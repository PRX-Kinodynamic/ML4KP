## Planning library of DirtMP

file(GLOB_RECURSE SRC_PRX_LIB_PLANNING  ./*.cpp)

add_library(${PROJECT_NAME}_planning  ${SRC_PRX_LIB_PLANNING})

target_include_directories(${PROJECT_NAME}_planning PUBLIC ".")