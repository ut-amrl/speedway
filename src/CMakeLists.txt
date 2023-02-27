add_subdirectory(shared)
include_directories(shared)
include_directories(config_reader/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(speedway
            race/race.cc)
target_link_libraries(speedway ${libs})