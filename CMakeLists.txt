project(hexapod)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

include_directories(/Users/osipov/dev/hex/eigen3)
include_directories(/usr/include/eigen3)

find_library(OROCOS_KDL orocos-kdl)

add_executable(main src/main.cpp)
target_link_libraries(main ${OROCOS_KDL})