project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

#set(sources src/main.cpp src/spline.h)
set(sources src/main2.cpp src/spline.h src/modules/map/map.cpp src/modules/map/map.h
        src/utility.h src/utility.cpp src/modules/prediction/classifier.h src/modules/prediction/classifier.cpp
        src/modules/planning/planning.cpp src/modules/planning/planning.h
        src/modules/prediction/prediction.cpp src/modules/prediction/prediction.h
        src/modules/localization/localization.h src/modules/sensor_fusion/sensor_fusion.h
        src/modules/Constants.h src/modules/vehicle.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS ${CURSES_LIBRARY})
