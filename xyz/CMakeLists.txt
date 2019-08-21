cmake_minimum_required(VERSION 3.12)

project(xyz)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
	find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})

message(${aris_DIR})


# user program started...
# et(SOURCE_FILES src/main.cpp src/kaanh.cpp src/kaanh.h)
# add_executable(kaanh ${SOURCE_FILES})

add_library(kaanh_lib STATIC IMPORTED)
set_property(TARGET kaanh_lib PROPERTY IMPORTED_LOCATION /home/kaanh/Desktop/xyz/libkaanh_lib.a)
add_executable(xyz src/main.cpp src/xyz.cpp src/xyz.h src/kaanh.h)
target_link_libraries(xyz kaanh_lib)


if(UNIX)
	target_link_libraries(xyz ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
	target_link_libraries(xyz ${aris_LIBRARIES})
endif(WIN32)
