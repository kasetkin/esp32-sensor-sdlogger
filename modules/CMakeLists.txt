cmake_minimum_required(VERSION 3.22)

project(tinygpsplus)

set(srcs 
  TinyGPSPlus/src/TinyGPS++.cpp
  TinyGPSPlus/src/unicore.cpp
)

# Create a static library target
add_library(${PROJECT_NAME} STATIC ${srcs})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/TinyGPSPlus/src)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 26)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

