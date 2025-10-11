cmake_minimum_required(VERSION 3.16)
project(secret_heights)

set(CMAKE_CXX_STANDARD 20)

# Add include directory
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/local/include/eigen3)

add_subdirectory(src)

add_executable(secret_heights main.cpp)

# Link your library to the executable
target_link_libraries(secret_heights PRIVATE secret_heights_lib)

find_package(Eigen3 CONFIG REQUIRED)
if (TARGET Eigen3::Eigen)
    target_link_libraries(secret_heights_lib PRIVATE Eigen3::Eigen)
else()
    # Fallback for older Eigen versions or if find_package fails
    include_directories(/usr/local/include/eigen3) # Or the path noted from brew install
endif()
