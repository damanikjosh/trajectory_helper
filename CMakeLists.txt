cmake_minimum_required(VERSION 3.27)

# Set the project name
project(trajectory_helper VERSION 1.0.0 LANGUAGES CXX)

# Define the interface library
add_library(${PROJECT_NAME} INTERFACE)

# Specify the include directories
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install target
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Config)

# Install header files
install(DIRECTORY include/ DESTINATION include)

# Export the package for find_package()
install(EXPORT ${PROJECT_NAME}Config
    FILE ${PROJECT_NAME}Config.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Enable CMake package registry
export(EXPORT ${PROJECT_NAME}Config
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
)
