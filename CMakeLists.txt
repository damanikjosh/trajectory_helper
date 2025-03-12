cmake_minimum_required(VERSION 3.27)

# Set project name and version
project(trajectory_helper VERSION 1.0.0 LANGUAGES CXX)

# Prevent in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. Create a separate build directory and run CMake from there.")
endif()

# Define installation paths
set(TRAJECTORY_HELPER_INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include/trajectory_helper")
set(TRAJECTORY_HELPER_CMAKE_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake/trajectory_helper")

# Define the interface library (header-only)
add_library(${PROJECT_NAME} INTERFACE)

# Set include directories
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${TRAJECTORY_HELPER_INCLUDE_INSTALL_DIR}>
)

# Install header files
install(DIRECTORY include/ DESTINATION ${TRAJECTORY_HELPER_INCLUDE_INSTALL_DIR})

# Export the library target
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)

# Generate package configuration files
include(CMakePackageConfigHelpers)

# Generate a version file for find_package()
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Configure the trajectory_helperConfig.cmake file
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    @ONLY
)

# Install package configuration files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION "${TRAJECTORY_HELPER_CMAKE_CONFIG_INSTALL_DIR}"
)

# Install the export targets file
install(EXPORT ${PROJECT_NAME}Targets
    FILE "${PROJECT_NAME}Targets.cmake"
    DESTINATION "${TRAJECTORY_HELPER_CMAKE_CONFIG_INSTALL_DIR}"
)

# Register the package in CMake’s package registry
export(EXPORT ${PROJECT_NAME}Targets FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

# Build options
option(BUILD_TESTS "Build test programs" OFF)

# Tests configuration
if(BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    
    # Find all test files in the test directory
    file(GLOB TEST_SOURCES "test/*_test.cpp")
    
    # Create a test executable for each test file
    foreach(TEST_SOURCE ${TEST_SOURCES})
        # Extract the filename without extension
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
        
        # Create the test executable
        add_executable(${TEST_NAME} ${TEST_SOURCE})
        target_link_libraries(${TEST_NAME}
            PRIVATE 
                GTest::GTest 
                GTest::Main
                ${PROJECT_NAME}
        )
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endforeach()
endif()
