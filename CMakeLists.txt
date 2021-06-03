cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(kf)

# Set Paths
set(TOOLS_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/tools)
set(KCONFIG_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/_generated)
set(KFLIB_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/kflib)
set(KFAPP_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/kfApp)
set(TEST_CASES_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/testCases)
set(KFH_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/src/kfheader)

# Use simulation  testing mode or Unit testing
option (USE_APP_MODE "Use APP Mode" OFF )
option (USE_UNIT_TESTING "Use Unit testing Mode" ON)
  
#find the pacakges required for eigen and save directory
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})
list(APPEND EXTRA_LIBS Eigen3::Eigen)

# configure a header file to pass some of the CMake settings
# to the source code
make_directory("${KCONFIG_INCLIDE_DIR}/_generated")
configure_file ("${PROJECT_SOURCE_DIR}/KalmanConfig.h.in"
                "${KCONFIG_INCLIDE_DIR}/KalmanConfig.h")

# #set build type to debug and C++ standard to 11
set(CMAKE_CXX_STANDARD 11) 

# add the binary tree to the search path for include files
# so that we will find KalmanConfig.h
include_directories ("${TOOLS_INCLIDE_DIR}")
include_directories ("${PROJECT_BINARY_DIR}")
include_directories ("${KCONFIG_INCLIDE_DIR}")
include_directories ("${KFH_INCLIDE_DIR}")
include_directories ("${KFLIB_INCLIDE_DIR}")
include_directories ("${TOOLS_INCLIDE_DIR}")
include_directories ("${KFAPP_INCLIDE_DIR}")
include_directories ("${TEST_CASES_INCLIDE_DIR}")

if(USE_APP_MODE)
    message("Use APP Mode")
    #Add Tools sub-directory
    add_subdirectory(${TOOLS_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS tools)
    list(APPEND EXTRA_INCLUDES "${TOOLS_INCLIDE_DIR}")

    #Add kfApp sub-directory
    add_subdirectory(${KFAPP_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS kfApp)
    list(APPEND EXTRA_INCLUDES "${KFAPP_INCLIDE_DIR}")

    #Add Kalman Filter library sub-directory
	list(APPEND EXTRA_INCLUDES "${KFLIB_INCLIDE_DIR}")

    #compile the source code and link it with external libraries
    add_executable(kf src/main.cpp)
    target_include_directories(kf PUBLIC ${EXTRA_INCLUDES})
    target_link_libraries (kf PUBLIC ${EXTRA_LIBS} ${PCL_LIBRARIES})
    
elseif(USE_UNIT_TESTING)

    message("Use unit testing Mode")
    enable_testing()

    #Find Gtest lib and include the directories
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    #Add Tools sub-directory
    add_subdirectory(${TOOLS_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS tools)
    list(APPEND EXTRA_INCLUDES "${TOOLS_INCLIDE_DIR}")

    #Add kfApp sub-directory
    add_subdirectory(${KFAPP_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS kfApp)
    list(APPEND EXTRA_INCLUDES "${KFAPP_INCLIDE_DIR}")

    #Add kfApp sub-directory
    add_subdirectory(${TEST_CASES_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS testCases)
    list(APPEND EXTRA_INCLUDES "${TEST_CASES_INCLIDE_DIR}")

    #Add Kalman Filter library sub-directory
	list(APPEND EXTRA_INCLUDES "${KFLIB_INCLIDE_DIR}")

	#compile the source code and link it with external libraries
    add_executable(gtest src/main_test.cpp)
    target_include_directories(gtest PUBLIC ${EXTRA_INCLUDES})
    target_link_libraries (gtest PUBLIC testCases)
    target_link_libraries (gtest PUBLIC tools)
    target_link_libraries (gtest PUBLIC kfApp)
    target_link_libraries(gtest PUBLIC ${GTEST_BOTH_LIBRARIES})
    
    #add the testing
    add_test(NAME gtest COMMAND gtest)
endif()