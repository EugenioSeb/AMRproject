cmake_minimum_required(VERSION 2.8)
project(simulator)

find_package(qibuild)

add_subdirectory(./remoteApi)
add_definitions(${CMAKE_CXX_FLAGS} "-std=c++0x")

# Create a executable named simulator
# with the source file: main.cpp

add_library(obstacle obstacle.h obstacle.cpp)
add_library(robtimer robtimer.h robtimer.cpp)
add_library(utils  utils.h utils.cpp)
set_target_properties(utils PROPERTIES LINKER_LANGUAGE CXX)
add_library(trajectory trajectory.h trajectory.cpp)
qi_create_bin(simulator "main.cpp")
qi_use_lib(simulator ALCOMMON ALERROR)
target_link_libraries(simulator remoteApi pthread obstacle robtimer trajectory utils)




# qi_create_bin(loadScene "loadScene.cpp")
# target_link_libraries(loadScene remoteApi)
# qi_use_lib(simulator2 ALCOMMON ALERROR)
# qi_create_bin(simulator2 "aplin.cpp") # aproximate linearization controller
# target_link_libraries(simulator2 remoteApi)

# Add a simple test:
enable_testing()
qi_create_test(test_simulator "test.cpp")
