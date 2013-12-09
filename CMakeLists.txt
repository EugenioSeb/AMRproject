cmake_minimum_required(VERSION 2.8)
project(simulator)

find_package(qibuild)

add_subdirectory(./remoteApi)
add_definitions(${CMAKE_CXX_FLAGS} "-std=c++0x")

# Create a executable named simulator
# with the source file: main.cpp

qi_create_bin(simulator "main.cpp")
qi_use_lib(simulator ALCOMMON ALERROR)
target_link_libraries(simulator remoteApi pthread)

# qi_create_bin(loadScene "loadScene.cpp")
# target_link_libraries(loadScene remoteApi)
# qi_use_lib(simulator2 ALCOMMON ALERROR)
# qi_create_bin(simulator2 "aplin.cpp") # aproximate linearization controller
# target_link_libraries(simulator2 remoteApi)

# Add a simple test:
enable_testing()
qi_create_test(test_simulator "test.cpp")
