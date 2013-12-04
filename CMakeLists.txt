cmake_minimum_required(VERSION 2.8)
project(simulator)

find_package(qibuild)

add_subdirectory(./remoteApi)

# Create a executable named simulator
# with the source file: main.cpp
qi_create_bin(loadScene "loadScene.cpp")
qi_create_bin(simulator "main.cpp")
qi_create_bin(simulator2 "aplin.cpp")
qi_use_lib(simulator ALCOMMON ALERROR)
qi_use_lib(simulator2 ALCOMMON ALERROR)
target_link_libraries(loadScene remoteApi)
target_link_libraries(simulator remoteApi)
target_link_libraries(simulator2 remoteApi)
# Add a simple test:
enable_testing()
qi_create_test(test_simulator "test.cpp")
