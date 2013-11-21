cmake_minimum_required(VERSION 2.8)
project(simulator)

find_package(qibuild)

# Create a executable named simulator
# with the source file: main.cpp
qi_create_bin(simulator "main.cpp")
qi_use_lib(simulator ALCOMMON ALERROR)
# Add a simple test:
enable_testing()
qi_create_test(test_simulator "test.cpp")

