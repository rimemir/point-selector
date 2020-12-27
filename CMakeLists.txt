cmake_minimum_required(VERSION 3.1.0)
project(point-selector VERSION 0.1.0)

find_package(gtest REQUIRED NAMES googletest-distribution gtest)

add_executable(test test.cpp)
target_link_libraries(test GTest::gtest_main)