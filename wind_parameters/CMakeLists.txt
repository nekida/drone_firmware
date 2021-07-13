cmake_minimum_required(VERSION 3.1) 

project(test_wind_parameters) 

message("CMake the running...")

set(SOURCE wind_parameters.c test_wind_parameters.c)
set(INCLUDE wind_parameters.h)

add_executable(test_wind_parameters ${SOURCE} ${INCLUDE})

set(PROJECT_TIMEOUT 1)

#tests enable
enable_testing()

#add test
add_test(test_integer_1         test_wind_parameters 1      2       3       4       i)
add_test(test_integer_2         test_wind_parameters 1213   32      345     90      i)
add_test(test_signed_integer_1  test_wind_parameters -11    -232    -13     -4      i)
add_test(test_signed_integer_2  test_wind_parameters 1213   -32     345     -90     i)
add_test(test_float_1           test_wind_parameters 1210   330     980     120     f)
add_test(test_float_2           test_wind_parameters 56.1   90.9    777     34      f)

#set behavior of tests
set_tests_properties(test_integer_1 test_integer_2 test_float_1 test_float_2
PROPERTIES TIMEOUT ${PROJECT_TIMEOUT})

set_tests_properties(test_integer_1 PROPERTIES  
PASS_REGULAR_EXPRESSION "1 2 3 4"
FAIL_REGULAR_EXPRESSION "Test 1 call with integer value is fail!")

set_tests_properties(test_integer_2 PROPERTIES  
PASS_REGULAR_EXPRESSION "1213 32 345 90"
FAIL_REGULAR_EXPRESSION "Test 2 call with integer value is fail!")

set_tests_properties(test_signed_integer_1 PROPERTIES  
PASS_REGULAR_EXPRESSION "-11 -232 -13 -4"
FAIL_REGULAR_EXPRESSION "Test 1 call with signed integer value is fail!")

set_tests_properties(test_signed_integer_2 PROPERTIES  
PASS_REGULAR_EXPRESSION "1213 -32 345 -90"
FAIL_REGULAR_EXPRESSION "Test 2 call with signed integer value is fail!")

set_tests_properties(test_float_1 PROPERTIES  
PASS_REGULAR_EXPRESSION "1210.0 330.0 980.0 120.0"
FAIL_REGULAR_EXPRESSION "Test 1 call with float value is fail!")

set_tests_properties(test_float_2 PROPERTIES  
PASS_REGULAR_EXPRESSION "56.1 90.9 777.0 34.0"
FAIL_REGULAR_EXPRESSION "Test 2 call with float value is fail!")

message("CMake the stop\n")