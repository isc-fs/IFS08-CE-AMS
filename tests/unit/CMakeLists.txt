cmake_minimum_required(VERSION 3.15)
project(ams_unit_tests LANGUAGES C CXX)

# Host build. Run via:
#   cmake -B build-tests -S tests/unit
#   cmake --build build-tests
#   ctest --test-dir build-tests --output-on-failure
#
# DO NOT pass the arm-none-eabi toolchain file here -- this is the
# host build that runs tests on a developer machine / in CI.

set(CMAKE_C_STANDARD            11)
set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# Unity test framework, fetched at configure time. Matches the version
# used by isc-fs/IFS08-CE-ECU so behaviour is consistent across our
# embedded projects.
include(FetchContent)
FetchContent_Declare(Unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.1
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Unity)

# Production sources we want to test on host.
# The full path to the firmware tree is two levels up from here.
set(AMS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Tests link against the real BmsService implementation. cmsis_os2 +
# bms_mutexHandle are mocked in tests/unit/mocks/.
add_executable(ams_unit_tests
    ${AMS_ROOT}/Core/Src/app/bms_service.cpp

    mocks/cmsis_os2_stub.cpp

    test_bms_service.cpp
    unity_runner.cpp
)

target_include_directories(ams_unit_tests PRIVATE
    ${AMS_ROOT}/Core/Inc
    ${AMS_ROOT}/Core/Inc/app
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks   # cmsis_os2.h stub wins over real
    ${Unity_SOURCE_DIR}/src
)

target_link_libraries(ams_unit_tests PRIVATE unity)

# Same flags as the firmware target so behavior matches.
target_compile_options(ams_unit_tests PRIVATE
    -Wall -Wextra -Wpedantic
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)

enable_testing()
add_test(
    NAME ams_unit_tests
    COMMAND ams_unit_tests
)
