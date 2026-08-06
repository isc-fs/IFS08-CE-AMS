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
    ${AMS_ROOT}/Core/Src/app/current_service.cpp
    ${AMS_ROOT}/Core/Src/app/ltc6811.cpp
    ${AMS_ROOT}/Core/Src/app/vehicle_service.cpp

    mocks/cmsis_os2_stub.cpp

    test_acu_tx_encoders.cpp
    test_balance_controller.cpp
    test_bms_service.cpp
    test_bootloader.cpp
    test_can_busoff_recovery.cpp
    test_crc32.cpp
    test_current_service.cpp
    test_datalogging.cpp
    test_log_rotation.cpp
    test_ntc_table.cpp
    test_dsl_parity.cpp
    test_dsl_dbc_consistency.cpp
    test_diag_dispatch.cpp
    test_diag_proto.cpp
    test_isotp.cpp
    test_logfs_server.cpp
    test_ltc6811_decode.cpp
    test_open_wire.cpp
    test_safety_predicates.cpp
    test_sil_scenarios.cpp
    test_soc_estimator.cpp
    test_state_machine.cpp
    test_telemetry_encoders.cpp
    test_pit_diag_emitter.cpp
    test_fw_health.cpp
    test_vehicle_service.cpp
    unity_runner.cpp
)

target_include_directories(ams_unit_tests PRIVATE
    ${AMS_ROOT}/Core/Inc
    ${AMS_ROOT}/Core/Inc/app
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks   # cmsis_os2.h stub wins over real
    ${Unity_SOURCE_DIR}/src
)

target_link_libraries(ams_unit_tests PRIVATE unity)

# Path to the committed DBC so test_dsl_dbc_consistency can assert the
# code-first descriptors match the wire contract (the C++<->DBC guard).
target_compile_definitions(ams_unit_tests PRIVATE
    AMS_DBC_PATH="${AMS_ROOT}/docs/dbc/ams.dbc"
)

# Host tool that emits docs/dbc/ams.dbc straight from the code-first DSL
# descriptors. Exposed as a CMake target (not just an ad-hoc compile) so
# CI -- and the DBCinator bot's composite action (dump-tool-cmake-target,
# IFS08-DBCinator#5) -- can build it with `cmake --build . --target dbc_dump`.
#   ./dbc_dump > docs/dbc/ams.dbc
add_executable(dbc_dump ${AMS_ROOT}/tools/dbc_dump.cpp)
target_include_directories(dbc_dump PRIVATE ${AMS_ROOT}/Core/Inc)
target_compile_features(dbc_dump PRIVATE cxx_std_17)

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
