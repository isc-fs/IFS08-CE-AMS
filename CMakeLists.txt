cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# C++ rules per docs/ARCHITECTURE.md §11.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME AMS)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages.
# CubeMX-generated sources stay in C; all application code lives in C++
# under Core/{Inc,Src}/app, wired in via target_sources below.
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Application sources (C++ — see docs/ARCHITECTURE.md §12).
# CubeMX-generated sources (main.c, freertos.c, HAL, etc.) are pulled in
# by cmake/stm32cubemx above. Anything here is hand-written app code.
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/Core/Src/app/*.cpp"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc/app
)

# ---------------------------------------------------------------------------
# Firmware identification (#118). Populates the bl_fwinfo_t record in
# firmware_info.cpp from build-system metadata so the BL / pit-tool can
# tell which commit a flashed image came from.
# ---------------------------------------------------------------------------

# Read VERSION file (semver: MAJOR.MINOR.PATCH).
file(READ ${CMAKE_SOURCE_DIR}/VERSION AMS_VERSION_RAW)
string(STRIP "${AMS_VERSION_RAW}" AMS_VERSION_RAW)
if(NOT AMS_VERSION_RAW MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
    message(FATAL_ERROR
        "VERSION file must contain a single MAJOR.MINOR.PATCH line, got '${AMS_VERSION_RAW}'")
endif()
set(AMS_FW_VERSION_MAJOR ${CMAKE_MATCH_1})
set(AMS_FW_VERSION_MINOR ${CMAKE_MATCH_2})
set(AMS_FW_VERSION_PATCH ${CMAKE_MATCH_3})
message(STATUS "AMS firmware version: ${AMS_FW_VERSION_MAJOR}.${AMS_FW_VERSION_MINOR}.${AMS_FW_VERSION_PATCH}")

# Git short hash. Captured at configure time -- builds in a non-git
# checkout (release tarball, etc.) get zeros. The first 8 hex chars
# pack into bl_fwinfo_t::git_hash[0..3].
execute_process(
    COMMAND git rev-parse --short=8 HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE   AMS_GIT_HASH_HEX
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE   AMS_GIT_HASH_RC
)
if(NOT AMS_GIT_HASH_RC EQUAL 0 OR NOT AMS_GIT_HASH_HEX MATCHES "^[0-9a-f]+$")
    set(AMS_GIT_HASH_HEX "00000000")
    message(STATUS "AMS git hash: <none> (not a git checkout)")
else()
    string(LENGTH "${AMS_GIT_HASH_HEX}" _ams_hash_len)
    while(_ams_hash_len LESS 8)
        string(APPEND AMS_GIT_HASH_HEX "0")
        math(EXPR _ams_hash_len "${_ams_hash_len}+1")
    endwhile()
    message(STATUS "AMS git hash: ${AMS_GIT_HASH_HEX}")
endif()
string(SUBSTRING "${AMS_GIT_HASH_HEX}" 0 2 AMS_GIT_HASH_0_HEX)
string(SUBSTRING "${AMS_GIT_HASH_HEX}" 2 2 AMS_GIT_HASH_1_HEX)
string(SUBSTRING "${AMS_GIT_HASH_HEX}" 4 2 AMS_GIT_HASH_2_HEX)
string(SUBSTRING "${AMS_GIT_HASH_HEX}" 6 2 AMS_GIT_HASH_3_HEX)

# Build timestamp. CMake honours SOURCE_DATE_EPOCH for reproducible
# builds; without it, every reconfigure picks up wall-clock now.
string(TIMESTAMP AMS_BUILD_TIMESTAMP "%s" UTC)
message(STATUS "AMS build timestamp: ${AMS_BUILD_TIMESTAMP}")

# ---------------------------------------------------------------------------
# HIL build flags (#205). Promote the legacy CLI-only #defines to
# proper CMake options.  Defaults are OFF -- flight is the safe-by-
# default build; bench users opt in via -DAMS_BMS_HIL_STUB=ON etc.
# ---------------------------------------------------------------------------
option(AMS_BMS_HIL_STUB           "HIL umbrella: bench-only diag counters, 0x4A2 layout swap, current-sensor freshness relax" OFF)
option(AMS_HIL_CLEAR_ERROR_LATCH  "HIL standalone: wipe RTC->BKP_DR1 ErrorLatch on every boot" OFF)
message(STATUS "AMS_BMS_HIL_STUB          = ${AMS_BMS_HIL_STUB}")
message(STATUS "AMS_HIL_CLEAR_ERROR_LATCH = ${AMS_HIL_CLEAR_ERROR_LATCH}")

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    AMS_FW_VERSION_MAJOR=${AMS_FW_VERSION_MAJOR}
    AMS_FW_VERSION_MINOR=${AMS_FW_VERSION_MINOR}
    AMS_FW_VERSION_PATCH=${AMS_FW_VERSION_PATCH}
    AMS_GIT_HASH_0=0x${AMS_GIT_HASH_0_HEX}
    AMS_GIT_HASH_1=0x${AMS_GIT_HASH_1_HEX}
    AMS_GIT_HASH_2=0x${AMS_GIT_HASH_2_HEX}
    AMS_GIT_HASH_3=0x${AMS_GIT_HASH_3_HEX}
    AMS_BUILD_TIMESTAMP=${AMS_BUILD_TIMESTAMP}
    $<$<BOOL:${AMS_BMS_HIL_STUB}>:AMS_BMS_HIL_STUB=1>
    $<$<BOOL:${AMS_HIL_CLEAR_ERROR_LATCH}>:AMS_HIL_CLEAR_ERROR_LATCH=1>
)

# C++ build rules (per docs/ARCHITECTURE.md §11).
# No exceptions, no RTTI, no thread-safe statics: smaller flash, no
# libstdc++ exception unwinder, no hidden mutex around static locals.
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
