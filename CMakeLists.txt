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

    # IWDG HAL was not in the .ioc when this project was first
    # generated; the .ioc now enables IWDG1 but the auto-generated
    # cmake/stm32cubemx/CMakeLists.txt still lacks the entry until the
    # next CubeMX regen. Wire it in explicitly so the build works in
    # the meantime. Drop this line once a regen picks the file up
    # automatically.
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_iwdg.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc/app
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
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
