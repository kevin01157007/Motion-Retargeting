cmake_minimum_required(VERSION 3.16)

# A few lab machines have no system C++ compiler. If the optional Python
# ziglang package is present, use its bundled C++ compiler as a local fallback.
# Normal GCC, Clang, AppleClang, and MSVC installations take priority.
if(UNIX AND NOT CMAKE_CXX_COMPILER AND "$ENV{CXX}" STREQUAL "")
    find_program(HW2_NATIVE_CXX NAMES c++ g++ clang++)
    if(NOT HW2_NATIVE_CXX)
        find_program(HW2_PYTHON3 NAMES python3)
        if(HW2_PYTHON3)
            execute_process(
                COMMAND "${HW2_PYTHON3}" -m ziglang version
                RESULT_VARIABLE HW2_ZIG_RESULT
                OUTPUT_QUIET ERROR_QUIET)
            if(HW2_ZIG_RESULT EQUAL 0)
                set(CMAKE_CXX_COMPILER "${CMAKE_CURRENT_SOURCE_DIR}/tools/zig-cxx.sh"
                    CACHE FILEPATH "C++ compiler" FORCE)
                message(STATUS "No system C++ compiler found; using Python ziglang")
            endif()
        endif()
    endif()
endif()

project(HW2MotionRetargeting LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(hw2
    src/main.cpp
    src/math.cpp
    src/fk.cpp
    src/ik.cpp
    src/retarget.cpp
    src/bonus.cpp
)
target_include_directories(hw2 PRIVATE src)
if(MSVC)
    target_compile_options(hw2 PRIVATE /W4)
else()
    target_compile_options(hw2 PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Keep the starter viewer current after each native build.
if(NOT CMAKE_CROSSCOMPILING)
    add_custom_target(hw2_viewer ALL
        COMMAND $<TARGET_FILE:hw2>
                "${CMAKE_CURRENT_SOURCE_DIR}/data/source.motion"
                "${CMAKE_CURRENT_SOURCE_DIR}/data/target.rig"
                "${CMAKE_CURRENT_BINARY_DIR}/viewer.html"
        DEPENDS hw2
        COMMENT "Refreshing the HW2 viewer"
        VERBATIM)
endif()

enable_testing()
add_test(NAME load_and_export
    COMMAND hw2 "${CMAKE_CURRENT_SOURCE_DIR}/data/source.motion"
                "${CMAKE_CURRENT_SOURCE_DIR}/data/target.rig"
                "${CMAKE_CURRENT_BINARY_DIR}/smoke.html" --frames 3)
