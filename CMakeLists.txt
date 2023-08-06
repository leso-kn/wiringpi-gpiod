#
#  SPDX-License-Identifier: MIT
#  Copyright © 2023 Lesosoftware https://github.com/leso-kn.
#
#  wiringpi-gpiod - Main CMake build file.
#

cmake_minimum_required(VERSION 3.0)
project(wiringpi-gpiod, VERSION 0.1.0 LANGUAGES C)

set(wiringpi_gpiod_SOURCES
    wiringPi.c)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED libgpiod)

# libwiringpi-gpiod
add_library(wiringpi-gpiod_o OBJECT ${wiringpi_gpiod_SOURCES})

add_library(wiringpi-gpiod SHARED)
add_library(wiringpi-gpiod_static STATIC)

target_link_libraries(wiringpi-gpiod wiringpi-gpiod_o)
target_link_libraries(wiringpi-gpiod_static wiringpi-gpiod_o)

set_target_properties(wiringpi-gpiod PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR})

set_target_properties(wiringpi-gpiod_static PROPERTIES
    OUTPUT_NAME wiringpi-gpiod)

set_property(TARGET wiringpi-gpiod_o PROPERTY POSITION_INDEPENDENT_CODE ON)

add_compile_definitions(WIRINGPIGEN_VERSION_MAJOR CMAKE_PROJECT_VERSION_MAJOR)
add_compile_definitions(WIRINGPIGEN_VERSION_MINOR CMAKE_PROJECT_VERSION_MINOR)
