# Pulls in Zephyr build system
cmake_minimum_required(VERSION 3.13.1)
# include(../zephyr/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
# include(../nrf/cmake/boilerplate.cmake NO_POLICY_SCOPE)
find_package(Zephyr REQUIRED HINTS ..)
project(node)
zephyr_include_directories(include)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/aht20.c)
target_sources(app PRIVATE src/model_handler.c)
