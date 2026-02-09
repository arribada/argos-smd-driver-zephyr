cmake_minimum_required(VERSION 3.20.0)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(spi_test LANGUAGES C)

target_sources(app PRIVATE src/main.c src/argos_spi_emul.c)
