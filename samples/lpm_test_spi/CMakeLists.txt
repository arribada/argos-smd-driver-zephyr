cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lpm_test_spi LANGUAGES C)

target_sources(app PRIVATE src/main.c)
