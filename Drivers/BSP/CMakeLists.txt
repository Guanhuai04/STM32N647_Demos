cmake_minimum_required(VERSION 3.22)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/Inc")

macro(add_bsp_package MODULE_NAME SRC_DIR)
    option(BSP_USE_${MODULE_NAME} "Include ${MODULE_NAME} module" OFF)

    if(BSP_USE_${MODULE_NAME})
        file(GLOB_RECURSE MODULE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/*.c")
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MODULE_SRCS})
        message(STATUS "BSP: [ON]  Module ${MODULE_NAME} is loaded.")
    endif()
endmacro()

add_bsp_package(LED Src/LED)
add_bsp_package(NORFLASH Src/NORFlash)
add_bsp_package(KEY Src/Key)
add_bsp_package(UART Src/UART)
add_bsp_package(SYS Src/SYS)