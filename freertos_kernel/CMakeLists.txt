cmake_minimum_required(VERSION 3.24)

# FreeRTOS Kernel Library
set(FREERTOS_KERNEL freertos_kernel)

# FreeRTOS source files
set(FREERTOS_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/tasks.c
        ${CMAKE_CURRENT_SOURCE_DIR}/list.c
        ${CMAKE_CURRENT_SOURCE_DIR}/queue.c
        ${CMAKE_CURRENT_SOURCE_DIR}/timers.c
        ${CMAKE_CURRENT_SOURCE_DIR}/event_groups.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stream_buffer.c
)

# ARM Cortex-A8 specific port files
set(FREERTOS_PORT_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/portable/port.c
)

# Check if portmacro.h exists (it should be in the portable directory)
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/portable/portmacro.h)
    message(FATAL_ERROR "portmacro.h not found in portable directory!")
endif()

# ASM files - separate handling needed for ARM assembly
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/portable/startup.s)
    set_source_files_properties(
            ${CMAKE_CURRENT_SOURCE_DIR}/portable/startup.s
            PROPERTIES LANGUAGE ASM
    )
    list(APPEND FREERTOS_PORT_SOURCES
            ${CMAKE_CURRENT_SOURCE_DIR}/portable/startup.s
    )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/portable/exceptionhandlers.s)
    set_source_files_properties(
            ${CMAKE_CURRENT_SOURCE_DIR}/portable/exceptionhandlers.s
            PROPERTIES LANGUAGE ASM
    )
    list(APPEND FREERTOS_PORT_SOURCES
            ${CMAKE_CURRENT_SOURCE_DIR}/portable/exceptionhandlers.s
    )
endif()

# Create static library for FreeRTOS kernel
add_library(${FREERTOS_KERNEL} STATIC
        ${FREERTOS_SOURCES}
        ${FREERTOS_PORT_SOURCES}
)

# Include directories
target_include_directories(${FREERTOS_KERNEL} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/portable)


# Compile definitions for ARM Cortex-A8
target_compile_definitions(${FREERTOS_KERNEL} PRIVATE
        __ARM_ARCH_7A__
        ARM_ARCH_7A
        __CORTEX_A8__
        CORTEX_A8
)

# Compiler flags for bare-metal ARM
target_compile_options(${FREERTOS_KERNEL} PRIVATE
        -mcpu=cortex-a8
        -marm
        -mthumb-interwork
        -mlittle-endian
        -msoft-float
        -mno-unaligned-access
        -ffunction-sections
        -fdata-sections
        -fno-strict-aliasing
        -fno-builtin
        -Wall
        -Wextra
)

# Note: FreeRTOS doesn't use exceptions or RTTI
target_compile_options(${FREERTOS_KERNEL} PRIVATE
        -fno-exceptions
        -fno-rtti
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)

# Assembly flags
target_compile_options(${FREERTOS_KERNEL} PRIVATE
        $<$<COMPILE_LANGUAGE:ASM>:-mcpu=cortex-a8>
        $<$<COMPILE_LANGUAGE:ASM>:-marm>
        $<$<COMPILE_LANGUAGE:ASM>:-mthumb-interwork>
        $<$<COMPILE_LANGUAGE:ASM>:-x>
        $<$<COMPILE_LANGUAGE:ASM>:-assembler-with-cpp>
)

# Linker flags (passed through the library)
target_link_options(${FREERTOS_KERNEL} INTERFACE
        -Wl,--gc-sections
        -Wl,-wrap,malloc
        -Wl,-wrap,free
        -Wl,-wrap,calloc
        -Wl,-wrap,realloc
)

# Set POSITION_INDEPENDENT_CODE property for ARM
set_target_properties(${FREERTOS_KERNEL} PROPERTIES
        POSITION_INDEPENDENT_CODE OFF
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS OFF
)

# Optional: Add custom target for size analysis
add_custom_target(freertos_size
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
        DEPENDS ${PROJECT_NAME}.elf
        COMMENT "Analyzing FreeRTOS footprint..."
)