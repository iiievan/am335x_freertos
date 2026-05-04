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
        ${CMAKE_CURRENT_SOURCE_DIR}/portable/MemMang/heap_4.c
)


# ARM Cortex-A8 specific port files
set(FREERTOS_PORT_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/portable/port.c
)

# ASM files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/portable/startup.s)
    list(APPEND FREERTOS_PORT_SOURCES
            ${CMAKE_CURRENT_SOURCE_DIR}/portable/startup.s
    )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/portable/exceptionhandlers.s)
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
        ${CMAKE_CURRENT_SOURCE_DIR}/portable
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/app  # For FreeRTOSConfig.h
)

# Compile definitions
target_compile_definitions(${FREERTOS_KERNEL} PRIVATE
        __ARM_ARCH_7A__
        ARM_ARCH_7A
        AM335X_FREERTOS
)

# Use the same architecture flags as BSP (hard float, NEON)
target_compile_options(${FREERTOS_KERNEL} PRIVATE
        -mcpu=cortex-a8
        -mtune=cortex-a8
        -mfloat-abi=hard
        -mfpu=neon
        -marm
        -mthumb-interwork
        -mlong-calls
        -fdata-sections
        -funsigned-char
        -ffunction-sections
        -Wall
        -Wextra
        -ffreestanding
)

# Assembly flags - важно: НЕ добавляем -x здесь, если он уже есть в toolchain
# Вместо этого используем свойство LANGUAGE
set_source_files_properties(
        ${CMAKE_CURRENT_SOURCE_DIR}/portable/exceptionhandlers.s
        ${CMAKE_CURRENT_SOURCE_DIR}/portable/startup.s
        PROPERTIES
        LANGUAGE ASM
        COMPILE_FLAGS "-x assembler-with-cpp"
)

# Optimization levels
target_compile_options(${FREERTOS_KERNEL} PRIVATE
        $<$<CONFIG:Debug>:-O0 -g3 -DDEBUG>
        $<$<CONFIG:Release>:-Os -g0 -DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-Os -g3 -DNDEBUG>
)

# Set C standard
set_target_properties(${FREERTOS_KERNEL} PROPERTIES
        POSITION_INDEPENDENT_CODE OFF
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS OFF
)