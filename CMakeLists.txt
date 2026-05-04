cmake_minimum_required(VERSION 3.24)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/am335x_bsp/cmake/toolchain.cmake)
    include(${CMAKE_CURRENT_SOURCE_DIR}/am335x_bsp/cmake/toolchain.cmake)
elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain.cmake)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain.cmake)
endif()

# project settings
project(am335x_freertos LANGUAGES ASM C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

set(APP ${PROJECT_NAME})
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/${APP}.ld)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LINKER_SCRIPT}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-gc-sections,--print-memory-usage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map,${APP}.map")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-warn-rwx-segments")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-e,Entry")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-u,Entry")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-u,__aeabi_uidiv")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-u,__aeabi_idiv")


add_subdirectory(am335x_bsp)
add_subdirectory(freertos_kernel)

set(APP_SOURCES app/init.cpp
                app/main.cpp)

add_executable(${APP}.elf ${APP_SOURCES})

target_link_libraries(${APP}.elf
        freertos_kernel
        am335x_bsp )

target_link_options(${APP}.elf PRIVATE
        -nostartfiles
        -lc
        -lgcc
)

set(HEX_FILE ${APP}.hex)
set(BIN_FILE ${APP}.bin)
add_custom_command(TARGET ${APP}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${APP}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${APP}.elf> ${BIN_FILE}
        COMMAND ${SIZE} $<TARGET_FILE:${APP}.elf>
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}\nSize information:"
)


