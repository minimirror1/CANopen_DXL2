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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME CANopen_DXL2)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here    
    # GM_DL 구현 파일
    Component/gm_motion_CANprotocol/common/GM_DL/Src/dl_can.c
    Component/gm_motion_CANprotocol/common/GM_DL/Src/dl_can_rx.c
    Component/gm_motion_CANprotocol/common/GM_DL/Src/dl_can_tx.c
    
    # GM_NET/PHD 구현 파일
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Src/net_phd_pid.c
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Src/net_phd_pid_speaker.c
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Src/net_phd_pid_sensor.c
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Src/net_phd_pid_motion.c
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Src/net_phd_pid_midi.c
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Src/net_phd_pid_inspection.c
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Src/net_phd_pid_lfs.c
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Src/net_phd_pid_init.c
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Src/net_phd_pid_error.c
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Src/net_phd_pid_edit.c
    
    # CANopenNode 구현 파일
    Component/CANopenNode_Com/sync.cpp
    Component/CANopenNode_Com/sdo.cpp
    Component/CANopenNode_Com/pdo.cpp
    Component/CANopenNode_STM32/CO_app_STM32.c
    Component/CANopenNode_STM32/CO_driver_STM32.c
    Component/CANopenNode_STM32/CO_storageBlank.c
    Component/cpp_tick/cpp_tick.cpp
    
    # CANopenNode 핵심 파일
    Component/CANopenNode/CANopen.c
    Component/CANopenNode/301/CO_Emergency.c
    Component/CANopenNode/301/CO_fifo.c
    Component/CANopenNode/301/CO_HBconsumer.c
    Component/CANopenNode/301/CO_NMT_Heartbeat.c
    Component/CANopenNode/301/CO_ODinterface.c
    Component/CANopenNode/301/CO_PDO.c
    Component/CANopenNode/301/CO_SDOclient.c
    Component/CANopenNode/301/CO_SDOserver.c
    Component/CANopenNode/301/CO_SYNC.c
    Component/CANopenNode/301/CO_TIME.c
    Component/CANopenNode/301/crc16-ccitt.c
    Component/CANopenNode/storage/CO_storage.c
    Component/CANopenNode/storage/CO_storageEeprom.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # gm_motion_CANprotocol 파일이 있는 경로 추가
    Component/gm_motion_CANprotocol/common/GM_APP/Inc
    Component/gm_motion_CANprotocol/common/GM_DL/Inc
    Component/gm_motion_CANprotocol/common/GM_NET/PDA/Inc
    Component/gm_motion_CANprotocol/common/GM_NET/PHA/Inc
    Component/gm_motion_CANprotocol/common/GM_NET/PHD/Inc
    Component/gm_motion_CANprotocol/common/GM_ERR/Inc
    Component/gm_motion_CANprotocol/common/GM_PRTC/Inc
    # Task_ZeroErr 헤더 파일 경로 추가
    Task_ZeroErr/Inc
    Task_MRS/Inc
    # cpp_tick 헤더 파일 경로 추가
    Component/cpp_tick
    # CANopen 관련 헤더 파일 경로 추가
    Component/CANopenNode_Com
    Component/CANopenNode_STM32
    # CANopenNode 라이브러리 추가
    Component/CANopenNode
    Component/CANopenNode/301
    Component/CANopenNode/storage
    Component/CANopenNode/example
    # 다른 필요한 헤더 파일 경로
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    
    # Add user defined libraries
)
