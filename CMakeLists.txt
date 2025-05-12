cmake_minimum_required(VERSION 2.8)
project(ixy C)

set(CMAKE_C_STANDARD 11)

add_compile_options(-g -O2 -march=native -fomit-frame-pointer
	-D_XOPEN_SOURCE=700
	-D_DEFAULT_SOURCE
	-Wall
	-Wextra
	-Wno-unused-parameter
	-Wno-unused-function
	-Wformat=2
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(PKT_SIZE 64 CACHE STRING "pkt size")
add_definitions(-DPKT_SIZE=${PKT_SIZE})
set(EXEC_TIME 1 CACHE STRING "exec time")
add_definitions(-DEXEC_TIME=${EXEC_TIME})

set(SOURCE_COMMON src/pci.c src/memory.c src/stats.c src/interrupts.c src/driver/device.c  src/driver/e1000.c src/driver/ixgbe.c src/driver/virtio.c src/libixy-vfio.c)

set(CMAKE_EXE_LINKER_FLAGS "-static")

# add_executable(ixy-pktgen src/app/ixy-pktgen.c ${SOURCE_COMMON})
# add_executable(ixy-fwd src/app/ixy-fwd.c ${SOURCE_COMMON})
# add_executable(ixy-pcap src/app/ixy-pcap.c ${SOURCE_COMMON})
add_executable(ixy-fwd-loopback src/app/ixy-fwd-loopback ${SOURCE_COMMON})
add_library(ixy-vfio SHARED src/libixy-vfio.c)
