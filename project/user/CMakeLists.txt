cmake_minimum_required(VERSION 3.5.0)

# 1. 基础项目配置
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)  
get_filename_component(PROJ_NAME ${PARENT_DIR} NAME)
project(${PROJ_NAME} VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 2. 龙芯交叉编译参数 (保留 -O3 优化和内存段优化)
set(COMMON_FLAGS "-O3 -march=loongarch64 -mtune=loongarch64 -fPIC -ffunction-sections -fdata-sections -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} -pthread -Wno-delete-non-virtual-dtor -Wno-placement-new")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# 交叉编译工具链 (请确保 cross.cmake 与此文件同目录)
include(cross.cmake)

# 3. OpenCV 与 NCNN 配置
# OpenCV 路径
set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build") 
find_package(OpenCV REQUIRED)

# NCNN 路径与静态库定义
set(NCNN_DIR "${CMAKE_CURRENT_LIST_DIR}/../../libraries/zf_components/ncnn")
set(NCNN_STATIC_LIB "${NCNN_DIR}/lib/libncnn.a")

# 4. 头文件搜索路径
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ../user 
    ../code 
    ../model
    ../../libraries/zf_common 
    ../../libraries/zf_device 
    ../../libraries/zf_driver
    ../../libraries/zf_components
    ${NCNN_DIR}/include
    ${NCNN_DIR}/include/ncnn
)

# 5. 收集源文件
set(DIR_SRCS "")

macro(add_dir_to_srcs DIR_PATH)
    if(EXISTS ${DIR_PATH})
        aux_source_directory(${DIR_PATH} TEMP_LIST)
        list(APPEND DIR_SRCS ${TEMP_LIST})
        unset(TEMP_LIST)
    endif()
endmacro()

# 仅收集当前项目所需的目录
add_dir_to_srcs(../user)
add_dir_to_srcs(../code)
add_dir_to_srcs(../model) # 存放新的 .param 和 .bin 
add_dir_to_srcs(../../libraries/zf_common)
add_dir_to_srcs(../../libraries/zf_device)
add_dir_to_srcs(../../libraries/zf_driver)
add_dir_to_srcs(../../libraries/zf_components)

# 6. 生成可执行文件
add_executable(${PROJ_NAME} ${DIR_SRCS})

# 设置运行时库搜索路径 (主要针对 OpenCV 动态库)
set_target_properties(${PROJ_NAME} PROPERTIES
    INSTALL_RPATH "/opt/ls_2k0300_env/opencv_4_10_build/lib"
    BUILD_WITH_INSTALL_RPATH ON
)

# 7. 静态链接 NCNN 及系统库
# 注意：ncnn 放在 pthread/m/dl 之前
target_link_libraries(${PROJ_NAME} 
    ${OpenCV_LIBS}
    ${NCNN_STATIC_LIB}
    pthread 
    m 
    dl
)