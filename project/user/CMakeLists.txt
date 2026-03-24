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

# 2. 龙芯交叉编译参数
set(COMMON_FLAGS "-O3 -march=loongarch64 -mtune=loongarch64 -fPIC -ffunction-sections -fdata-sections -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} -pthread -Wno-delete-non-virtual-dtor -Wno-placement-new")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# 交叉编译工具链
include(cross.cmake)

# 初始化源文件列表
set(DIR_SRCS "")

# 3. OpenCV 与 NCNN 配置
set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build") 
find_package(OpenCV REQUIRED)

set(NCNN_DIR "${CMAKE_CURRENT_LIST_DIR}/../../libraries/zf_components/ncnn")
set(NCNN_STATIC_LIB "${NCNN_DIR}/lib/libncnn.a")

# 4. 头文件搜索路径
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ../user ../code ../model
    ../../libraries/zf_common 
    ../../libraries/zf_device 
    ../../libraries/zf_driver
    ../../libraries/zf_components
    ${NCNN_DIR}/include
    ${NCNN_DIR}/include/ncnn
    ../../libraries/zf_components/tflm
    ../../libraries/zf_components/tflm/third_party/flatbuffers/include
    ../../libraries/zf_components/tflm/third_party/gemmlowp
    ../../libraries/zf_components/tflm/third_party/gemmlowp/fixedpoint
    ../../libraries/zf_components/tflm/third_party/ruy
    ../../libraries/zf_components/tflm/third_party/ruy/ruy
    ../../libraries/zf_components/tflm/tensorflow/lite/micro
    ../../libraries/zf_components/tflm/tensorflow/lite/micro/kernels
    ../../libraries/zf_components/tflm/tensorflow/lite/kernels/internal
    ../../libraries/zf_components/tflm/tensorflow/lite/core/api
    ../../libraries/zf_components/tflm/tensorflow/lite/core/c
    ../../libraries/zf_components/tflm/tensorflow/lite/schema
)

# 自动包含模型目录头文件
file(GLOB_RECURSE MODEL_DIRS LIST_DIRECTORIES true "${CMAKE_CURRENT_LIST_DIR}/../../model_zf/*")
foreach(dir ${MODEL_DIRS})
    if(IS_DIRECTORY ${dir})
        include_directories(${dir})
    endif()
endforeach()

# 5. 收集常规源文件
file(GLOB_RECURSE MODEL_ZF_SRCS "${CMAKE_CURRENT_LIST_DIR}/../../model_zf/*.cpp")
list(APPEND DIR_SRCS ${MODEL_ZF_SRCS})

file(GLOB_RECURSE MODEL_LQ_SRCS "${CMAKE_CURRENT_LIST_DIR}/../../model_lq/*.cpp")
list(APPEND DIR_SRCS ${MODEL_LQ_SRCS})

# 辅助宏：收集目录下的所有源文件并加入 DIR_SRCS
macro(add_dir_to_srcs DIR_PATH)
    aux_source_directory(${DIR_PATH} TEMP_LIST)
    list(APPEND DIR_SRCS ${TEMP_LIST})
    unset(TEMP_LIST)
endmacro()

add_dir_to_srcs(../user)
add_dir_to_srcs(../code)
add_dir_to_srcs(../model)
add_dir_to_srcs(../../libraries/zf_common)
add_dir_to_srcs(../../libraries/zf_device)
add_dir_to_srcs(../../libraries/zf_driver)
add_dir_to_srcs(../../libraries/zf_components)

# 6. 【核心修复】全盘递归搜索 TFLM 缺失的算子源文件
# 我们在 tflm 根目录下全局搜索这些文件名，防止路径深度对不上
set(TFLM_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../libraries/zf_components/tflm")
set(MISSING_OPS_FILES 
    logistic.cc 
    mean.cc 
    add.cc 
    mul.cc 
    concatenation.cc 
    kernel_util.cc 
    reduce.cc
)

set(TFLM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../libraries/zf_components/tflm")
set(TFLM_UNUSED_DIR "${TFLM_ROOT}/tensorflow/lite/micro/kernels/unused")

message(STATUS "--- 开始自动补全 TFLM 缺失算子 ---")
set(TFLM_KERNELS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../libraries/zf_components/tflm/tensorflow/lite/micro/kernels")

# 明确指定这几个下载好的官方源文件
set(OFFICIAL_OPS_SRCS
    "${TFLM_KERNELS_DIR}/logistic.cc"
    "${TFLM_KERNELS_DIR}/logistic_common.cc"  # <--- 新增
    "${TFLM_KERNELS_DIR}/reduce.cc"
    "${TFLM_KERNELS_DIR}/reduce_common.cc"    # <--- 新增
    "${TFLM_KERNELS_DIR}/unpack.cc"
    "${TFLM_KERNELS_DIR}/add.cc"
    "${TFLM_KERNELS_DIR}/mul.cc"
    "${TFLM_KERNELS_DIR}/concatenation.cc"
    "${TFLM_KERNELS_DIR}/kernel_util.cc"
    "${TFLM_KERNELS_DIR}/strided_slice.cc"
)

foreach(op_src ${OFFICIAL_OPS_SRCS})
    if(EXISTS ${op_src})
        list(APPEND DIR_SRCS ${op_src})
        message(STATUS "Official Op Added: ${op_src}")
    else()
        message(FATAL_ERROR "找不到官方算子文件，请检查下载路径: ${op_src}")
    endif()
endforeach()

# 强制将 kernels 目录加入包含路径，确保能找到刚才伪造的 logistic.h
include_directories("${TFLM_ROOT}/tensorflow/lite/micro/kernels")
message(STATUS "--- 算子补全结束 ---")

# 7. 生成可执行文件
link_directories(../../libraries/zf_components/tflm/lib)

add_executable(${PROJ_NAME} ${DIR_SRCS})

set_target_properties(${PROJ_NAME} PROPERTIES
    INSTALL_RPATH "/opt/ls_2k0300_env/opencv_4_10_build/lib"
    BUILD_WITH_INSTALL_RPATH ON
)

target_link_libraries(${PROJ_NAME} 
    ${OpenCV_LIBS}
    ${NCNN_STATIC_LIB}
    tflm 
    pthread 
    m 
    dl
)