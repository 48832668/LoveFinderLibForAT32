cmake_minimum_required(VERSION 3.22)

#
# LoveFinderLibForAT32 — 可复用外设驱动库 (纯 C++17)
#
# 本文件是库的【唯一】构建入口。任何 AT32 工程只需:
#   add_subdirectory(<LoveFinderLibForAT32> <build_dir>/lovefinder_lib)
#   target_link_libraries(<your_target> lovefinder_lib)
# 即可引入全部驱动。
#
# 依赖 (由宿主工程提供):
#   at32_workbench    — AT32 BSP 头文件与宏 (USE_STDPERIPH_DRIVER / AT32F421G8U7)
#   宿主工程 inc 目录  — 提供 lf_config.h (通过 include path)
#

if(NOT TARGET at32_workbench)
    message(FATAL_ERROR
        "LoveFinderLib requires the host project to provide the 'at32_workbench' "
        "INTERFACE target (AT32 BSP include dirs + compile definitions).")
endif()

#==============================================================================
# 库源文件清单
#==============================================================================
set(LF_LIB_SOURCES
    # ---- I2C 主机 (阻塞式, 直绑 i2c_type 实例) ----
    ${CMAKE_CURRENT_SOURCE_DIR}/I2C/i2c_master.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/I2C/i2c_sw.cpp

    # ---- 按键 ----
    ${CMAKE_CURRENT_SOURCE_DIR}/BUTTON/BUTTON.cpp

    # ---- 可选的 TIM 毫秒时基 (BUTTON_TIMEBASE_TIMER=1 时被引用) ----
    ${CMAKE_CURRENT_SOURCE_DIR}/TIMER/TIMER.cpp

    # ---- 快充控制器 ----
    ${CMAKE_CURRENT_SOURCE_DIR}/SW3526/SW3526.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SW2305/SW2305.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SW3538/SW3538.cpp

    # ---- Type-C / PD 控制器 ----
    ${CMAKE_CURRENT_SOURCE_DIR}/FUSB302/FUSB302.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FUSB302/PDSink.cpp

    # ---- 测温 ----
    ${CMAKE_CURRENT_SOURCE_DIR}/NTC_Temperature/NTC_Temperature.cpp

    # ---- LCD 与字库 ----
    ${CMAKE_CURRENT_SOURCE_DIR}/ST7735/st7735.cpp
)

# ---- FontLib (PickSoul 新架构: 像素在库, 编译清单在工程) ----
#   库: font.h (固定接口) + font_data.cpp (全部字形, 字符级编译) + font_manifest.json (真相源)
#   工程: font_config.hpp (只声明「编译哪些字」, 经 LF_PROJECT_INC_DIR 提供)
#   库目录里绝不能放 font_config.hpp —— 否则所有工程会共用一份清单。
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/FontLib/font_data.cpp")
    list(APPEND LF_LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/FontLib/font_data.cpp)
else()
    message(STATUS "LoveFinderLib: FontLib/font_data.cpp 不存在 (请用 PickSoul 生成)")
endif()

# C 源文件 (字库图标为纯数据 C 数组, 保持 C 编译)
set(LF_LIB_C_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/ST7735/icons.c
)

# Mode A 完整字库 (PickSoul 新架构下全部字形都在 font_data.cpp, 不再单独编译)

# 只加入实际存在的文件 —— 便于分阶段交付
set(LF_LIB_SOURCES_FILTERED "")
foreach(_src IN LISTS LF_LIB_SOURCES)
    if(EXISTS "${_src}")
        list(APPEND LF_LIB_SOURCES_FILTERED "${_src}")
    else()
        message(STATUS "LoveFinderLib: skip (not present) ${_src}")
    endif()
endforeach()

set(LF_LIB_C_SOURCES_FILTERED "")
foreach(_src IN LISTS LF_LIB_C_SOURCES)
    if(EXISTS "${_src}")
        list(APPEND LF_LIB_C_SOURCES_FILTERED "${_src}")
    else()
        message(STATUS "LoveFinderLib: skip (not present) ${_src}")
    endif()
endforeach()

#==============================================================================
# 目标定义
#==============================================================================
add_library(lovefinder_lib STATIC
    ${LF_LIB_SOURCES_FILTERED}
    ${LF_LIB_C_SOURCES_FILTERED}
)

#==============================================================================
# 头文件搜索路径
#==============================================================================
target_include_directories(lovefinder_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/I2C
    ${CMAKE_CURRENT_SOURCE_DIR}/BUTTON
    ${CMAKE_CURRENT_SOURCE_DIR}/TIMER
    ${CMAKE_CURRENT_SOURCE_DIR}/SW3526
    ${CMAKE_CURRENT_SOURCE_DIR}/SW2305
    ${CMAKE_CURRENT_SOURCE_DIR}/SW3538
    ${CMAKE_CURRENT_SOURCE_DIR}/FUSB302
    ${CMAKE_CURRENT_SOURCE_DIR}/NTC_Temperature
    ${CMAKE_CURRENT_SOURCE_DIR}/ST7735
    ${CMAKE_CURRENT_SOURCE_DIR}/FontLib
)

#==============================================================================
# 依赖
#==============================================================================
target_link_libraries(lovefinder_lib PUBLIC at32_workbench)

# 宿主工程 inc 目录 (提供 lf_config.h)。由宿主工程通过变量传入。
if(DEFINED LF_PROJECT_INC_DIR AND EXISTS "${LF_PROJECT_INC_DIR}")
    target_include_directories(lovefinder_lib PUBLIC ${LF_PROJECT_INC_DIR})
endif()

#==============================================================================
# 编译选项
#==============================================================================
target_compile_features(lovefinder_lib PUBLIC cxx_std_17)

target_compile_options(lovefinder_lib PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti -fno-exceptions -fno-threadsafe-statics>
)
