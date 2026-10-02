/**
 * @file    NTC_Temperature.cpp
 * @brief   NTC 测温实现 — C++17 (LoveFinderLibForAT32)
 *
 * 所有换算逻辑都是 header 内联的 static 函数 (便于 host 侧单元自测)。
 * 本文件保留以维持构建系统的一致性, 并为将来放置非内联实现留位置。
 */
#include "NTC_Temperature.hpp"

namespace LoveFinderLib {

/* 当前无独立的非内联实现。
 * 若将来需要加入查表法 (LUT) 或分段线性拟合, 在此实现。 */

} // namespace LoveFinderLib
