/**
 * @file font.h
 * @brief FontLib 公共接口 — 驱动无关 (字体名册 + 查表 API) (生成物 — 请勿手工编辑)
 * @author LoveFinder
 *
 * 本文件由 PickSoul (FontHub Editor) 从 font_manifest.json (唯一真相源) 生成。
 * 字形像素数据属于【共享库】, 所有引用该库的工程一起更新。
 *
 * 本目录【不含】font_config.hpp —— 那是每个工程自己的编译清单,
 * 经 -I 路径提供 (见各工程 LoveFinderLib/FontLib/font_config.hpp)。
 */

#ifndef FONT_H
#define FONT_H

#include <stdint.h>

/* 工程编译清单 (每个工程自己的 LoveFinderLib/FontLib/font_config.hpp) */
#include "font_config.hpp"

/*============================================================================
 * 库的字体名册
 *   每个字体都是一套【唯一】的像素数据, 放在本库中, 所有工程共享。
 *   工程用 USE_<FONT> 决定是否编译, 用 <FONT>_CHARS(X) 决定编译哪些字。
 *   未声明的字体默认关闭。
 *==========================================================================*/
#ifndef USE_FULLFONT_STY1_11X18
#define USE_FULLFONT_STY1_11X18 0
#endif
#ifndef USE_FULLFONT_STY1_16X26
#define USE_FULLFONT_STY1_16X26 0
#endif
#ifndef USE_FULLFONT_STY1_7X10
#define USE_FULLFONT_STY1_7X10 0
#endif
#ifndef USE_NUMFONT_STY1_11X18
#define USE_NUMFONT_STY1_11X18 0
#endif
#ifndef USE_NUMFONT_STY1_9X16
#define USE_NUMFONT_STY1_9X16 0
#endif
#ifndef USE_FULLFONT_STY1_9X16
#define USE_FULLFONT_STY1_9X16 0
#endif

/*============================================================================
 * FontDef — 字库统一描述结构
 *   本库所有字体都是字符级编译 (data == nullptr, 走 font_get_glyph 查表)。
 *==========================================================================*/
typedef struct FontDef {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;

/* --- 字体实例 (由工程清单启用) --- */
#if USE_FULLFONT_STY1_11X18
extern FontDef FULLFONT_STY1_11x18;   /* 11x18 ascii, 95 个字形可用 */
#endif
#if USE_FULLFONT_STY1_16X26
extern FontDef FULLFONT_STY1_16x26;   /* 16x26 ascii, 95 个字形可用 */
#endif
#if USE_FULLFONT_STY1_7X10
extern FontDef FULLFONT_STY1_7x10;   /* 7x10 ascii, 68 个字形可用 */
#endif
#if USE_NUMFONT_STY1_11X18
extern FontDef NUMFONT_STY1_11x18;   /* 11x18 ascii, 10 个字形可用 */
#endif
#if USE_NUMFONT_STY1_9X16
extern FontDef NUMFONT_STY1_9X16;   /* 9x16 ascii, 12 个字形可用 */
#endif
#if USE_FULLFONT_STY1_9X16
extern FontDef FULLFONT_STY1_9X16;   /* 9x16 ascii, 43 个字形可用 */
#endif

/*============================================================================
 * 字形查表 (字符级编译)
 *   未编译的字符返回 nullptr -> 渲染端自动跳过 (不显示也不占位)
 *==========================================================================*/
/** 按 ASCII 码查字模 (ASCII 字体专用) */
const uint16_t* font_get_glyph(const FontDef& font, uint8_t ch);

/** 按 Unicode 码点查字模 (汉字等非 ASCII 字体专用) */
const uint16_t* font_get_glyph_unicode(const FontDef& font, uint16_t uni);

#endif /* FONT_H */
