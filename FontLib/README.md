# FontLib — AT32 字库模块 (PickSoul 规范)

驱动无关的字符级编译字库。**不包含任何 MCU 头文件**，可独立编译。

## 文件说明

| 文件 | 语言 | 说明 |
|------|------|------|
| `fonts.h` | C/C++ | 公共接口：`FontDef` + 模式 A 字体声明 |
| `fonts_config.hpp` | 预处理 | ⚠ **工程侧文件**（不随库分发）：编译开关 `USE_FONT_*` 与子集字符集。库内 `#include "fonts_config.hpp"` 经 include path 解析到工程副本（仿 PY32 的 `font_config.hpp` 清单模式） |
| `fonts.c` | **C11** | 模式 A 完整字库 (`Font_7x10` / `Font_11x18` / `Font_16x26`) |
| `fonts_subset.h` | C/C++ | 模式 B 子集声明 + C++ 查表接口 |
| `fonts_subset.cpp` | **C++17** | 模式 B 字符级子集：每字符独立数组 + `switch` 查表 |
| `font.h` | — | 向后兼容转发头 → `fonts.h` |
| `font_config.hpp` | — | 向后兼容转发头 → `fonts_config.hpp`（工程侧） |
| `font_manifest.json` | — | PickSoul 唯一真相源 |

## 双模式字库

| 模式 | 判据 | 索引方式 | 数据文件 |
|------|------|----------|----------|
| **A** 传统完整 | `FontDef.data != nullptr` | 固定 `(ch - 32) * height` | `fonts.c` |
| **B** 字符级子集 | `FontDef.data == nullptr` | `subset_glyph_*` 查表 | `fonts_subset.cpp` |

模式 B 的每个字符是独立的 `static const uint16_t` 数组，只有被查表引用的字符
才会被链接器保留 —— 未编译的字符渲染端跳过，不影响其它字符。

```cpp
#include "fonts.h"
#include "fonts_subset.h"

ST7735_WriteString(10, 10, "HELLO", Font_7x10, ST7735::WHITE, ST7735::BLACK);
ST7735_WriteStringUTF8(20, 40, "\xE4\xBD\xA0\xE5\xA5\xBD",
                       Font_Subset_ZH_16x16, ST7735::WHITE, ST7735::BLACK);
```

`font_get_glyph()` / `font_get_glyph_unicode()` 同时处理模式 A 与模式 B，
渲染端无需自行判断 `font.data`。

## 字模格式

- 每个字符 = `height` 个 `uint16_t`
- **bit15 = 最左列**
- 渲染：`for (j = 0; j < width; j++) if ((data[row] << j) & 0x8000) DrawPixel()`
- `16x26` 每字符 26 行，行宽 16 bit，同样用 `uint16_t` 行

## 命名约定

| 类别 | 命名 | 示例 |
|------|------|------|
| ASCII 子集字形 | `g7_<name>` | `g7_A`, `g7_space`, `g7_colon`, `g7_0` |
| 汉字子集字形 | `gzh_<hex>` | `gzh_4f60`, `gzh_597d` |
| 查表函数 | `subset_glyph_*` | `subset_glyph_7x10`, `subset_glyph_zh_16x16` |

查表 `switch` 由**工程清单宏**展开（见 `fonts_subset.cpp`）：

```cpp
#define FONT_CASE(cp, name) case cp: return g7_##name;
    FONT_SUBSET_7X10_CHARS(FONT_CASE)
```

ASCII 用**十进制码点** (`case 65:`)，汉字用**十六进制码点** (`case 0x4F60:`)。

## 编译开关

`fonts_config.hpp`：

| 宏 | 默认 | 作用 |
|----|------|------|
| `USE_FONT_FULL_7X10` | 1 | 编译 `Font_7x10` |
| `USE_FONT_FULL_11X18` | 1 | 编译 `Font_11x18` |
| `USE_FONT_FULL_16X26` | 1 | 编译 `Font_16x26` |
| `USE_FONT_SUBSET_7X10` | 1 | 编译 `Font_Subset_7x10` |
| `USE_FONT_SUBSET_ZH_16X16` | 0 | 编译 `Font_Subset_ZH_16x16` |
| `FONT_SUBSET_7X10_CHARS` | 全部 65 字形 | 7x10 子集字符集（X 宏清单） |
| `FONT_SUBSET_ZH_CHARS` | 你好 | 汉字子集码点（X 宏清单） |

开关为 0 时，对应字库的**声明与定义都不参与编译**。

### 字符集是 X 宏清单，不是字符串

```c
/* 工程侧 fonts_config.hpp —— 例如"只要数字"的工程 */
#define FONT_SUBSET_7X10_CHARS(CP) \
    CP(32, space) CP(58, colon)                                     \
    CP(48, 0) CP(49, 1) CP(50, 2) CP(51, 3) CP(52, 4)               \
    CP(53, 5) CP(54, 6) CP(55, 7) CP(56, 8) CP(57, 9)
```

`CP` 是形参名，展开时被替换为 `FONT_CASE`；每一项
`CP(<码点>, <数组名后缀>)` 对应 `fonts_subset.cpp` 里的一个 `g7_<后缀>` 数组。
`*_STR` 宏只是给人看的可读形式，不参与编译。

> ⚠ **形参名不要取 `X`**：字形名里有大写 `X`（`CP(88, X)`），
> 预处理会把作为**实参**的那个 `X` 也一并替换，生成 `g7_FONT_CASE` 而报错。
> 本库统一用 `CP`。

### 多工程互不干扰（已实测）

每个工程 include 各自的 `fonts_config.hpp`；库内每个字形是独立的
`static const uint16_t` 数组（`-fdata-sections`），未被 `switch` 引用的
由 `--gc-sections` 丢弃，不占 Flash：

| 工程清单内容 | 放入 `.rodata` 的 `g7_*` 数组 | FLASH |
|--------------|-------------------------------|-------|
| 空格 `:` `0-9` | 12 | 23 524 B |
| 全部 65 个 ASCII 字形 | 65 | 25 268 B |

（LoveFinder891 实测：清单只留数字后 FLASH 减少 1 744 B。）
于是「只要数字的工程」与「要大量中文的工程」共用同一个库，互不干扰。

## 移植

1. Include Paths 加入 `LoveFinderLibForAT32/FontLib`
2. 屏幕驱动 `#include "fonts.h"` + `#include "fonts_subset.h"`
3. 在**工程侧**（如 `project/inc/`）提供 `fonts_config.hpp`，控制哪些字体/字符编译进固件（库内**不带**此文件，`#include "fonts_config.hpp"` 经 include path 解析到工程副本）
4. `fonts.c` 用 **C 编译器**，`fonts_subset.cpp` 用 **C++17**

## 添加汉字

1. Python+PIL 生成 16x16 字模（每行 `uint16_t`，bit15 = 最左）
2. `fonts_subset.cpp` 添加 `static const uint16_t gzh_<hex>[16]`（`case` 由清单宏自动生成，不必手写）
3. 需要的工程在自己的 `fonts_config.hpp` 的 `FONT_SUBSET_ZH_CHARS` 里声明该码点（**没声明的工程不受影响**）
4. `font_manifest.json` 的 `codepoints` 同步登记
