#ifndef __ICONS_H__
#define __ICONS_H__

#include <stdint.h>

/* 图标尺寸定义 */
#define ICON_WIDTH  12
#define ICON_HEIGHT 12
#define ICON_PIXELS (ICON_WIDTH * ICON_HEIGHT)

/* 图标数据结构 */
typedef struct {
    const uint16_t *data;  /* RGB565像素数据 */
} IconDef;

/* 图标枚举 - 方便索引 */
typedef enum {
    ICON_VOLTAGE = 0,   /* 电压/闪电 */
    ICON_CURRENT,       /* 电流 */
    ICON_BATTERY,       /* 电池 */
    ICON_CHECK,         /* 成功/勾 */
    ICON_CROSS,         /* 失败/叉 */
    ICON_WARNING,       /* 警告 */
    ICON_USB,           /* USB */
    ICON_COUNT          /* 图标总数 */
} IconIndex;

/* 外部图标声明 */
extern IconDef Icons[ICON_COUNT];

#ifdef __cplusplus
extern "C" {
#endif

/* 图标绘制函数 */
void ST7735_DrawIcon(uint16_t x, uint16_t y, IconIndex icon);

#ifdef __cplusplus
}
#endif

#endif /* __ICONS_H__ */
