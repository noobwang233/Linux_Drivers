#ifndef  __OLED_DRV_H
#define  __OLED_DRV_H

#include <linux/types.h>
#include <linux/i2c.h>      //i2c子系统相关头文件

#define IOC_OLED_DRAW_POINT             100
#define IOC_OLED_DRAW_LINE              101
#define IOC_OLED_DRAW_RECTANGLE         102
#define IOC_OLED_DRAW_RECTANGLE_FILLED  103

typedef enum {
    RGB444 = 0,
    RGB565,
    RGB666
} color_mode;


/* 变量声明 */
struct oled_buf{
    int x1;
    int y1;
    int x2;
    int y2;
    unsigned int color; /* RGB444 RGB565 RGB666 */
    color_mode cmode;
};

#endif