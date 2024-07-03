#ifndef _SPI_LCD_DRV_H
#define _SPI_LCD_DRV_H
#include <linux/spi/spi.h>

struct st7735s_dev {
    struct spi_device *spi;     /* 使用的 spi 设备 */
    int dc_gpio;                /* 命令选择引脚 */
    int res_gpio;               /* 屏幕复位引脚 */
    int bl_gpio;                /* 背光引脚     */
    int cs_gpio;                /* 片选引脚     */
};

extern struct st7735s_dev st7735sdev;

#endif