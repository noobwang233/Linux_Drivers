#include <linux/spi/spi.h>
#include <linux/types.h>
#include "spi_transfer.h"
#include <linux/gpio.h>
#include "spi_lcd_drv.h"

#define ST_PAGE_SIZE 64

/*
 * @description    : 向st7735s多个寄存器写入数据
 * @param - dev:  st7735s设备
 * @param - buf:  要写入的数据
 * @param - len:  要写入的数据长度
 * @return       :   操作结果
 */
static s32 st7735s_write_regs(struct spi_device *spi, u8 *buf, u8 len)
{
    int ret;
    struct spi_message m;
    struct spi_transfer *t;

    /* 申请内存 */
    t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);

    /* 发送要写入的数据 */
    t->tx_buf = buf;                    /* 要写入的数据 */
    t->len = len;                       /* 写入的字节数 */
    spi_message_init(&m);            /* 初始化spi_message */
    spi_message_add_tail(t, &m);     /* 将spi_transfer添加到spi_message队列 */
    ret = spi_sync(spi, &m);   /* 同步发送 */
    kfree(t);                           /* 释放内存 */
    return ret;
}


/*
    funciton: 写一个命令
*/
void write_command(struct spi_device *spi, u8 cmd)
{
    // cs 0
    gpio_set_value(st7735sdev.cs_gpio, 0);
    // dc , command:0
    gpio_set_value(st7735sdev.dc_gpio, 0); 
    st7735s_write_regs(spi, &cmd, 1);
    // cs 1
    gpio_set_value(st7735sdev.cs_gpio, 1);
}

/*
    funciton: 写数据
*/
void write_datas(struct spi_device *spi, u8 *data,int len)
{
    int i = 0;
    int index = 0;
    // cs 0
    gpio_set_value(st7735sdev.cs_gpio, 0);
    // dc , data:1
    gpio_set_value(st7735sdev.dc_gpio, 1);

    for( i= 0; i < len / ST_PAGE_SIZE; i++)
    {
        index = i * ST_PAGE_SIZE;
        st7735s_write_regs(spi, data + (index), ST_PAGE_SIZE);
    }
    if ((len % ST_PAGE_SIZE) != 0) {
        st7735s_write_regs(spi, data + (index), (len % ST_PAGE_SIZE));
    }

    // cs 1
    gpio_set_value(st7735sdev.cs_gpio, 1);
}


/*
    funciton: 写u16数据
*/
void write_data_u16(struct spi_device *spi, u16 data)
{
    u8 buf[2] = {0};
    buf[0] = (u8)(data >> 8);
    buf[1] = (u8)(data);

    write_datas(spi, buf, 2);
}

/*
    funciton: 写u8数据
*/
void write_data_u8(struct spi_device *spi, u8 data)
{
    write_datas(spi, &data, 1);
}