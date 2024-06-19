#ifndef  __AT24_DRV_H
#define  __AT24_DRV_H

#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>      //i2c子系统相关头文件


#define IOC_W25Q128_READ  100
#define IOC_W25Q128_WRITE 101

struct at24_buf{
    int addr;
    int len;
    char *data;
};


#endif