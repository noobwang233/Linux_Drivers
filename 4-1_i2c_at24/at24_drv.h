#ifndef  __AT24_DRV_H
#define  __AT24_DRV_H

#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>      //i2c子系统相关头文件



// AT24c02设备结构体定义
struct at24_dev_data_t
{
    dev_t  dt;                   /* 设备号 	 */
    struct cdev *at24_cdev;         /* 字符设备结构体*/
	struct device *dev;	        /* 设备 	 */
    struct i2c_client *client;      //因为只有一个i2c地址，所以只用一个指针变量
    char    *writebuf;
    char    name[15];
};

#endif