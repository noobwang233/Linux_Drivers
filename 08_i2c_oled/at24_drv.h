#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>      //i2c子系统相关头文件


struct at24_platform_data {
	u32		byte_len;		/* size (sum of all addr) */
	u16		page_size;		/* for writes */
	u8		flags;          
};


// AT24设备结构体定义
struct at24_dev_data_t
{
    dev_t  dev;
    struct i2c_driver *i2cdriver; // i2c_driver结构体
    struct miscdevice *miscdev; //杂项字符结构体
    struct mutex lock;         //互斥锁
    struct i2c_client *client; //因为只有一个i2c地址，所以只用一个指针变量
    struct at24_platform_data chip; //at24xx特有数据
    unsigned num_addresses; //i2c地址数量
    u8 *writebuf;
};