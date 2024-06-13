#include <linux/kernel.h>
#include <linux/types.h>    // 定义了ssize_t的头文件
#include <linux/ide.h>
#include <linux/init.h>     // 模块加载init和卸载exit相关头文件
#include <linux/module.h>
#include <linux/device.h>   //设备号dev_t相关头文件
#include <linux/errno.h>    //错误相关头文件
#include <linux/slab.h>     //kzalloc头文件
#include <linux/cdev.h>     // cdev相关头文件
#include <linux/i2c.h>      //i2c子系统相关头文件
#include <linux/miscdevice.h> //使用miscdev
#include <linux/delay.h>      //usleep_range 头文件
#include "linux/err.h"
#include "linux/printk.h"
#include "oled_drv.h" 


// OLED参数
#define OLED_PAGE       8                      // OLED页数
#define OLED_ROW        8 * OLED_PAGE          // OLED行数
#define OLED_COLUMN     128                    // OLED列数
#define OLED_COLUMN_OFFSET 2
// 显存
volatile unsigned char OLED_GRAM[OLED_PAGE][OLED_COLUMN];   // 每个像素1 bit, 64 * 128 bit


#define OLED_WRITE_TIMEOUT     25
#define ERR_DBUG(fmt, ...)      printk(KERN_ERR "%s, LINE %d :  " pr_fmt(fmt), __func__, __LINE__, ##__VA_ARGS__)
#define loop_until_timeout(tout, op_time)                \
    for (tout = jiffies + msecs_to_jiffies(OLED_WRITE_TIMEOUT), op_time = 0; \
         op_time ? time_before(op_time, tout) : true;        \
         usleep_range(1000, 1500), op_time = jiffies) // 休眠时间在 1000 到 1500 微秒之间的随机时长



// OLED设备结构体定义
struct oled_dev_data_t
{
    dev_t  dt;                      /* 设备号 	 */
    struct cdev *oled_cdev;         /* 字符设备结构体*/
	struct device *dev;	            /* 设备 	 */
    struct i2c_client *client;      // 因为只有一个i2c地址，所以只用一个指针变量
    char   name[15];
};

/* private function declear */
// probe and remove
static int oled_drv_probe(struct i2c_client *client, const struct i2c_device_id *dt);
static int oled_drv_remove(struct i2c_client *client);
//file_operations
static long oled_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int oled_release(struct inode *inode, struct file *filp);
static int oled_open(struct inode *inode, struct file *filp);
//module init and exit
static int __init oled_drv_init(void);
static void __exit oled_drv_exit(void);
// 使用i2c_transfer
static ssize_t oled_write_i2c(struct oled_dev_data_t *oled, char *buf, size_t count);
/* private date definition */
static struct oled_dev_data_t oled_dev = {
    .name = "oled_sh1106",
};
static struct class *oled_cls;//设备类

//必须有这个，不然装载驱动不成功
static const struct i2c_device_id oled_sh1106_ids[] = {
    { "wt,oled_sh1106",},
    { /* END OF LIST */ }
};
static const struct of_device_id oled_dev_match_table[] = 
{
    {.compatible = "wt,oled_sh1106",}, //.data 可以放具体设备是参数吗
    {}
};
// i2c_driver结构体
static struct i2c_driver oled_i2c_drv = 
{
    .probe = oled_drv_probe,
    .remove = oled_drv_remove,
    .driver = {
        .name = "oled_drv",
        .owner = THIS_MODULE,
        .of_match_table = oled_dev_match_table,
    },
    .id_table = oled_sh1106_ids,
};

static struct file_operations oled_fop = 
{
    .owner = THIS_MODULE,
    .release = oled_release,
    .open = oled_open,
    .unlocked_ioctl = oled_ioctl,
};

static ssize_t oled_write_i2c(struct oled_dev_data_t *oled, char *buf, size_t count)
{
    unsigned long timeout, write_time;
    struct i2c_client *client;
    struct i2c_msg msg;
    ssize_t status = 0;


    if (count > OLED_COLUMN + 1)
        count = OLED_COLUMN + 1;

    client = oled->client;

    msg.addr = client->addr;
    msg.flags = 0;

    /* msg.buf is u8 and casts will mask the values */
    msg.buf = (char *)buf;
    msg.len = count;

    loop_until_timeout(timeout, write_time) {
        status = i2c_transfer(client->adapter, &msg, 1);
        if (status == 1)
            status = count;

        if (status == count)
            return count;
    }

    return -ETIMEDOUT;
}

/**
 * @brief 向OLED发送指令
 */
static int OLED_SendCmd(const char cmd) {
    int ret;
    static char sendBuffer[2] = {0};
    sendBuffer[1] = cmd;
    ret = oled_write_i2c(&oled_dev, sendBuffer, 2);
    if (ret != 2 ) {
        ERR_DBUG("i2c wirte failed\r\n");
        return -ECOMM;
    }
    else {
        return 0;
    }
}

// ========================== 显存操作函数 ==========================

/**
 * @brief 清空显存 绘制新的一帧
 */
static void OLED_NewFrame(void) {
    memset((void *)OLED_GRAM, 0, sizeof(OLED_GRAM));
}

static void OLED_PageSet(unsigned char page)
{
    OLED_SendCmd(0xb0+page);
}
static void OLED_ColumnSet(unsigned char column)
{
    column+=OLED_COLUMN_OFFSET;
    OLED_SendCmd((0x10 )|(column >> 4));        // 设置列地址高4位
    OLED_SendCmd((0x00 )|(column & 0x0F));      // 设置列地址低4位
}


/**
 * @brief 将当前显存显示到屏幕上
 * @note 此函数是移植本驱动时的重要函数 将本驱动库移植到其他驱动芯片时应根据实际情况修改此函数
 */
static void OLED_ShowFrame(void) {
    uint8_t i;
    static char sendBuffer[OLED_COLUMN + 1];
    sendBuffer[0] = 0x40;
    for (i = 0; i < OLED_PAGE; i++) {
        OLED_PageSet(i);    // 设置页地址
        OLED_ColumnSet(0);  // 设置列地址
        memcpy(&sendBuffer[1], (const void*)OLED_GRAM[i], OLED_COLUMN);
        oled_write_i2c(&oled_dev, sendBuffer, OLED_COLUMN + 1);
    }
}

/**
 * @brief 设置一个像素点
 * @param x 横坐标
 * @param y 纵坐标
 * @param color 颜色
 */
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color) {
    if (x >= OLED_COLUMN || y >= OLED_ROW) return;
    if (!color) {
        OLED_GRAM[y / 8][x] |= 1 << (y % 8);
    } else {
        OLED_GRAM[y / 8][x] &= ~(1 << (y % 8));
    }
}

// ========================== 图形绘制函数 ==========================
/**
 * @brief 绘制一条线段
 * @param x1 起始点横坐标
 * @param y1 起始点纵坐标
 * @param x2 终止点横坐标
 * @param y2 终止点纵坐标
 * @param color 颜色
 * @note 此函数使用Bresenham算法绘制线段
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color) {
    uint8_t x, y;
    static uint8_t temp = 0;
    if (x1 == x2) {
    if (y1 > y2) {
      temp = y1;
      y1 = y2;
      y2 = temp;
    }
    for (y = y1; y <= y2; y++) {
      OLED_SetPixel(x1, y, color);
    }
    } else if (y1 == y2) {
    if (x1 > x2) {
      temp = x1;
      x1 = x2;
      x2 = temp;
    }
    for (x = x1; x <= x2; x++) {
      OLED_SetPixel(x, y1, color);
    }
  } else {
    // Bresenham直线算法
    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;
    int16_t ux = ((dx > 0) << 1) - 1;
    int16_t uy = ((dy > 0) << 1) - 1;
    int16_t x = x1, y = y1, eps = 0;
    dx = abs(dx);
    dy = abs(dy);
    if (dx > dy) {
      for (x = x1; x != x2; x += ux) {
        OLED_SetPixel(x, y, color);
        eps += dy;
        if ((eps << 1) >= dx) {
          y += uy;
          eps -= dx;
        }
      }
    } else {
      for (y = y1; y != y2; y += uy) {
        OLED_SetPixel(x, y, color);
        eps += dx;
        if ((eps << 1) >= dy) {
          x += ux;
          eps -= dy;
        }
      }
    }
  }
}

static int oled_open(struct inode *inode, struct file *filp)
{
    // printk("open oled_dev:%s major=%d,minor=%d\r\n", oled_dev.client->name, MAJOR(oled_dev.dt), MINOR(oled_dev.dt));
    filp->private_data = &oled_dev;

    return 0;

}
static int oled_release(struct inode *inode, struct file *filp)
{
    // printk("close oled_dev:%s major=%d,minor=%d\r\n", oled_dev.client->name, MAJOR(oled_dev.dt), MINOR(oled_dev.dt));
    return 0;
}

static long oled_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int retvalue;
    struct oled_buf *user_buf = (struct oled_buf *)arg;
    struct oled_buf ker_buf;

    retvalue = copy_from_user(&ker_buf, user_buf, (sizeof(struct oled_buf)));
    if(retvalue < 0)
    {
        ERR_DBUG("copy_from_user failed!\n");
        return retvalue;
    }

    switch (cmd) {
        case IOC_OLED_DRAW_POINT:
        {
            OLED_NewFrame();

            break;
        }
        case IOC_OLED_DRAW_LINE:
        {

            break;
        }
        default:
            // assert
            break;
    }
    return 0;
}


/* function definition */
static int oled_drv_probe(struct i2c_client *client, const struct i2c_device_id *dt)
{
    int retvalue;

    // check the functionality if it supports i2c
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        ERR_DBUG("i2c adapter doesn't support i2c func!\n");
        return -EPFNOSUPPORT;
    }

    /* 使用cdev注册字符设备 */
    oled_dev.oled_cdev = cdev_alloc();//申请cdev字符设备的空间
    if(oled_dev.oled_cdev == NULL)
    {
        pr_err("oled_cdev cdev_alloc failed! \n");
        return -ENOMEM;
    }
    printk("oled_cdev cdev_alloc successfully!\n");

    /*生成设备号*/
    #ifdef OLED_MAJOR
        oled_dev.dt = MKDEV(OLED_MAJOR, 0);
        retvalue = register_chrdev_region(oled_dev.dt, 1, oled_dev.name);
        if(retvalue != 0)
        {
            pr_err("oled_dev dev_t register failed! start alloc register!\n");
            retvalue = alloc_chrdev_region(&(oled_dev.dt), 0, 1, oled_dev.name);
            if(retvalue != 0)
            {
                pr_err("oled_dev dev_t register failed! \n");
                goto freecdev;
            }
        }
    #else
        retvalue = alloc_chrdev_region(&(oled_dev.dt), 0, 1, oled_dev.name);
        if(retvalue != 0)
        {
            pr_err("oled_dev dev_t register failed! \n");
            goto freecdev;
        }
    #endif
    printk("oled register dev_t success! major=%d,minor=%d\r\n", MAJOR(oled_dev.dt), MINOR(oled_dev.dt));
    /*注册字符设备*/
    oled_dev.oled_cdev->owner = THIS_MODULE;
    cdev_init(oled_dev.oled_cdev, &oled_fop);
    printk("cdev_init success!\n");
    retvalue = cdev_add(oled_dev.oled_cdev, oled_dev.dt, 1);
    if(retvalue != 0) 
    {
        pr_err("cannot register cdev driver\n");
        goto freedevt;
    }
    printk("cdev_add success!\n");
    /*生成设备节点*/
    oled_dev.dev = device_create(oled_cls, NULL,  oled_dev.dt,  NULL,  "oled_dev");
    if(oled_dev.dev == NULL)
    {
        pr_err("device_create failed!\n");
        retvalue = -ENOMEM;
        goto freedevt;
    }
    printk("oled_dev create success!\r\n");

    oled_dev.client = client;
    memcpy(client->name, (const char *)oled_dev.name, strlen(oled_dev.name));
    printk("oled_drv_probe successfully!\n");
    return 0;

freedevt:
    unregister_chrdev_region(oled_dev.dt, 1); /* 注销设备号 */
    ERR_DBUG("free_misc successfully\n");
freecdev:
    if(oled_dev.oled_cdev != NULL)
        cdev_del(oled_dev.oled_cdev);
    ERR_DBUG("free cdev successfully\n");
    return retvalue;
}

static int oled_drv_remove(struct i2c_client *client)
{
    device_destroy(oled_cls, oled_dev.dt);
    printk("device_destroy success!\n");
    if(oled_dev.oled_cdev != NULL)
    {
        cdev_del(oled_dev.oled_cdev);
        printk("cdev_del success!\n");
    }
    unregister_chrdev_region(oled_dev.dt, 1);
    printk("unregister_chrdev_region success!\n");
    return 0;
}

static int __init oled_drv_init(void)
{
    int retvalue;
    oled_cls = class_create(THIS_MODULE, "led_drv");
    if(oled_cls == NULL)
    {
        pr_err("create oled class failed! \n");
        return -EIO;
    }
    printk("class_create success!\n");
    //注册i2c_driver
    retvalue = i2c_add_driver(&oled_i2c_drv);
    if(!retvalue)
        ERR_DBUG("successfully\n");
    else
         ERR_DBUG("failed\n");
    return retvalue;
}

static void __exit oled_drv_exit(void)
{
    //注销i2c_driver
    i2c_del_driver(&oled_i2c_drv);
    ERR_DBUG("successfully\n");
    class_destroy(oled_cls);
    printk("class_destroy success!\n");
}

module_init(oled_drv_init);
module_exit(oled_drv_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");
MODULE_INFO(intree, "Y");
