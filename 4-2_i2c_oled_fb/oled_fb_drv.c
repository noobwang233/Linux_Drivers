#include <linux/kernel.h>
#include <linux/types.h>        // 定义了ssize_t的头文件
#include <linux/ide.h>
#include <linux/init.h>         // 模块加载init和卸载exit相关头文件
#include <linux/module.h>
#include <linux/i2c.h>          //i2c子系统相关头文件
#include <linux/fb.h>
#include <linux/delay.h>

/***************************************** 接线 *********************************************/
/* I2C1 SCL ---- SCL         */
/* I2C1 SDA ---- SDA         */


/***************************************** 宏定义 *********************************************/
#define SH1106_DATA         0x40
#define SH1106_COMMAND      0x00

#define SH1106_DISPLAY_ON   0xAF


/***************************************** 数据类型定义 *********************************************/
// OLED设备结构体定义
struct oled_dev_data_t
{
    dev_t               dt;             /* 设备号      */
    struct cdev         *oled_cdev;     /* 字符设备结构体*/
    struct device       *dev;           /* 设备      */
    struct i2c_client   *client;        /* 因为只有一个i2c地址，所以只用一个指针变量 */
    char                name[15];
    struct fb_info      fb;
};

struct sh1106_fb_par {
    struct i2c_client   *client;
    u32 width;
    u32 height;
    struct fb_info *info;
    u32 page_offset;
};


struct sh1106_fb_array {
    u8    type;
    u8    data[0];
};

/***************************************** 函数声明 *********************************************/
/* i2c 驱动probe 和remove 函数 */
static int oled_drv_probe(struct i2c_client *client, const struct i2c_device_id *dt);
static int oled_drv_remove(struct i2c_client *client);

/* 模块加载和卸载函数 */
static int __init oled_drv_init(void);
static void __exit oled_drv_exit(void);

/***************************************** 全局变量 *********************************************/
static struct fb_fix_screeninfo sh1106_fb_fix = {
    .id        = "oled_sh1106",
    .type        = FB_TYPE_PACKED_PIXELS,
    .visual        = FB_VISUAL_MONO10,
    .xpanstep    = 0,
    .ypanstep    = 0,
    .ywrapstep    = 0,
    .accel        = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo sh1106_fb_var = {
    .bits_per_pixel    = 1,
};

/* oled初始化参数 */
const uint8_t sh1106_InitCmd[] = {
    0xAE,//关闭显示

    0xD5,//设置时钟分频因子,震荡频率
    0x80,  //[3:0],分频因子;[7:4],震荡频率

    0xA8,//设置驱动路数
    0X3F,//默认(1/64)

    0xD3,//设置显示偏移
    0X00,//默认为0

    0x40,//设置显示开始行 [5:0],行数.

    0x8D,//电荷泵设置
    0x14,//bit2，开启/关闭
    0x20,//设置内存地址模式
    0x02,//[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
    0xA1,//段重定义设置,bit0:0,0->0;1,0->127;  A1

    0xC8,//设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数 (C0 翻转显示) C8

    0xDA,//设置COM硬件引脚配置
    0x12,//[5:4]配置  

    0x81,//对比度设置
    0x66,//1~255;默认0X7F (亮度设置,越大越亮)

    0xD9,//设置预充电周期
    0xf1,//[3:0],PHASE 1;[7:4],PHASE 2;

    0xDB,//设置VCOMH 电压倍率
    0x30,//[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

    0xA4,//全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)

    0xA6,//设置显示方式;bit0:1,反相显示;0,正常显示 

};





/***************************************** 函数定义 *********************************************/

/**
 * @brief 分配sh1106的通信数据buffer
 */
static struct sh1106_fb_array *sh1106_fb_alloc_array(u32 len, u8 type)
{
    struct sh1106_fb_array *array;

    array = kzalloc(sizeof(struct sh1106_fb_array) + len, GFP_KERNEL);
    if (!array)
        return NULL;

    array->type = type;

    return array;
}

/**
 * @brief 将sh1106的通信数据buffer通过i2c 写入sh1106
 */
static int sh1106_fb_write_array(struct i2c_client *client,
                 struct sh1106_fb_array *array, u32 len)
{
    int ret;

    len += sizeof(struct sh1106_fb_array);

    ret = i2c_master_send(client, (u8 *)array, len);
    if (ret != len) {
        dev_err(&client->dev, "Couldn't send I2C command.\n");
        return ret;
    }

    return 0;
}

/**
 * @brief 写命令给sh1106
 */
static inline int sh1106_fb_write_cmd(struct i2c_client *client, u8 cmd)
{
    struct sh1106_fb_array *array;
    int ret;

    array = sh1106_fb_alloc_array(1, SH1106_COMMAND);
    if (!array)
        return -ENOMEM;

    array->data[0] = cmd;

    ret = sh1106_fb_write_array(client, array, 1);
    kfree(array);

    return ret;
}

/**
 * @brief 刷新sh1106的显存，显示新画面
 */
static void sh1106_fb_update_display(struct sh1106_fb_par *par)
{
    struct sh1106_fb_array *array;
    u8 *vmem = par->info->screen_base;
    int i, j, k;

    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);
    array = sh1106_fb_alloc_array(par->width * par->height / 8,
                      SH1106_DATA);
    if (!array)
        return;

    /*
     * The screen is divided in pages, each having a height of 8
     * pixels, and the width of the screen. When sending a byte of
     * data to the controller, it gives the 8 bits for the current
     * column. I.e, the first byte are the 8 bits of the first
     * column, then the 8 bits for the second column, etc.
     *
     *
     * Representation of the screen, assuming it is 5 bits
     * wide. Each letter-number combination is a bit that controls
     * one pixel.
     *
     * A0 A1 A2 A3 A4
     * B0 B1 B2 B3 B4
     * C0 C1 C2 C3 C4
     * D0 D1 D2 D3 D4
     * E0 E1 E2 E3 E4
     * F0 F1 F2 F3 F4
     * G0 G1 G2 G3 G4
     * H0 H1 H2 H3 H4
     *
     * If you want to update this screen, you need to send 5 bytes:
     *  (1) A0 B0 C0 D0 E0 F0 G0 H0
     *  (2) A1 B1 C1 D1 E1 F1 G1 H1
     *  (3) A2 B2 C2 D2 E2 F2 G2 H2
     *  (4) A3 B3 C3 D3 E3 F3 G3 H3
     *  (5) A4 B4 C4 D4 E4 F4 G4 H4
     */

    for (i = 0; i < (par->height / 8); i++)
    {
        for (j = 0; j < par->width; j++)
        {
            u32 array_idx = i * par->width + j;
            array->data[array_idx] = 0;
            for (k = 0; k < 8; k++)
            {
                u32 page_length = par->width * i;
                u32 index = page_length + (par->width * k + j) / 8;
                u8 byte = *(vmem + index);
                u8 bit = byte & (1 << (j % 8));
                bit = bit >> (j % 8);
                array->data[array_idx] |= bit << k;
            }
        }
    }

    sh1106_fb_write_array(par->client, array, par->width * par->height / 8);
    kfree(array);
}


static ssize_t sh1106_fb_write(struct fb_info *info, const char __user *buf,
                                size_t count, loff_t *ppos)
{
    struct sh1106_fb_par *par = info->par;
    unsigned long total_size;
    unsigned long p = *ppos;
    u8 __iomem *dst;

    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);
    total_size = info->fix.smem_len;

    if (p > total_size)
        return -EINVAL;

    if (count + p > total_size)
        count = total_size - p;

    if (!count)
        return -EINVAL;

    dst = (void __force *) (info->screen_base + p);

    if (copy_from_user(dst, buf, count))
        return -EFAULT;

    sh1106_fb_update_display(par);

    *ppos += count;

    return count;
}


static void sh1106_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
    struct sh1106_fb_par *par = info->par;
    sys_fillrect(info, rect);
    sh1106_fb_update_display(par);
}

static void sh1106_fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
    struct sh1106_fb_par *par = info->par;
    sys_copyarea(info, area);
    sh1106_fb_update_display(par);
}

static void sh1106_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
    struct sh1106_fb_par *par = info->par;
    sys_imageblit(info, image);
    sh1106_fb_update_display(par);
}

static struct fb_ops sh1106_fb_ops = {
    .owner        = THIS_MODULE,
    .fb_read    = fb_sys_read,
    .fb_write    = sh1106_fb_write,
    .fb_fillrect    = sh1106_fb_fillrect,
    .fb_copyarea    = sh1106_fb_copyarea,
    .fb_imageblit    = sh1106_fb_imageblit,
};

static void sh1106_fb_deferred_io(struct fb_info *info,
                struct list_head *pagelist)
{
    sh1106_fb_update_display(info->par);
}

static struct fb_deferred_io sh1106_fb_defio = {
    .delay        = HZ,
    .deferred_io    = sh1106_fb_deferred_io,
};


static int sh1106_init(struct sh1106_fb_par *par)
{
    int ret;
    uint8_t i;
    const uint8_t *InitCmd = sh1106_InitCmd;
    const uint8_t len = sizeof(sh1106_InitCmd) / sizeof(sh1106_InitCmd[0]);

    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);
    for(i = 0; i < len; i++)
    {
        ret = sh1106_fb_write_cmd(par->client, InitCmd[i]);
        if (ret < 0)
            return ret;
    }

    /* Turn on the display */
    ret = sh1106_fb_write_cmd(par->client, SH1106_DISPLAY_ON);
    if (ret < 0)
        return ret;

    return 0;
}

static int oled_drv_probe(struct i2c_client *client, const struct i2c_device_id *dt)
{
    struct fb_info *info;
    struct device_node *node = client->dev.of_node;
    u32 vmem_size;
    struct sh1106_fb_par *par;
    u8 *vmem;
    int ret;

    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);

    if (!node) {
        dev_err(&client->dev, "No device tree data found!\n");
        return -EINVAL;
    }

    info = framebuffer_alloc(sizeof(struct sh1106_fb_par), &client->dev);
    if (!info) {
        dev_err(&client->dev, "Couldn't allocate framebuffer.\n");
        return -ENOMEM;
    }

    par = info->par;
    par->info = info;
    par->client = client;

    if (of_property_read_u32(node, "width", &par->width))
        par->width = 96;

    if (of_property_read_u32(node, "height", &par->height))
        par->height = 16;

    if (of_property_read_u32(node, "page-offset", &par->page_offset))
        par->page_offset = 0;

    vmem_size = par->width * par->height / 8;

    vmem = devm_kzalloc(&client->dev, vmem_size, GFP_KERNEL);
    if (!vmem) {
        dev_err(&client->dev, "Couldn't allocate graphical memory.\n");
        ret = -ENOMEM;
        goto fb_alloc_error;
    }

    info->fbops = &sh1106_fb_ops;
    info->fix = sh1106_fb_fix;
    info->fix.line_length = par->width / 8;
    info->fbdefio = &sh1106_fb_defio;

    info->var = sh1106_fb_var;
    info->var.xres = par->width;
    info->var.xres_virtual = par->width;
    info->var.yres = par->height;
    info->var.yres_virtual = par->height;

    info->var.red.length = 1;
    info->var.red.offset = 0;
    info->var.green.length = 1;
    info->var.green.offset = 0;
    info->var.blue.length = 1;
    info->var.blue.offset = 0;

    info->screen_base = (u8 __force __iomem *)vmem;
    info->fix.smem_start = (unsigned long)vmem;
    info->fix.smem_len = vmem_size;

    fb_deferred_io_init(info);

    i2c_set_clientdata(client, info);

    ret = sh1106_init(par);
    if (ret)
    {
        goto reset_oled_error;
    }


    ret = register_framebuffer(info);
    if (ret) {
        dev_err(&client->dev, "Couldn't register the framebuffer\n");
        goto reset_oled_error;
    }

    dev_info(&client->dev, "fb%d: %s framebuffer device registered, using %d bytes of video memory\n", info->node, info->fix.id, vmem_size);
    printk("==========%s success =======\n", __FUNCTION__);
    return 0;


reset_oled_error:
    fb_deferred_io_cleanup(info);
fb_alloc_error:
    framebuffer_release(info);
    printk("==========%s failed =======\n", __FUNCTION__);
    return ret;

}

/*
 * @description     : i2c驱动的remove函数，移除i2c驱动的时候此函数会执行
 * @param - client     : i2c设备
 * @return          : 0，成功;其他负值,失败
 */
static int oled_drv_remove(struct i2c_client *client)
{
    struct fb_info *info = i2c_get_clientdata(client);

    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);

    unregister_framebuffer(info);
    fb_deferred_io_cleanup(info);
    framebuffer_release(info);

    printk("==========%s success =======\n", __FUNCTION__);
    return 0;
}


/* 传统匹配方式ID列表 */
static const struct i2c_device_id oled_sh1106_ids[] = {
    { "wt,oled_sh1106",},
    { /* END OF LIST */ }
};

/* 设备树匹配列表 */
static const struct of_device_id oled_dev_match_table[] = 
{
    {.compatible = "wt,oled_sh1106",},
    {}
};
/* i2c_driver结构体 */
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

/*
 * @description : 驱动入口函数
 * @param : 无
 * @return
 */

static int __init oled_drv_init(void)
{
    int retvalue;

    //注册i2c_driver
    retvalue = i2c_add_driver(&oled_i2c_drv);
    if(!retvalue)
        printk("===========%s successfully =============\n", __FUNCTION__);
    else
        printk("===========%s failed =============\n", __FUNCTION__);
    return retvalue;
}


/*
 * @description : 驱动出口函数
 * @param : 无
 * @return
 */
static void __exit oled_drv_exit(void)
{
    // 注销i2c_driver
    i2c_del_driver(&oled_i2c_drv);
    printk("class_destroy success!\n");
}

module_init(oled_drv_init);
module_exit(oled_drv_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");
MODULE_INFO(intree, "Y");
