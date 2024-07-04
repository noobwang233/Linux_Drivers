#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/fb.h>
#include "image.h"
#include "linux/printk.h"
#include "linux/slab.h"
#include "spi_transfer.h"
#include "spi_lcd_drv.h"

/************************ 接线 ****************************/
/* UART2_RXD    SPI3_SCLk   PIN18 ------ SCL          */
/* UART2_CTS    SPI3_MOSI   PIN20 ------ SDA          */
/* UART2_TXD    SPI3_SS0    PIN17 ------ CS           */
/* UART3_RXD    GPIO1_IO25  PIN35 ------ BLK          */
/* UART3_TXD    GPIO1_IO24  PIN33  ------ DC          */
/* GPIO_1                   PIN7  ------ RST          */


/*-----------------------------------------------------------
 * 
 * 宏定义
 * 
 *-----------------------------------------------------------
 */
#define st7735s_NAME    "st7735s"

#define RED             0xf800
#define GREEN           0x07e0
#define BLUE            0x001f
#define WHITE           0xffff
#define BLACK           0x0000
#define YELLOW          0xFFE0

/************************ 命令 ****************************/
 
#define ST7735_SleepOut         0x11    
#define ST7735_FullColor        0xB1    //in normal mode
#define ST7735_8Colors          0xB2    //in idle mode
#define ST7735_InPartialMode    0xB3    //
#define ST7735_INVCTR           0xB4    //display inversion control
#define ST7735_PWCTR1           0xC0    //power control 1
#define ST7735_PWCTR2           0xC1    //power control 2
#define ST7735_PWCTR3           0xC2    //power control 3
#define ST7735_PWCTR4           0xC3    //power control 4
#define ST7735_PWCTR5           0xC4    //power control 5
#define ST7735_VMCTR1           0xC5    //VCOM    control 1
#define ST7735_MADCTL           0x36    //Memory data access control
#define ST7735_GMCTRP1          0xE0    //Gamma '+'polarity Correction characteristics setting
#define ST7735_GMCTRN1          0xE1    //Gamma '-'polarity Correction characteristics setting
#define ST7735_COLMOD           0x3A    //interface pixel format
#define ST7735_TEST             0xF0    //Enable test command
#define ST7735_DIS_RAM_PW       0xF6    //Disable ram power save mode
#define ST7735_CASET            0x2A    //column address set
#define ST7735_RASET            0x2B    //Row Address Set
#define ST7735_RAMWR            0x2C    //Memory write
#define ST7735_DISPON           0x29    //display on


/*-----------------------------------------------------------
 * 
 * 类型声明
 * 
 *-----------------------------------------------------------
 */

/* framebuffer资源 */
typedef struct {
    struct spi_device *spi;         //记录fb_info对象对应的spi设备对象
    struct task_struct *thread;     //记录线程对象的地址，此线程专用于把显存数据发送到屏的驱动ic
} st7735s_data_t;

/* st7735s指令集 */
struct spi_lcd_cmd {
    u8  reg_addr;               // command
    u8  len;                    // 需要从spi_lcd_datas数组里发出数据字节数
    int delay_ms;               // 此命令发送数据完成后，需延时多久
};

/*-----------------------------------------------------------
 * 
 * 函数声明
 * 
 *-----------------------------------------------------------
 */

void st7735s_reginit(struct st7735s_dev *dev);
void st7735s_fb_show(struct fb_info *fbi, struct spi_device *spi);
static int st3775s_setcolreg(unsigned int regno, unsigned int red,
                             unsigned int green, unsigned int blue,
                             unsigned int trans, struct fb_info *fbi);
/*-----------------------------------------------------------
 * 
 * 全局变量
 * 
 *-----------------------------------------------------------
 */
struct st7735s_dev st7735sdev;

struct spi_lcd_cmd cmds[] = {
/*   cmd                数据长度    发送完成之后的延时 */
    {ST7735_SleepOut,       0,      120},
    {ST7735_FullColor,      3,      0},
    {ST7735_8Colors,        3,      0},
    {ST7735_InPartialMode,  6,      0},
    {ST7735_INVCTR,         1,      0},
    {ST7735_PWCTR1,         3,      0},
    {ST7735_PWCTR2,         1,      0},
    {ST7735_PWCTR3,         2,      0},
    {ST7735_PWCTR4,         2,      0},
    {ST7735_PWCTR5,         2,      0},
    {ST7735_VMCTR1,         1,      0},
    {ST7735_MADCTL,         1,      0},
    {ST7735_GMCTRP1,        16,     0},
    {ST7735_GMCTRN1,        16,     0},
    {ST7735_CASET,          4,      0},
    {ST7735_RASET,          4,      0},
    {ST7735_TEST,           1,      0},
    {ST7735_DIS_RAM_PW,     1,      0},
    {ST7735_COLMOD,         1,      0},
    {ST7735_DISPON,         0,      0}
};

/* st7735s数据集 */
u8 spi_lcd_datas[] = {
    0x01,0x2C,0x2D,
    0x01,0x2C,0x2D,
    0x01,0x2C,0x2D,0x01,0x2C,0x2D,
    0x07,
    0xA2,0x02,0x84,
    0xC5,
    0x0A,0x00,
    0x8A,0x2A,
    0x8A,0xEE,
    0x0E,
    0xA0,// 0x36配置：横屏RGB 0xA0 | 竖屏RGB 0xC0 | 横屏BGE 0xA8 | 竖屏RGB 0xC8
    0x0f,0x1a,0x0f,0x18,0x2f,0x28,0x20,0x22,0x1f,0x1b,0x23,0x37,0x00,0x07,0x02,0x10,
    0x0f,0x1b,0x0f,0x17,0x33,0x2c,0x29,0x2e,0x30,0x30,0x39,0x3f,0x00,0x07,0x03,0x10,
    0x00,0x00,0x00,0x7F,
    0x00,0x00,0x00,0x9F,
    0x01,
    0x00,
    0x05,
};

struct fb_info *fbi;

struct fb_ops fops = {
    .owner		= THIS_MODULE,
    .fb_setcolreg   = st3775s_setcolreg,
    .fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

/*-----------------------------------------------------------
 * 
 * 函数定义
 * 
 *-----------------------------------------------------------
 */

static u32 chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int st3775s_setcolreg(unsigned int regno, unsigned int red,
			   unsigned int green, unsigned int blue,
			   unsigned int trans, struct fb_info *fbi)
{
	u32 val;
	int ret = 1;

	dev_dbg(fbi->device, "%s, regno = %u\n", __func__, regno);

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (fbi->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (fbi->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fbi->pseudo_palette;

			val = chan_to_field(red, &fbi->var.red);
			val |= chan_to_field(green, &fbi->var.green);
			val |= chan_to_field(blue, &fbi->var.blue);

			pal[regno] = val;

			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}


int fb_thread_func(void *data)
{
    st7735s_data_t *ldata = fbi->par;

    printk("===========%s : %s=============\n", __FUNCTION__, "fb thread running...........");
    while (1)
    {   
        if (kthread_should_stop())
            break;
        st7735s_fb_show(fbi, ldata->spi);
    }

    return 0;
}

/* 写入屏幕地址函数 */
void Address_set(struct spi_device *spi, unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{ 
    write_command(spi,ST7735_CASET);
    write_data_u16(spi, x1);
    write_data_u16(spi, x2);

    write_command(spi,ST7735_RASET);
    write_data_u16(spi, y1);
    write_data_u16(spi, y2);

    write_command(spi,ST7735_RAMWR);
}


/*
    全屏填充函数
*/
void LCD_Set_color(struct spi_device *spi, u16 Color)
{
    int x, y;
    u32 height = fbi->var.yres;
    u32 width = fbi->var.xres;

    Address_set(spi,0,0,width,height);
    for (y = 0; y < height; y++)
    {
        for (x = 0; x < width; x++)
        {
            write_data_u16(spi, Color);
        }
    }
}


/* framebuffer线程刷屏函数 */
void st7735s_fb_show(struct fb_info *fbi, struct spi_device *spi)
{
    int x, y;
    u8 *p = (u8 *)(fbi->screen_base);
    u32 height = fbi->var.yres;
    u32 width = fbi->var.xres;
    u32 data_len;

    Address_set(spi,0,0,width,height);
    // data len of per line
    data_len = width * fbi->var.bits_per_pixel / 8;
    for (y = 0; y < height; y++)
    {
        for (x = 0; x < width; x++)
        {
            write_data_u16(spi,*(u16 *) (p + (y * data_len) + (x * 2)));
        }
    }
}


/*
 * st7735s内部寄存器初始化函数 
 * @param      : 无
 * @return     : 无
 */
void st7735s_reginit(struct st7735s_dev *dev)
{
    int i, n;

    /* reset st7735s */
    gpio_set_value(st7735sdev.res_gpio, 0);
    mdelay(100);
    gpio_set_value(st7735sdev.res_gpio, 1);
    mdelay(50);


    n = 0; // n用于记录数据数组spi_lcd_datas的位置
    //发命令，并发出命令所需的数据
    for (i = 0; i < ARRAY_SIZE(cmds); i++) //命令
    {
        write_command(dev->spi, cmds[i].reg_addr);
        if(cmds[i].len != 0)
        {
            write_datas(dev->spi, &spi_lcd_datas[n], cmds[i].len);
            n = n + cmds[i].len;
        }
        if (cmds[i].delay_ms != 0) //如有延时则延时
            mdelay(cmds[i].delay_ms);
    }

    /* 全屏颜色填充测试 */
    LCD_Set_color(dev->spi, RED);
    mdelay(100);
    LCD_Set_color(dev->spi, GREEN);
    mdelay(100);
    LCD_Set_color(dev->spi, BLUE);
    mdelay(100);

    printk("st7735s lcd init & test finish!\n");
}


/*-----------------------------------------------------------
 * 
 * framebuffer创建函数
 * 
 *-----------------------------------------------------------
 */

int st7735s_fb_create(struct spi_device *spi) //此函数在spi设备驱动的probe函数里被调用
{
    struct device_node *pnd;
    u32 height;
    u32 width;
    int ret;

    printk("===========%s : %d=============\n", __FUNCTION__, __LINE__);
    pnd = spi->dev.of_node;

    /* 获取物理分辨率 */
    ret = of_property_read_u32(pnd,  "height", &height);
    if (ret) {
        printk("get height error");
        return ret;
    }
    ret = of_property_read_u32(pnd,  "width", &width);
    if (ret) {
        printk("get width error");
        return ret;
    }

    /* 1. 分配fb_info结构体空间 */
    fbi = framebuffer_alloc(sizeof(st7735s_data_t), &spi->dev);
    if(fbi == NULL){
        printk("st7735s fbi alloc error!\n");
        return -1;
    }

    /* 2. 设置fb_info */
    /* 2.1 设置var */
    fbi->var.xres = width;          /* 分辨率 */
    fbi->var.yres = height;
    fbi->var.xres_virtual = width;  /* 虚拟分辨率 */
    fbi->var.yres_virtual = height;

    fbi->var.bits_per_pixel = 16;   /* RGB565 */
    fbi->var.red.offset = 11;
    fbi->var.red.length = 5;
    fbi->var.green.offset = 5;
    fbi->var.green.length = 6;
    fbi->var.blue.offset = 0;
    fbi->var.blue.length = 5;

    /* 2.2 设置fix */
    strcpy(fbi->fix.id, "st7735s_fb");
    fbi->fix.line_length = fbi->var.xres * fbi->var.bits_per_pixel / 8;
    // 物理内存大小
    fbi->fix.smem_len = fbi->var.xres * fbi->var.yres * fbi->var.bits_per_pixel / 8;

    // 虚拟内存地址和大小
    fbi->screen_base = devm_kzalloc(&spi->dev, fbi->fix.smem_len, GFP_KERNEL);
    if (fbi->screen_base == NULL) {
        printk("screen base alloc error!\n");
        goto free_framebuffer;
    }
    fbi->screen_size = fbi->fix.smem_len;
    // 显存的物理地址
    fbi->fix.smem_start = __pa(fbi->screen_base);
    fbi->fix.type = FB_TYPE_PACKED_PIXELS;
    fbi->fix.visual = FB_VISUAL_TRUECOLOR;

    /* 3. 设置fbops */
    fbi->fbops = &fops;

    /* 4. 设置调色板 */
    fbi->pseudo_palette = devm_kzalloc(&spi->dev, fbi->fix.smem_len, GFP_KERNEL);
    if (fbi->pseudo_palette == NULL) {
        printk("pseudo_palette alloc error!\n");
        goto free_screen_mem;
    }

    /* 5. 注册fb_info */
    ret = register_framebuffer(fbi);
    if (ret) {
        printk("register framebuffer device error, err code: %d\n", ret);
        goto free_pseudo_palette;
    }

    return 0;

free_pseudo_palette:
    devm_kfree(&spi->dev, fbi->pseudo_palette);
free_screen_mem:
    devm_kfree(&spi->dev, fbi->screen_base);
free_framebuffer:
    framebuffer_release(fbi);
    return -ENOMEM;
}

/*-----------------------------------------------------------
 * 
 * framebuffer删除函数
 * 
 *-----------------------------------------------------------
 */
void st7735s_fb_delete(struct spi_device *spi) //此函数在spi设备驱动remove时被调用
{
    unregister_framebuffer(fbi);
    devm_kfree(&spi->dev, fbi->pseudo_palette);
    devm_kfree(&spi->dev, fbi->screen_base);
    framebuffer_release(fbi);
}

/*-----------------------------------------------------------
 * 
 * probe 和 remove函数
 * 
 *-----------------------------------------------------------
 */


/*
  * @description     : spi驱动的probe函数，当驱动与
  *                    设备匹配以后此函数就会执行
  * @param - spi     : spi设备
  * 
  */
static int st7735s_probe(struct spi_device *spi)
{
    struct device_node *pnd;
    st7735s_data_t *data;
    int ret = 0;

    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);

	/* 1. 获取设备树中cs片选信号，使用软件cs */
    pnd = of_get_parent(spi->dev.of_node);
	if(pnd == NULL) {
		printk("ecspi1 node not find!\r\n");
		goto get_err;
	}
	st7735sdev.cs_gpio = of_get_named_gpio(pnd, "cs-gpio", 0);
	if(st7735sdev.cs_gpio < 0) {
		printk("can't get cs-gpio!\r\n");
		goto get_err;
	}


    /* 2. 获取设备树中Res复位, DC(data or command), BL GPIO */
    pnd = spi->dev.of_node;

    st7735sdev.res_gpio = of_get_named_gpio(pnd, "res-gpio", 0);
    if(st7735sdev.res_gpio < 0) {
        printk("can't get res-gpio!\r\n");
        goto get_err;
    }

    st7735sdev.dc_gpio = of_get_named_gpio(pnd, "dc-gpio", 0);
    if(st7735sdev.dc_gpio < 0) {
        printk("can't get dc-gpio!\r\n");
        goto get_err;
    }

    st7735sdev.bl_gpio = of_get_named_gpio(pnd, "bl-gpio", 0);
    if(st7735sdev.bl_gpio < 0) {
        printk("can't get bl-gpio!\r\n");
        goto get_err;
    }

    /* 3. 申请gpio */
    ret = gpio_request(st7735sdev.cs_gpio, "st7735s_cs");
    if (ret) {
        printk("request gpio %d failed\n", st7735sdev.cs_gpio);
        goto get_err;
    }

    ret = gpio_request(st7735sdev.res_gpio, "st7735s_res");
    if (ret) {
        printk("request gpio %d failed\n", st7735sdev.res_gpio);
        goto free_cs;
    }

    ret = gpio_request(st7735sdev.dc_gpio, "st7735s_dc");
    if (ret) {
        printk("request gpio %d failed\n", st7735sdev.dc_gpio);
        goto free_res;
    }

    ret = gpio_request(st7735sdev.bl_gpio, "st7735s_blk");
    if (ret) {
        printk("request gpio %d failed\n", st7735sdev.bl_gpio);
        goto free_dc;
    }

    /* 4. 设置GPIO为输出，并且输出高电平 */
    ret = gpio_direction_output(st7735sdev.cs_gpio, 1);
    if(ret < 0) {
        printk("can't set cs gpio!\r\n");
        goto free_bl;
    }
    ret = gpio_direction_output(st7735sdev.res_gpio, 1);
    if(ret < 0) {
        printk("can't set res gpio!\r\n");
        goto free_bl;
    }
    ret = gpio_direction_output(st7735sdev.dc_gpio, 1);
    if(ret < 0) {
        printk("can't set dc gpio!\r\n");
        goto free_bl;
    }
    ret = gpio_direction_output(st7735sdev.bl_gpio, 1);
    if(ret < 0) {
        printk("can't set bl gpio!\r\n");
        goto free_bl;
    }

    /* 5. 初始化spi_device */
    spi->mode = SPI_MODE_0;         /*MODE0，CPOL=0，CPHA=0*/
    spi_setup(spi);
    st7735sdev.spi = spi;

    /* 6. 注册framebuffer设备 */
    ret = st7735s_fb_create(spi);
    if(ret < 0) {
        goto free_bl;
    }

    /* 7. 初始化st7735s内部寄存器 */
    st7735s_reginit(&st7735sdev);

    /* 8. 运行内核线程 */
    data = fbi->par; //data指针指向额外分配的空间
    data->spi = spi;
    data->thread = kthread_run(fb_thread_func, fbi, spi->modalias);
    if (data->thread == NULL) {
        goto fb_delete;
    }

    return 0;

fb_delete:
    st7735s_fb_delete(spi);
free_bl:
    gpio_free(st7735sdev.bl_gpio);

free_dc:
    gpio_free(st7735sdev.dc_gpio);

free_res:
    gpio_free(st7735sdev.res_gpio);

free_cs:
    gpio_free(st7735sdev.cs_gpio);

get_err:
    // 获取GPIO资源失败时，需要注销
    printk("\n get error! \n");
    return -EINVAL;
}

/*
 * @description     : spi驱动的remove函数，移除spi驱动的时候此函数会执行
 * @param - client     : spi设备
 * @return          : 0，成功;其他负值,失败
 */
static int st7735s_remove(struct spi_device *spi)
{
    st7735s_data_t *data = fbi->par;

    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);
    kthread_stop(data->thread); //让刷图线程退出
    /* 注销fb */
    st7735s_fb_delete(spi);
    gpio_free(st7735sdev.bl_gpio);
    gpio_free(st7735sdev.dc_gpio);
    gpio_free(st7735sdev.res_gpio);
    gpio_free(st7735sdev.cs_gpio);
    return 0;
}


/*-----------------------------------------------------------
 * 
 * 驱动匹配全局变量
 * 
 *-----------------------------------------------------------
 */

/* 传统匹配方式ID列表 */
static const struct spi_device_id st7735s_id[] = {
    {"wt,st7735s", 0},  
    {/* Sentinel */ }
};

/* 设备树匹配列表 */
static const struct of_device_id st7735s_of_match[] = {
    { .compatible = "wt,st7735s" },
    { /* Sentinel */ }
};

/* SPI驱动结构体 */    
static struct spi_driver st7735s_driver = {
    .probe = st7735s_probe,
    .remove = st7735s_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "st7735s",
        .of_match_table = st7735s_of_match, 
    },
    .id_table = st7735s_id,
};

/*-----------------------------------------------------------
 * 
 * 模块入口和出口函数
 * 
 *-----------------------------------------------------------
 */

/*
 * @description : 驱动入口函数
 * @param : 无
 * @return
 */
static int __init st7735s_init(void)
{
    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);
    return spi_register_driver(&st7735s_driver);
}

/*
 * @description : 驱动出口函数
 * @param : 无
 * @return
 */
static void __exit st7735s_exit(void)
{
    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);
    spi_unregister_driver(&st7735s_driver);
}

module_init(st7735s_init);
module_exit(st7735s_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");