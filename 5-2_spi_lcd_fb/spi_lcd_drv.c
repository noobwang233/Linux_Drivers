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

/**************接线******************/
/* UART2_RXD SPI3_SCLk  ------ SCL          */
/* UART2_CTS SPI3_MOSI  ------ SDA          */
/* UART2_TXD SPI3_SS0   ------ CS           */
/* GPIO_4               ------ BLK          */
/* GPIO_2               ------ DC           */
/* GPIO_1               ------ RST          */



#define st7735s_CNT     1
#define st7735s_NAME    "st7735s"

#define DISP_W          160
#define DISP_H          128

#define LCD_W           DISP_W
#define LCD_H           DISP_H

#define RED             0xf800
#define GREEN           0x07e0
#define BLUE            0x001f
#define WHITE           0xffff
#define BLACK           0x0000
#define YELLOW          0xFFE0


/***************************** command ***************************/
 
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


struct st7735s_dev {
    dev_t devid;                /* 设备号       */
    struct cdev cdev;           /* cdev         */
    struct class *class;        /* 类           */
    struct device *device;      /* 设备         */
    struct device_node    *nd;  /* 设备节点     */
    int major;                  /* 主设备号     */
    void *private_data;         /* 私有数据     */
    int dc_gpio;                /* 命令选择引脚 */
    int res_gpio;               /* 屏幕复位引脚 */
    int bl_gpio;                /* 背光引脚     */
    int cs_gpio;                /* 背光引脚     */
};
static struct st7735s_dev st7735sdev;

void st7735s_reginit(struct st7735s_dev *dev);

/* st7735s指令集 */
struct spi_lcd_cmd {
    u8  reg_addr;               // command
    u8  len;                    // 需要从spi_lcd_datas数组里发出数据字节数
    int delay_ms;               // 此命令发送数据完成后，需延时多久
};

struct spi_lcd_cmd cmds[] = {
/*   cmd                数据长度    发送完成之后的延时 */
    {ST7735_SleepOut,       0,      120},
    {ST7735_FullColor,      3,      10},
    {ST7735_8Colors,        3,      10},
    {ST7735_InPartialMode,  6,      10},
    {ST7735_INVCTR,         1,      10},
    {ST7735_PWCTR1,         3,      10},
    {ST7735_PWCTR2,         1,      10},
    {ST7735_PWCTR3,         2,      10},
    {ST7735_PWCTR4,         2,      10},
    {ST7735_PWCTR5,         2,      10},
    {ST7735_VMCTR1,         1,      10},
    {ST7735_MADCTL,         1,      10},
    {ST7735_GMCTRP1,        16,     10},
    {ST7735_GMCTRN1,        16,     10},
    {ST7735_CASET,          4,      10},
    {ST7735_RASET,          4,      10},
    {ST7735_TEST,           1,      10},
    {ST7735_DIS_RAM_PW,     1,      10},
    {ST7735_COLMOD,         1,      10},
    {ST7735_DISPON,         0,      10}
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

void st7735s_fb_show(struct fb_info *fbi, struct spi_device *spi);

static int st7735s_fb_setcolreg(unsigned int regno, unsigned int red,
                                unsigned int green, unsigned int blue,
                                unsigned int transp, struct fb_info *info);

/* framebuffer资源 */
typedef struct {
    struct spi_device *spi;         //记录fb_info对象对应的spi设备对象
    struct task_struct *thread;     //记录线程对象的地址，此线程专用于把显存数据发送到屏的驱动ic
} st7735s_data_t;

struct fb_info *fbi;
struct fb_ops fops = {
    .owner		= THIS_MODULE,
    .fb_setcolreg	=  st7735s_fb_setcolreg,  //实现颜色寄存器设置函数
    .fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};


int fb_thread_func(void *data)
{
    printk("===========%s : %s=============\n", __FUNCTION__, "fb thread running...........");
    st7735s_data_t *ldata = fbi->par;

    while (1)
    {   
        if (kthread_should_stop())
            break;
        st7735s_fb_show(fbi, ldata->spi);
    }

    return 0;
}



static u32 pseudo_palette[LCD_W*LCD_H];
/*fb_bitfield结构体：fb缓存的RGB位域，该结构描述每一个像素显示缓冲区的组织方式，
假如为RGB565模式，R占5位=bit[11:15]，G占6位=bit[10:5] B占5位=bit[4:0] */
static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}


/* 颜色寄存器设置函数 */
static int st7735s_fb_setcolreg(unsigned int regno, unsigned int red,
			     unsigned int green, unsigned int blue,
			     unsigned int transp, struct fb_info *info)
{
	unsigned int val;
	
	if (regno > 16)
	{
		return 1;
	}

	/* 用red,green,blue三原色构造出val  */
	val  = chan_to_field(red,	&info->var.red);
	val |= chan_to_field(green, &info->var.green);
	val |= chan_to_field(blue,	&info->var.blue);
	
	pseudo_palette[regno] = val;
	return 0;
}


/* framebuffer创建函数 */
int st7735s_fb_create(struct spi_device *spi) //此函数在spi设备驱动的probe函数里被调用
{
	printk("===========%s : %s=============\n", __FUNCTION__, "start.");
    u8 *v_addr;
    u32 p_addr;
    st7735s_data_t *data;
	int X=LCD_H;
	int Y=LCD_W;
    /*
        coherent:连贯的
        分配连贯的物理内存
    */
    v_addr = dma_alloc_coherent(NULL, LCD_W*LCD_H*4, &p_addr, GFP_KERNEL);
	printk("===========%s : %s=============\n", __FUNCTION__, "dma_alloc_coherent (Done).");
    
    //额外分配st7735s_data_t类型空间
    fbi = framebuffer_alloc(sizeof(st7735s_data_t), NULL);
    if(fbi == NULL){
		printk("st7735s fbi alloc error!\n");
        return -1;
    }
    data = fbi->par; //data指针指向额外分配的空间
    data->spi = spi;

    fbi->pseudo_palette = pseudo_palette;
	fbi->var.activate       = FB_ACTIVATE_NOW;

    fbi->var.xres = LCD_H;
    fbi->var.yres = LCD_W;
    fbi->var.xres_virtual = X;
    fbi->var.yres_virtual = Y;
    fbi->var.bits_per_pixel = 32; // 屏是rgb565, 但QT程序只能支持32位.还需要在刷图时把32位的像素数据转换成rgb565
    // fbi->var.red.offset = 11;
    // fbi->var.red.length = 5;
    // fbi->var.green.offset = 5;
    // fbi->var.green.length = 6;
    // fbi->var.blue.offset = 0;
    // fbi->var.blue.length = 5;
    fbi->var.red.offset = 16;
    fbi->var.red.length = 8;
    fbi->var.green.offset = 8;
    fbi->var.green.length = 8;
    fbi->var.blue.offset = 0;
    fbi->var.blue.length = 8;

    strcpy(fbi->fix.id, "st7735s_fb");
    fbi->fix.smem_start = p_addr; //显存的物理地址
    fbi->fix.smem_len = X*Y*4; 
    fbi->fix.type = FB_TYPE_PACKED_PIXELS;
    fbi->fix.visual = FB_VISUAL_TRUECOLOR;
    fbi->fix.line_length = X*4;

    fbi->fbops = &fops;
    fbi->screen_base = v_addr; //显存虚拟地址
    fbi->screen_size = X*Y*4; //显存大小

    register_framebuffer(fbi);
	printk("===========%s : %s=============\n", __FUNCTION__, "register_framebuffer (Done).");
    data->thread = kthread_run(fb_thread_func, fbi, spi->modalias);

	printk("===========%s  :  %s=============\n", __FUNCTION__, "end.");
    return 0;    
}


/* framebuffer删除函数 */
void st7735s_fb_delete(void) //此函数在spi设备驱动remove时被调用
{
    st7735s_data_t *data = fbi->par;
    kthread_stop(data->thread); //让刷图线程退出
    unregister_framebuffer(fbi);
    dma_free_coherent(NULL, fbi->screen_size, fbi->screen_base, fbi->fix.smem_start);
    framebuffer_release(fbi);
}


/*
 * @description    : 向st7735s多个寄存器写入数据
 * @param - dev:  st7735s设备
 * @param - buf:  要写入的数据
 * @param - len:  要写入的数据长度
 * @return       :   操作结果
 */
static s32 st7735s_write_regs(struct st7735s_dev *dev, u8 *buf, u8 len)
{
    int ret;
    struct spi_message m;
    struct spi_transfer *t;
    struct spi_device *spi = (struct spi_device *)dev->private_data;

    /* 申请内存 */
    t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);

    /* 发送要写入的数据 */
    t->tx_buf = buf;                /* 要写入的数据 */
    t->len = len;                   /* 写入的字节数 */
    spi_message_init(&m);           /* 初始化spi_message */
    spi_message_add_tail(t, &m);    /* 将spi_transfer添加到spi_message队列 */
    ret = spi_sync(spi, &m);        /* 同步发送 */
    kfree(t);                       /* 释放内存 */
    return ret;
}

/*
 * @description    : 向st7735s指定寄存器写入指定的值，写一个寄存器
 * @param - dev:  st7735s设备
 * @param - buf: 要写入的值
 * @return   :    无
 */    
static void st7735s_write_onereg(struct st7735s_dev *dev, u8 buf)
{
    st7735s_write_regs(dev,&buf, 1);
}
/*
    funciton: 写一个命令
*/
void write_command(struct st7735s_dev *dev, u8 cmd)
{
    // dc , command:0
    gpio_set_value(dev->dc_gpio, 0); 
    printk("wite cmd: 0x%x\n", cmd);
    st7735s_write_onereg(dev,cmd);
}

/*
    funciton: 写数据
*/
static void write_datas(struct st7735s_dev *dev, u8 *data,int len)
{
    u32 i = 0;

    gpio_set_value(dev->dc_gpio, 1);
    printk("wite data len: %d\n", len);
    for (i = 0; i < len; i++) {
        printk("index: %d data: 0x%x\n", i, data[i]);
    }
    st7735s_write_regs(dev, data, len);
}

/*
    funciton: 写多个数据
*/
static void write_data_u16(struct st7735s_dev *dev, u16 data)
{
    u8 buf[2] = {0};
    buf[0] = (u8)(data >> 8);
    buf[1] = (u8)(data);

    write_datas(dev, buf, 2);
}

/*
    funciton: 写一个数据
*/
void write_data_u8(struct st7735s_dev *dev, u8 data)
{
    write_datas(dev, &data, 1);
}
/*
 * @description        : 打开设备
 * @param - inode     : 传递给驱动的inode
 * @param - filp     : 设备文件，file结构体有个叫做pr似有ate_data的成员变量
 *                       一般在open的时候将private_data似有向设备结构体。
 * @return             : 0 成功;其他 失败
 */
static int st7735s_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &st7735sdev; /* 设置私有数据 */
    st7735s_reginit(&st7735sdev);
    return 0;
}
/*
 * @description        : 关闭/释放设备
 * @param - filp     : 要关闭的设备文件(文件描述符)
 * @return             : 0 成功;其他 失败
 */
static int st7735s_release(struct inode *inode, struct file *filp)
{
    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);
    return 0;
}

/* st7735s操作函数 */
static const struct file_operations st7735s_ops = {
    .owner = THIS_MODULE,
    .open = st7735s_open,
    .release = st7735s_release,
};

/* 写入屏幕地址函数 */
void Address_set(struct st7735s_dev *dev,unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{ 
    write_command(dev,ST7735_CASET);
    write_data_u16(dev, x1);
    write_data_u16(dev, x2);

    write_command(dev,ST7735_RASET);
    write_data_u16(dev, y1);
    write_data_u16(dev, y2);

    write_command(dev,ST7735_RAMWR);
}

/*
    全屏填充函数
*/
void LCD_Set_color(struct st7735s_dev *dev, u16 Color)
{
    u16 i,j;      
    Address_set(dev,0,0,LCD_W-1,LCD_H-1);
    for(i=0;i<LCD_W;i++)
    {
        for (j=0;j<LCD_H;j++)
        {
            write_data_u16(dev, Color);
        }
    }
}
/*
    区域填充函数
*/
void LCD_Set_area_color(struct st7735s_dev *dev, unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, u16 Color)
{
    unsigned int x_len, y_len, i, j;

    Address_set(dev,x1,y1,x2,y2);
    //计算填充区域的长度和宽度，终点坐标减起点坐标+1
    x_len = x2 - x1 + 1;//计算x坐标的长度
    y_len = y2 - y1 + 1;//计算y坐标的长度

    for(i = 0; i < x_len; i++)
    {
        for(j = 0; j < y_len; j++)
        {
            write_data_u16(dev, Color);
        }
    }
}
/*
    描点函数
*/
void LCD_Set_point_color(struct st7735s_dev *dev, unsigned int x, unsigned int y, u16 Color)
{
    Address_set(dev,x,y,x,y);

    write_data_u16(dev, Color);
}
/*
    图片显示
*/
void LCD_Set_image(struct st7735s_dev *dev, const unsigned char *p) //显示图片
{
    unsigned char picH,picL;
    uint16_t i=0;

    LCD_Set_color(dev, WHITE); //清屏  

    Address_set(dev,0,0,LCD_W-2,LCD_H-3);

    for(i=0;i<20480;i++)
     {    
        picL=*(p+i*2);    //数据低位在前
        picH=*(p+i*2+1);
        write_data_u8(dev,picH);
        write_data_u8(dev,picL);
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

    gpio_set_value(st7735sdev.res_gpio, 0);
    mdelay(500);
    gpio_set_value(st7735sdev.res_gpio, 1);
    mdelay(500);

    n = 0; // n用于记录数据数组spi_lcd_datas的位置
    //发命令，并发出命令所需的数据
    for (i = 0; i < ARRAY_SIZE(cmds); i++) //命令
    {
        write_command(dev, cmds[i].reg_addr);
        if(cmds[i].len != 0)
        {
            write_datas(dev, &spi_lcd_datas[n], cmds[i].len);
            n = n + cmds[i].len;
        }
        if (cmds[i].delay_ms != 0) //如有延时则延时
            mdelay(cmds[i].delay_ms);
    }

    /* 全屏颜色填充测试 */
    LCD_Set_color(dev, WHITE);
    mdelay(1000);
    LCD_Set_color(dev, RED);
    mdelay(1000);
    LCD_Set_color(dev, GREEN);
    mdelay(1000);
    LCD_Set_color(dev, BLUE);
    mdelay(1000);
    LCD_Set_color(dev, BLACK);
    mdelay(1000);

    /* 区域颜色填充测试 */
    LCD_Set_area_color(dev,0,0,40,40,RED);
    mdelay(1000);

    /* 描点测试 */
    for(i=0;i<100;++i)
    {
        LCD_Set_point_color(dev,i,i,RED);
        LCD_Set_point_color(dev,100,i,GREEN);
        LCD_Set_point_color(dev,i,100,BLUE);
    }
    mdelay(1000);

    /* 图片显示测试 */
    LCD_Set_image(dev, gImage_image);

    printk("st7735s lcd init & test finish!\n");
}

/*
  * @description     : spi驱动的probe函数，当驱动与
  *                    设备匹配以后此函数就会执行
  * @param - client  : spi设备
  * @param - id      : spi设备ID
  * 
  */
static int st7735s_probe(struct spi_device *spi)
{
    struct device_node *pnd;
    int ret = 0;

    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);
    /* 1、构建设备号 */
    if (st7735sdev.major) {
        st7735sdev.devid = MKDEV(st7735sdev.major, 0);
        register_chrdev_region(st7735sdev.devid, st7735s_CNT, st7735s_NAME);
    } else {
        alloc_chrdev_region(&st7735sdev.devid, 0, st7735s_CNT, st7735s_NAME);
        st7735sdev.major = MAJOR(st7735sdev.devid);
    }
    /* 2、注册设备 */
    cdev_init(&st7735sdev.cdev, &st7735s_ops);
    cdev_add(&st7735sdev.cdev, st7735sdev.devid, st7735s_CNT);
    /* 3、创建类 */
    st7735sdev.class = class_create(THIS_MODULE, st7735s_NAME);
    if (IS_ERR(st7735sdev.class)) {
        return PTR_ERR(st7735sdev.class);
    }
    /* 4、创建设备 */
    st7735sdev.device = device_create(st7735sdev.class, NULL, st7735sdev.devid, NULL, st7735s_NAME);
    if (IS_ERR(st7735sdev.device)) {
        return PTR_ERR(st7735sdev.device);
    }

	/* 获取设备树中cs片选信号，请根据实际设备树修改*/
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


    /* 获取设备树中Res复位, DC(data or command), BL GPIO ，请根据实际设备树修改*/
    st7735sdev.nd = spi->dev.of_node;

    st7735sdev.res_gpio = of_get_named_gpio(st7735sdev.nd, "res-gpio", 0);
    if(st7735sdev.res_gpio < 0) {
        printk("can't get res-gpio!\r\n");
        goto get_err;
    }

    st7735sdev.dc_gpio = of_get_named_gpio(st7735sdev.nd, "dc-gpio", 0);
    if(st7735sdev.dc_gpio < 0) {
        printk("can't get dc-gpio!\r\n");
        goto get_err;
    }

    st7735sdev.bl_gpio = of_get_named_gpio(st7735sdev.nd, "bl-gpio", 0);
    if(st7735sdev.bl_gpio < 0) {
        printk("can't get bl-gpio!\r\n");
        goto get_err;
    }

    /* 设置GPIO为输出，并且输出高电平 */
    ret = gpio_direction_output(st7735sdev.res_gpio, 1);
    if(ret < 0) {
        printk("can't set res gpio!\r\n");
    }
    ret = gpio_direction_output(st7735sdev.dc_gpio, 1);
    if(ret < 0) {
        printk("can't set dc gpio!\r\n");
    }
    ret = gpio_direction_output(st7735sdev.bl_gpio, 1);
    if(ret < 0) {
        printk("can't set bl gpio!\r\n");
    }
	ret = gpio_direction_output(st7735sdev.cs_gpio, 1);
	if(ret < 0) {
		printk("can't set cs gpio!\r\n");
	}

    /*初始化spi_device */
    spi->mode = SPI_MODE_2;         /*MODE2，CPOL=1，CPHA=0*/
    spi_setup(spi);
    st7735sdev.private_data = spi;  /* 设置私有数据 */
    /* 初始化st7735s内部寄存器 */
    st7735s_reginit(&st7735sdev);

    return 0;

get_err:
    // 获取GPIO资源失败时，需要注销
    /* 删除设备 */
    cdev_del(&st7735sdev.cdev);
    unregister_chrdev_region(st7735sdev.devid, st7735s_CNT);
    /* 注销掉类和设备 */
    device_destroy(st7735sdev.class, st7735sdev.devid);
    class_destroy(st7735sdev.class);
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
    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);
    /* 删除设备 */
    cdev_del(&st7735sdev.cdev);
    unregister_chrdev_region(st7735sdev.devid, st7735s_CNT);
    /* 注销掉类和设备 */
    device_destroy(st7735sdev.class, st7735sdev.devid);
    class_destroy(st7735sdev.class);
    return 0;
}

/* 传统匹配方式ID列表 */
static const struct spi_device_id st7735s_id[] = {
    {"wt,st7735s", 0},  
    {}
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