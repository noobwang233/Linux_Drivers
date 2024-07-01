#include "linux/dma-mapping.h"
#include "linux/fb.h"
#include "linux/gfp.h"
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fb.h>

static struct fb_info *lcd_fb_info;

static struct fb_ops lcd_ops = {
	.owner = THIS_MODULE,
	.fb_fillrect = sys_fillrect,
	.fb_copyarea = sys_copyarea,
	.fb_imageblit = sys_imageblit,
};


/*
 * @description : 驱动入口函数
 * @param : 无
 * @return
 */
static int __init lcd_init(void)
{
    dma_addr_t phy_addr;

    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);

    /* 1. 分配fb_info结构体空间 */
    lcd_fb_info = framebuffer_alloc(0, NULL);

    /* 2. 设置fb_info */

    /* 2.1 设置var */
    lcd_fb_info->var.xres = 800;            /* 分辨率 */
    lcd_fb_info->var.yres = 480;

    lcd_fb_info->var.bits_per_pixel = 24;   /* RGB888 */
    lcd_fb_info->var.red.offset = 16;
    lcd_fb_info->var.red.length = 8;
    lcd_fb_info->var.green.offset = 8;
    lcd_fb_info->var.green.length = 8;
    lcd_fb_info->var.blue.offset = 0;
    lcd_fb_info->var.blue.length = 8;

    /* 2.2 设置fix */
    lcd_fb_info->fix.smem_len = lcd_fb_info->var.xres * lcd_fb_info->var.yres * 3u;

    lcd_fb_info->screen_base = dma_alloc_writecombine(NULL, lcd_fb_info->fix.smem_len, &phy_addr, GFP_KERNEL);
    lcd_fb_info->fix.smem_start = phy_addr;         /* 物理地址 */
    lcd_fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
    lcd_fb_info->fix.visual = FB_VISUAL_TRUECOLOR;

    /* 3. 设置fbops */
    lcd_fb_info->fbops = &lcd_ops;

    /* 4. 注册fb_info */
    register_framebuffer(lcd_fb_info);

    /* 5. 硬件初始化 */

    return 0;
}

/*
 * @description : 驱动出口函数
 * @param : 无
 * @return
 */
static void __exit lcd_exit(void)
{
    printk("===========%s %d=============\n", __FUNCTION__, __LINE__);

    /* 1.注销fb_info */
    unregister_framebuffer(lcd_fb_info);

    /* 2.释放fb_info的空间 */
    framebuffer_release(lcd_fb_info);

}

module_init(lcd_init);
module_exit(lcd_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");