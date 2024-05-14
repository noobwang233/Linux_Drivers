#include <linux/init.h>    // 模块加载init和卸载exit相关头文件
#include <linux/ide.h>     // printk等

static int __init test_drv_init(void)
{
    printk("test_drv_init success!\n");
    return 0;
}

static void __exit test_drv_exit(void)
{
    printk("test_drv_exit success!\n");
}

//模块加载和卸载
module_init(test_drv_init);
module_exit(test_drv_exit);