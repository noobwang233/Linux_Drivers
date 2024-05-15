#include <linux/init.h>      // 模块加载init和卸载exit相关头文件
#include <linux/ide.h>       // printk等
#include <linux/module.h>    /*包含内核模块信息声明的相关函数*/

static int __init test_drv_init(void) //__init指将这段函数指定保存在__init段内存
{
    printk("test_drv_init success!\n");
    return 0;
}

static void __exit test_drv_exit(void) //__exit指将这段函数指定保存在__exit段内存
{
    printk("test_drv_exit success!\n");
}

//模块加载和卸载
module_init(test_drv_init); //test_drv_init作为模块函数入口
module_exit(test_drv_exit); //test_drv_exit作为模块函数出口

MODULE_LICENSE("GPL");                         //表示模块代码接受的软件许可协议
MODULE_AUTHOR("wt");                            //描述模块的作者信息
MODULE_INFO(intree, "Y");                       //表示此模块是主线内核源代码树的一部分。