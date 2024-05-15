#include <linux/init.h>      // 模块加载init和卸载exit相关头文件
#include <linux/ide.h>       // printk等
#include <linux/module.h>    /*包含内核模块信息声明的相关函数*/
#include <linux/fs.h>        //file_operations结构体
#include <linux/types.h>
#include <linux/kernel.h>

#define CHRDEVBASE_MAJOR	200				/* 主设备号 */
#define CHRDEVBASE_NAME		"test_dev" 	/* 设备名     */

static uint8_t status;		/* 读缓冲区 */

static int testdrv_open(struct inode *inode, struct file *filp)
{
    printk("testdrv_open!\r\n");
    return 0;
}

static int testdrv_release(struct inode *inode, struct file *filp)
{
    printk("testdrv_release!\r\n");
    return 0;
}

static ssize_t testdrv_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    int retvalue = 0;

	/* 向用户空间发送数据 */
	retvalue = copy_to_user(buf, &status, cnt);
    if(retvalue == 0)
    {
        printk("test_dev driver senddata ok!\r\n");
    }
    else
    {
        printk("test_dev driver senddata failed!\r\n");
    }
    return 0;
}

static ssize_t testdrv_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
    int retvalue = 0;
	/* 接收用户空间传递给内核的数据并且打印出来 */
	retvalue = copy_from_user(&status, buf, cnt);
	if(retvalue == 0){
		printk("test_dev driver recevdata: %d\r\n", status);
	}else{
		printk("test_dev driver recevdata failed!\r\n");
	}
    return 0;
}

static struct file_operations test_drv_fop = {
    .owner = THIS_MODULE,
    .open = testdrv_open,
    .release = testdrv_release,
    .write = testdrv_write,
    .read = testdrv_read,
};

static int __init test_drv_init(void) //__init指将这段函数指定保存在__init段内存
{
    int retvalue = 0;
	/* 注册字符设备驱动 */
	retvalue = register_chrdev(CHRDEVBASE_MAJOR, CHRDEVBASE_NAME, &test_drv_fop);
	if(retvalue < 0){
		printk("test_dev driver register failed\r\n");
	}
	printk("test_dev driver init!\r\n");
    return 0;
}

static void __exit test_drv_exit(void) //__exit指将这段函数指定保存在__exit段内存
{
    /* 注销字符设备驱动 */
	unregister_chrdev(CHRDEVBASE_MAJOR, CHRDEVBASE_NAME);
    printk("test_dev driver success!\n");
}

//模块加载和卸载
module_init(test_drv_init); //test_drv_init作为模块函数入口
module_exit(test_drv_exit); //test_drv_exit作为模块函数出口

MODULE_LICENSE("GPL");                         //表示模块代码接受的软件许可协议
MODULE_AUTHOR("wt");                            //描述模块的作者信息
MODULE_INFO(intree, "Y");                       //表示此模块是主线内核源代码树的一部分。