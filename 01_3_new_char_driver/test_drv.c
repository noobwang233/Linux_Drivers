#include <linux/init.h>      // 模块加载init和卸载exit相关头文件
#include <linux/ide.h>       // printk等
#include <linux/module.h>    /*包含内核模块信息声明的相关函数*/
#include <linux/fs.h>        //file_operations结构体
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/cdev.h>      // cdev相关头文件
#include <linux/device.h>    // device相关头文件
#include <linux/errno.h>

#define NEWCHRLED_CNT			1		  	/* 设备号个数 */

struct test_dev{
    struct device *device;	/* 设备 	 */
	int major;				/* 主设备号	  */
	int minor;				/* 次设备号   */
    struct cdev cdev;		/* cdev 	*/
    char name[15];          /* 设备名称 	*/
    struct class *class;    /* 设备类 	*/
    dev_t devid;			/* 设备号 	 */
    unsigned char status;   /* 设备数据 */
};

static struct test_dev test_dev = {
    .name = "test_dev",
};

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
	retvalue = copy_to_user(buf, &test_dev.status, cnt);
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
	retvalue = copy_from_user(&test_dev.status, buf, cnt);
	if(retvalue == 0){
		printk("test_dev driver recevdata: %d\r\n", test_dev.status);
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
	/* 1、创建设备号 */
    if (test_dev.major) {		/*  定义了设备号 */
       test_dev.devid = MKDEV(test_dev.major, 0);
    } else {						/* 没有定义设备号 */
		alloc_chrdev_region(&test_dev.devid, 0, NEWCHRLED_CNT, test_dev.name);	/* 申请设备号 */
		test_dev.major = MAJOR(test_dev.devid);	/* 获取分配号的主设备号 */
		test_dev.minor = MINOR(test_dev.devid);	/* 获取分配号的次设备号 */
	}
    printk("test_dev major=%d,minor=%d\r\n",test_dev.major, test_dev.minor);	

	/* 2、初始化cdev */
	test_dev.cdev.owner = THIS_MODULE;
	cdev_init(&test_dev.cdev, &test_drv_fop);

	/* 3、添加一个cdev */
	cdev_add(&test_dev.cdev, test_dev.devid, NEWCHRLED_CNT);

	/* 4、创建类 */
	test_dev.class = class_create(THIS_MODULE, test_dev.name);
	if (IS_ERR(test_dev.class)) {
		return PTR_ERR(test_dev.class);
	}

	/* 5、创建设备 */
	test_dev.device = device_create(test_dev.class, NULL, test_dev.devid, NULL, test_dev.name);
	if (IS_ERR(test_dev.device)) {
		return PTR_ERR(test_dev.device);
	}

    return 0;
}

static void __exit test_drv_exit(void) //__exit指将这段函数指定保存在__exit段内存
{
	/* 注销字符设备驱动 */
	device_destroy(test_dev.class, test_dev.devid);
	class_destroy(test_dev.class);

	cdev_del(&test_dev.cdev);/*  删除cdev */
	unregister_chrdev_region(test_dev.devid, NEWCHRLED_CNT); /* 注销设备号 */
    printk("test_dev driver exit success!\r\n");
}

//模块加载和卸载
module_init(test_drv_init); //test_drv_init作为模块函数入口
module_exit(test_drv_exit); //test_drv_exit作为模块函数出口

MODULE_LICENSE("GPL");                         //表示模块代码接受的软件许可协议
MODULE_AUTHOR("wt");                            //描述模块的作者信息
MODULE_INFO(intree, "Y");                       //表示此模块是主线内核源代码树的一部分。