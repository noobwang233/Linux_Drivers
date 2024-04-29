#include <linux/types.h>   // 定义了ssize_t的头文件
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/init.h>    // 模块加载init和卸载exit相关头文件
#include <linux/module.h>
#include <linux/cdev.h>         // cdev相关头文件
#include <linux/device.h>   //设备号dev_t相关头文件
#include <linux/errno.h>    //错误相关头文件
#include <linux/gpio.h>     //gpio子系统相关头文件
#include <linux/of_gpio.h>  //of_gpio函数相关头文件
#include <linux/of.h>       //of_函数相关头文件
#include <linux/miscdevice.h> //miscdevice头文件
#include <linux/platform_device.h> //platform_device头文件
#include <linux/of_platform.h> //platform of函数

#define LED_MISC_MINOR  233     /* LED_MISC的次设备号 */
#define LED_MISC_NAME  "led_misc"  /* 驱动的主设备名称 */
#define DEVICE_CNT    1        /* 设备数量 */

struct led_dev_t
{
    struct device_node *np;
    int gpio; /* led 所使用的 GPIO 编号 */
    struct miscdevice *misc;
    spinlock_t lock;
    struct platform_driver *p_driver;
    bool status;
};
static struct file_operations testdrv_fop;
struct platform_driver test_platform_driver;
/* MISC 设备结构体 */    /* 填充miscdevice结构体*/
static struct miscdevice led_miscdev = {
    .minor = LED_MISC_MINOR,
    .name = LED_MISC_NAME,
    .fops = &testdrv_fop,
};
struct led_dev_t led_dev_0 = {
    .misc = &led_miscdev,
    .p_driver = &test_platform_driver,
};

static int testdrv_open(struct inode *inode, struct file *filp)
{
    unsigned long flags;/*中断标记*/

    filp->private_data = &led_dev_0;
    spin_lock_irqsave(&led_dev_0.lock, flags);//上锁
    if(led_dev_0.status != true) //设备忙
    {
        spin_unlock_irqrestore(&led_dev_0.lock, flags);//释放锁
        pr_err("testdrv busy!\n");
        return -EBUSY;
    }
    led_dev_0.status = false;//占用设备
    spin_unlock_irqrestore(&led_dev_0.lock, flags);//释放锁
    printk("testdrv open!\r\n");
    return 0;
}

static int testdrv_release(struct inode *inode, struct file *filp)
{
    unsigned long flags;/*中断标记*/

    spin_lock_irqsave(&led_dev_0.lock, flags);//上锁
    led_dev_0.status = true;//释放设备
    spin_unlock_irqrestore(&led_dev_0.lock, flags);//释放锁
    printk("testdrv close!\r\n");
    return 0;
}
//typedef int		__kernel_ssize_t;
//typedef __kernel_ssize_t	ssize_t;
static ssize_t testdrv_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    int retvalue = 0;
    u8 status = 0;

    status = gpio_get_value(led_dev_0.gpio);
    /* 向用户空间发送数据 */
    retvalue = copy_to_user(buf, &status, cnt);
    if(retvalue == 0)
    {
        printk("kernel senddata ok!\r\n");
    }
    else
    {
        printk("kernel senddata failed!\r\n");
    }

    printk("testdrv read!\r\n");
    return 0;
}

static ssize_t testdrv_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
    int retvalue = 0;
    u8 cmd = 0;

    retvalue = copy_from_user(&cmd, buf, cnt);
    if(retvalue == 0)
    {
        printk("kernel recevdata:%d\r\n", cmd);
    }
    else
    {
        printk("kernel recevdata failed!\r\n");
    }

    switch (cmd)
    {
    case 0:// GPIO_ACTIVE_LOW
        gpio_set_value(led_dev_0.gpio, 0);
        printk("led on\n");
        break;
    case 1:
        gpio_set_value(led_dev_0.gpio, 1);
        printk("led off\n");
        break;
    default:
        gpio_set_value(led_dev_0.gpio, (gpio_get_value(led_dev_0.gpio)==0? 1:0));
        printk("led trigger\n");
        break;
    }
    return 0;
}

static int testdrv_probe(struct platform_device *device)
{
    int retvalue = 0;

    /* 初始化自旋锁*/
    spin_lock_init(&led_dev_0.lock);
    led_dev_0.status = true;
    /* 注册字符设备驱动 */
    /* 查找设备结点 */
    led_dev_0.np = of_find_compatible_node(NULL , NULL , "led_gpio");
    if (led_dev_0.np == NULL)
    {
        pr_err("find device node failed\n");
        return -EIO;
    }
    printk("find device node successfully! \n");
    /* 解析gpio属性 */
    led_dev_0.gpio = of_get_named_gpio(led_dev_0.np, "gpios", 0);
    if (led_dev_0.gpio <= 0)
    {
        pr_err("get gpio failed\n");
        return -EIO;
    }
    printk("get gpio %d successfully! \n", led_dev_0.gpio);
    /* 申请gpio */
    retvalue = gpio_request(led_dev_0.gpio, "led_gpio");
    if (retvalue != 0)
    {
        pr_err("request gpio failed\n");
        return -EIO;
    }
    printk("request gpio %d successfully! \n", led_dev_0.gpio);
    /* 设置为输出且默认为1 GPIO_ACTIVE_LOW 关闭led*/
    retvalue = gpio_direction_output(led_dev_0.gpio, 1);
    if (retvalue != 0)
    {
        pr_err("request gpio output failed\n");
        goto freegpio;
    }
    printk("request gpio %d output successfully! \n", led_dev_0.gpio);

    /* 注册MISC设备 */
    retvalue = misc_register(led_dev_0.misc);
    if(retvalue != 0) 
    {
        pr_err("cannot register misc driver\n");
        goto freegpio;
    }
    printk("led_misc_init() success! major=%d,minor=%d\r\n", 10, led_dev_0.misc->minor);
    return 0;
freegpio:
    gpio_free(led_dev_0.gpio);
    return -EIO;
}

static int __init testdrv_init(void)
{
    int retvalue = 0;

    /* 注册platform驱动 */
    retvalue = platform_driver_register(led_dev_0.p_driver);
    if(retvalue != 0)
    {
        pr_err("platform driver register failed!\n");
        return -EIO;
    }
    printk("platform driver register success!\n");
    return 0;
}

static void __exit testdrv_exit(void)
{
    /* 注销platform驱动 */
    platform_driver_unregister(led_dev_0.p_driver);
    printk("platform driver unregister success!\n");
}

static int testdrv_remove(struct platform_device *device)
{
    /* 注销字符设备驱动 */
    misc_deregister(led_dev_0.misc);
    gpio_free(led_dev_0.gpio);
    printk("led_misc deinit\n");
    printk("led_misc_remove() success!\r\n");
    return 0;
}

static struct file_operations testdrv_fop = {
    .owner = THIS_MODULE,
    .open = testdrv_open,
    .release = testdrv_release,
    .write = testdrv_write,
    .read = testdrv_read,
};

const struct of_device_id leds_of_match_table[] = {
    {.compatible = "led_gpio",},
    {},
};

struct platform_driver test_platform_driver = {
    .probe = testdrv_probe,
    .remove = testdrv_remove,

    .driver = {
        .name = "led_gpio",
        .of_match_table = leds_of_match_table,
    },
};


module_init(testdrv_init);
module_exit(testdrv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");
MODULE_INFO(intree, "Y");
