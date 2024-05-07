#include <linux/kernel.h>
#include <linux/types.h>   // 定义了ssize_t的头文件
#include <linux/ide.h>
#include <linux/init.h>    // 模块加载init和卸载exit相关头文件
#include <linux/module.h>
#include <linux/cdev.h>         // cdev相关头文件
#include <linux/device.h>   //设备号dev_t相关头文件
#include <linux/errno.h>    //错误相关头文件
#include <linux/gpio.h>     //gpio子系统相关头文件
#include <linux/of_gpio.h>  //of_gpio函数相关头文件
#include <linux/of.h>       //of_函数相关头文件
#include <linux/platform_device.h> //platform_device头文件
#include <linux/of_platform.h> //platform of函数
#include <asm/string.h>
#include <linux/slab.h> //kzalloc头文件
#include <linux/string.h>
#include <linux/poll.h>

#define LED_MAJOR 235
#define DEV_COUNT 4 // 最多4个led
#define LED_ON 1
#define LED_OFF 0
#define LED_FREE 1
#define LED_BUSY 0
/*private date*/
static u8 led_dev_count = 0; /* 设备计数 */
static struct class *led_cls;//设备类
/* device struct 设备结构体*/
struct led_dev_t
{
    dev_t dt;
    struct platform_device *led_pdev;
    int gpio; /* led 所使用的 GPIO 编号 */
    spinlock_t lock;
    atomic_t status; /*设备状态*/
    struct cdev *led_cdev; /*字符设备结构体*/
    struct class *cls;
    struct device *dev;
    wait_queue_head_t r_wait_list; //读等待队列
    wait_queue_head_t w_wait_list; //写等待队列
};
// match table
const struct of_device_id leds_of_match_table[] = {
    {.compatible = "led_gpio",},
    {},
};
struct led_dev_t *led_devs[DEV_COUNT] = {0};


/* private function declear */
// file_operation functions
static int led_drv_open(struct inode *inode, struct file *filp);
static int led_drv_release(struct inode *inode, struct file *filp);
static ssize_t led_drv_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt);
static ssize_t led_drv_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt);
unsigned int led_drv_poll(struct file *filp, struct poll_table_struct *wait);
static int led_drv_probe(struct platform_device *device);
static int led_drv_remove(struct platform_device *device);
//module init and exit
static int __init led_drv_init(void);
static void __exit led_drv_exit(void);
//private led init and deinit function
static int led_dev_init(struct led_dev_t **led_devs, u32 index);
static int led_drv_deinit(struct led_dev_t **led_devs, u32 index);

/* 驱动提供的文件操作结构体 */
static struct file_operations led_drv_fop = {
    .owner = THIS_MODULE,
    .open = led_drv_open,
    .release = led_drv_release,
    .write = led_drv_write,
    .read = led_drv_read,
    .poll = led_drv_poll,
};
/* 用于注册平台驱动的platform_driver结构体 */
static struct platform_driver led_platform_driver = {
    .probe = led_drv_probe,
    .remove = led_drv_remove,

    .driver = {
        .name = "led_drv",
        .of_match_table = leds_of_match_table,
    },
};


static int led_drv_open(struct inode *inode, struct file *filp)
{
    u32 index;

    for(index = 0; index < DEV_COUNT; index++)
    {
        if(led_devs[index] != NULL)
        {
            if(led_devs[index]->dt == inode->i_cdev->dev)
                break;
        }
    }
    if(index >= DEV_COUNT)
    {
        pr_err("can not find dev data in dev list!\n");
        return -EIO;
    }
    filp->private_data = led_devs[index];
    printk("open device file: %s",led_devs[index]->led_pdev->name);

    printk("led_drv open!\r\n");
    return 0;
}
unsigned int led_drv_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int mask = 0;
    struct led_dev_t *led_dev = filp->private_data;

    /*应用程序添加到等待队列中*/
    poll_wait(filp, &led_dev->w_wait_list, wait);
  
    if(atomic_read(&led_dev->status) == LED_FREE)/* 如果设备事件有效 */
    {
        mask = POLLOUT; /* POLLOUT */
    }
    return mask;
}
static int led_drv_release(struct inode *inode, struct file *filp)
{
    printk("led_drv close!\r\n");
    return 0;
}

static ssize_t led_drv_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    int retvalue = 0;
    u8 value = 0;
    struct led_dev_t *led_dev = filp->private_data;


    value = gpio_get_value(led_dev->gpio);
    /* 向用户空间发送数据 */
    retvalue = copy_to_user(buf, &value, 1);
    if(retvalue == 0)
    {
        printk("led_drv send data ok!\r\n");
    }
    else
    {
        printk("led_drv send data failed!\r\n");
        return -EIO;
    }
    printk("read led_drv finish!\r\n");
    return 0;
}

static ssize_t led_drv_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
    int retvalue = 0;
    u8 cmd = 0;
    struct led_dev_t *led_dev = filp->private_data;

    // 检查是否设置了O_NONBLOCK标志
    if (filp->f_flags & O_NONBLOCK) 
    {
        //非阻塞写入
        if(atomic_read(&led_dev->status) != LED_FREE)
        {
            pr_err("%s busy!\n",led_dev->led_pdev->name);
            return -EBUSY;
        }
    }
    else {
        //阻塞写入
        retvalue = wait_event_interruptible(led_dev->w_wait_list, atomic_read(&led_dev->status) == LED_FREE);//设备忙就加入等待队列
        if(retvalue)
            return retvalue;
    }
    /* 从用户空间读取数据 */
    atomic_set(&led_dev->status, LED_BUSY);//start wirte led busy
    retvalue = copy_from_user(&cmd, buf, sizeof(cmd));
    if(retvalue == 0)
    {
        printk("led_drv send data ok!\r\n");
    }
    else
    {
        printk("led_drv send data failed!\r\n");
        atomic_set(&led_dev->status, LED_FREE);//free led
        return -EIO;
    }
    switch (cmd)
    {
    case LED_ON:// GPIO_ACTIVE_LOW
    case LED_OFF:
        gpio_set_value(led_dev->gpio, cmd);
        printk("led %s\n",(cmd == LED_ON ? "on" : "off"));
        break;
    default:
        gpio_set_value(led_dev->gpio, (gpio_get_value(led_dev->gpio)==0? 1:0));
        printk("led trigger\n");
        break;
    }
    atomic_set(&led_dev->status, LED_FREE);
    if(waitqueue_active(&led_dev->w_wait_list))
    {
        wake_up_interruptible(&led_dev->w_wait_list);
    }
    return 0;
}
/*
led_dev: 按键设备的结构体指针
*/
static int led_dev_init(struct led_dev_t **led_devs, u32 index)
{
    int retvalue;
    struct device_node *np = led_devs[index]->led_pdev->dev.of_node;

    /* 获取gpio号 */
    led_devs[index]->gpio = of_get_named_gpio(np, "gpios", 0);
    if (led_devs[index]->gpio <= 0)
    {
        pr_err("get gpio failed\n");
        return -EIO;
    }
    printk("get gpio %d successfully! \n", led_devs[index]->gpio);

    /* 申请gpio */
    retvalue = gpio_request(led_devs[index]->gpio, "led_gpio");
    if (retvalue != 0)
    {
        pr_err("request gpio failed\n");
        return -EIO;
    }
    printk("request gpio %d successfully! \n", led_devs[index]->gpio);
    /* 设置为输出 */
    retvalue = gpio_direction_output(led_devs[index]->gpio, LED_OFF);
    if (retvalue != 0)
    {
        pr_err("set gpio %d input failed! \n", led_devs[index]->gpio);
        goto freegpio;
    }
    printk("set gpio %d input successfully! \n", led_devs[index]->gpio);

    /* 使用cdev注册字符设备 */
    led_devs[index]->led_cdev = cdev_alloc();//申请cdev字符设备的空间
    if(led_devs[index]->led_cdev == NULL )
    {
        pr_err("led_dev led_cdev kzalloc failed! \n");
        goto freegpio;
    }
    printk("led_dev led_cdev kzalloc successfully!\n");

    /*生成设备号*/
    #ifdef LED_MAJOR
        led_devs[index]->dt = MKDEV(LED_MAJOR, index);
        retvalue = register_chrdev_region(led_devs[index]->dt, 1, led_devs[index]->led_pdev->name);
        if(retvalue != 0)
        {
            pr_err("led_dev dev_t register failed! start alloc register!\n");
            retvalue = alloc_chrdev_region(&(led_devs[index]->dt), 0, 1, led_devs[index]->led_pdev->name);
            if(retvalue != 0)
            {
                pr_err("led_dev dev_t register failed! \n");
                goto freecdev;
            }
        }
    #else
        retvalue = alloc_chrdev_region(&(led_devs[index]->dt), 0, 1, led_devs[index]->led_pdev->name);
        if(retvalue != 0)
        {
            pr_err("led_dev dev_t register failed! \n");
            goto freecdev;
        }
    #endif
    printk("led register dev_t success! major=%d,minor=%d\r\n", MAJOR(led_devs[index]->dt), MINOR(led_devs[index]->dt));
    /*注册字符设备*/
    led_devs[index]->led_cdev->owner = THIS_MODULE;
    cdev_init(led_devs[index]->led_cdev, &led_drv_fop);
    printk("cdev_init success!\n");
    retvalue = cdev_add(led_devs[index]->led_cdev, led_devs[index]->dt, 1);
    if(retvalue != 0) 
    {
        pr_err("cannot register cdev driver\n");
        goto freedevt;
    }
    printk("cdev_add success!\n");
    /*生成设备节点*/
    led_devs[index]->dev = device_create(led_devs[index]->cls, NULL,  led_devs[index]->dt,  NULL,  "led_dev_%d", index);
    if(led_devs[index]->dev == NULL)
    {
        pr_err("device_create failed!\n");
        goto delcdev;
    }
    printk("led_dev_%d create success!\r\n", index);
    /* 初始化设备自旋锁*/
    spin_lock_init(&led_devs[index]->lock);
    atomic_set(&led_devs[index]->status, LED_FREE);
    /*初始化等待队列*/
    init_waitqueue_head(&led_devs[index]->r_wait_list);
    init_waitqueue_head(&led_devs[index]->w_wait_list);
    return 0;

//错误处理
delcdev:
    cdev_del(led_devs[index]->led_cdev);
freedevt:
    unregister_chrdev_region(led_devs[index]->dt, 1);
freecdev:
    if (led_devs[index]->led_cdev != NULL)
    {
        kfree(led_devs[index]->led_cdev);
    }
freegpio:
    gpio_free(led_devs[index]->gpio);
    return -EIO;
}

static int led_drv_deinit(struct led_dev_t **led_devs, u32 index)
{
    device_destroy(led_devs[index]->cls, led_devs[index]->dt);
    printk("device_destroy success!\n");
    cdev_del(led_devs[index]->led_cdev);
    printk("cdev_del success!\n");
    unregister_chrdev_region(led_devs[index]->dt, 1);
    printk("unregister_chrdev_region success!\n");
    if (led_devs[index]->led_cdev != NULL)
    {
        kfree(led_devs[index]->led_cdev);
    }
    gpio_free(led_devs[index]->gpio);
    printk("gpio_free success!\n");
    if (led_devs[index] != NULL)
    {
        kfree(led_devs[index]);
        printk("kfree(led_devs[index]) success!\n");
    }
    return 0;
}

static int led_drv_probe(struct platform_device *device)
{
    int retvalue = 0;
    u32 index;
    const char *str;
    struct device_node *np = device->dev.of_node;

    //check status
    retvalue = of_property_read_string(np, "status", &str);
    if(retvalue < 0)
    {
        return -EINVAL;
    }
    if(strcmp(str, "okay"))
    {
        return -EINVAL;
    }
    printk("%s status oled!\n", device->name);
    //get index
    retvalue = of_property_read_u32_array(np, "num", &index, 1);
    if(retvalue < 0)
    {
        pr_err("can not get index !\n");
        return -EINVAL;
    }
    printk("get index = %d \n", index);
    if(led_devs[index] != NULL)
    {
        printk("led_devs[index] != NULL, start deinit!\n");
        led_drv_deinit(led_devs, index);
    }
    //分配设备结构体空间
    led_devs[index] = kzalloc(sizeof(struct led_dev_t), GFP_KERNEL);
    if(led_devs[index] == NULL )
    {
        pr_err("led_devs[index] kzalloc failed! \n");
        return -EIO;
    }
    printk("led_dev kzalloc successfully!\n");
    led_devs[index]->led_pdev = device;
    led_devs[index]->cls = led_cls;
    retvalue = led_dev_init(led_devs, index);
    if(retvalue != 0)
    {
        goto freeled_dev;
    }
    printk("%s probe() success!\r\n",led_devs[index]->led_pdev->name);
    led_dev_count++;
    return 0;

freeled_dev:
    if (led_devs[index] != NULL)
        kfree(led_devs[index]);
    return retvalue;
}

static int led_drv_remove(struct platform_device *device)
{
    u32 index;

    for(index = 0 ; index <= DEV_COUNT; index++)
    {
        if(led_devs[index] != NULL)
        {
            if(!strcmp(led_devs[index]->led_pdev->name, device->name))
                break;
        }
    }
    if(index >= DEV_COUNT)
    {
        pr_err("can not find dev in dev list!\n");
        return -EIO;
    }
    /* 注销字符设备 */
    printk("%s remove() led_dev address %d!\r\n",led_devs[index]->led_pdev->name,(int)led_devs[index]);
    led_drv_deinit(led_devs, index);
    printk("%s remove() success!\r\n",led_devs[index]->led_pdev->name);
    led_dev_count--;
    return 0;
}

static int __init led_drv_init(void)
{
    int retvalue = 0;

    led_cls = class_create(THIS_MODULE, "led_drv");
    if(led_cls == NULL)
    {
        pr_err("create led_drv class failed! \n");
        return -EIO;
    }
    printk("class_create success!\n");
    /* 注册platform驱动 */
    retvalue = platform_driver_register(&led_platform_driver);
    if(retvalue != 0)
    {
        pr_err("led platform driver register failed!\n");
        return -EIO;
    }
    printk("led platform driver register success!\n");
    return 0;
}

static void __exit led_drv_exit(void)
{
    /* 注销platform驱动 */
    platform_driver_unregister(&led_platform_driver);
    printk("led platform driver unregister success!\n");
    class_destroy(led_cls);
    printk("class_destroy success!\n");
}

module_init(led_drv_init);
module_exit(led_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");
MODULE_INFO(intree, "Y");
