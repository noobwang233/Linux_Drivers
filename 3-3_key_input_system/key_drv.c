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
#include <linux/interrupt.h> //中断相关头文件
#include <linux/timer.h>
#include <linux/input.h>

#define GPIO_ACTIVE_HIGH 0
#define GPIO_ACTIVE_LOW 1

#define DEV_COUNT 2
/*private date*/
/* device struct 设备结构体*/
struct key_dev_t
{
    struct platform_device *key_pdev;
    int gpio; /* key 所使用的 GPIO 编号 */
    int irq; //中断号
    spinlock_t lock; /*设备自旋锁*/
    atomic_t key_value_temp; /*按键暂时状态*/
    struct input_dev *inputdev;// input_dev
    unsigned int kcode; //按键code
    struct timer_list timer; //用于消抖的计时器
    unsigned int active;
};
// match table
const struct of_device_id keys_of_match_table[] = {
    {.compatible = "my_key",},
    {},
};
struct key_dev_t *key_devs[DEV_COUNT] = {0};

/* private function declear */
//platform device function
static int key_drv_probe(struct platform_device *device);
static int key_drv_remove(struct platform_device *device);
//module init and exit
static int key_gpio_init(struct key_dev_t *key_dev);
static void key_gpio_deinit(struct key_dev_t *key_dev);
static int __init key_drv_init(void);
static void __exit key_drv_exit(void);
//private key init and deinit function
static int key_dev_init(struct key_dev_t *key_dev);
static int key_drv_deinit(struct key_dev_t *key_dev);
//irq handle function
irqreturn_t key_irq_handler(int irq, void *dev);

/* 用于注册平台驱动的platform_driver结构体 */
static struct platform_driver key_platform_driver = {
    .probe = key_drv_probe,
    .remove = key_drv_remove,

    .driver = {
        .name = "key_drv",
        .of_match_table = keys_of_match_table,
    },
};

/* 定时器回调函数 */ 
void key_time_function(unsigned long arg) 
{
    u8 value;
    unsigned long flags;
    struct key_dev_t *key_dev = (struct key_dev_t *)arg;

    spin_lock_irqsave(&key_dev->lock, flags);
    value = gpio_get_value(key_dev->gpio);
    if (key_dev->active == GPIO_ACTIVE_LOW) {
        value = !value;
    }
    if(value == atomic_read(&key_dev->key_value_temp))
    {
        //操作有效
        printk("key status changed value %d\n", value);
        //上报按键值
        input_report_key(key_dev->inputdev, key_dev->kcode, value);
        input_sync(key_dev->inputdev);
    }
    else
    {
        printk("invalid %d\n", value);
        //操作无效不更改键值
    }
    spin_unlock_irqrestore(&key_dev->lock, flags);
}

static int key_gpio_init(struct key_dev_t *key_dev)
{
    int retvalue;
    struct device_node *np = key_dev->key_pdev->dev.of_node;
    /* 获取gpio号 */
    key_dev->gpio = of_get_named_gpio(np, "key-gpio", 0);
    if (key_dev->gpio <= 0)
    {
        pr_err("get gpio failed\n");
        return -EIO;
    }
    printk("get gpio %d successfully! \n", key_dev->gpio);

    /* 申请gpio */
    retvalue = gpio_request(key_dev->gpio, "key_gpio");
    if (retvalue != 0)
    {
        pr_err("request gpio failed\n");
        return -EIO;
    }
    printk("request gpio %d successfully! \n", key_dev->gpio);
    /* 设置为输入 */
    retvalue = gpio_direction_input(key_dev->gpio);
    if (retvalue != 0)
    {
        pr_err("set gpio %d input failed! \n", key_dev->gpio);
        goto freegpio;
    }
    printk("set gpio %d input successfully! \n", key_dev->gpio);

    //irq init
    key_dev->irq = gpio_to_irq(key_dev->gpio);//get irq num
    if(key_dev->irq < 0)
    {
        pr_err("get irq num failed!\n");
        goto freegpio;
    }
    printk("get irq num %d success!\r\n", key_dev->irq);
    retvalue = request_irq(key_dev->irq, key_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, key_dev->key_pdev->name, key_dev);
    if(retvalue != 0) 
    {
        pr_err("request irq failed!\n");
        if(retvalue == -EBUSY) 
        {
            pr_err("irq already requset!\n");
        }
        goto freegpio;
    }
    return 0;

freegpio:
    gpio_free(key_dev->gpio);
    printk("gpio %d free success!\r\n", key_dev->gpio);
    return -EIO;
}


static void key_gpio_deinit(struct key_dev_t *key_dev)
{
    if(key_dev->irq >= 0)
    {
        free_irq(key_dev->irq,key_dev);
        printk("free_irq success!\n");
    }
    if(key_dev->gpio >= 0)
    {
        gpio_free(key_dev->gpio);
        printk("gpio_free success!\n");
    }
}
/*
key_dev: 按键设备的结构体指针
*/
static int key_dev_init(struct key_dev_t *key_dev)
{
    int retvalue;
    
    // init timer
    init_timer(&key_dev->timer); /* 初始化定时器 */
    key_dev->timer.function = key_time_function; /* 设置定时处理函数 */
    key_dev->timer.data = (unsigned long)(key_dev); //设置定时器处理函数的传入参数为设备结构体地址

    //init gpio
    retvalue = key_gpio_init(key_dev);
    if(retvalue != 0)
    {
        goto del_timer;
    }

    /* 注册并初始化 input_dev */
    key_dev->inputdev = input_allocate_device(); /* 申请 input_dev 内存*/
    if(key_dev->inputdev == NULL )
    {
        pr_err("key_dev input_allocate_device failed! \n");
        retvalue = -ENOMEM;
        goto gpiodeinit;
    }
    printk("key_dev input_allocate_device successfully!\n");
    key_dev->inputdev->name = key_dev->key_pdev->name;//set name

    /* 初始化input_dev，设置产生哪些事件 */
    __set_bit(EV_KEY, key_dev->inputdev->evbit); /* 设置产生按键事件 */
    __set_bit(EV_REP, key_dev->inputdev->evbit); /* 重复事件，比如按下去不放开，就会一直输出信息 */

    /* 初始化input_dev，设置产生哪些按键 */
    __set_bit(key_dev->kcode, key_dev->inputdev->keybit);

	/* 注册 input_dev */
	retvalue = input_register_device(key_dev->inputdev);
    if(retvalue != 0)
    {
        pr_err("input_register_device failed! \n");
        goto freeinput_dev;
    }
    /* 初始化设备自旋锁*/
    spin_lock_init(&key_dev->lock);
    return 0;

//错误处理
freeinput_dev:
    input_free_device(key_dev->inputdev);
    pr_err("input_free_device ! \n");
gpiodeinit:
    key_gpio_deinit(key_dev);
    pr_err("key_gpio_deinit ! \n");
del_timer:
    del_timer_sync(&key_dev->timer);
    pr_err("del_timer_sync ! \n");
    return retvalue;
}

static int key_drv_deinit(struct key_dev_t *key_dev)
{
    input_unregister_device(key_dev->inputdev);
    printk("input_unregister_device success!\n");
    if (key_dev->inputdev != NULL)
    {
        input_free_device(key_dev->inputdev);
        printk("input_free_device success!\n");
    }
    key_gpio_deinit(key_dev);
    del_timer_sync(&key_dev->timer);
    pr_err("del_timer_sync ! \n");
    if (key_dev != NULL)
    {
        kfree(key_dev);
        printk("kfree(key_dev) success!\n");
    }
    return 0;
}

static int key_drv_probe(struct platform_device *device)
{
    int retvalue = 0;
    u32 index;
    const char *str;
    struct device_node *np = device->dev.of_node;

    //check dev_status
    retvalue = of_property_read_string(np, "status", &str);
    if(retvalue < 0)
    {
        return -EINVAL;
    }
    if(strcmp(str, "okay"))
    {
        return -EINVAL;
    }
    printk("%s status okey!\n", device->name);
    //get index
    retvalue = of_property_read_u32_array(np, "num", &index, 1);
    if(retvalue < 0)
    {
        pr_err("can not get index !\n");
        return -EINVAL;
    }
    printk("get index = %d \n", index);
    if(key_devs[index] != NULL)
    {
        printk("key_devs[index] != NULL, start deinit!\n");
        key_drv_deinit(key_devs[index]);
    }
    //分配设备结构体空间
    key_devs[index] = kzalloc(sizeof(struct key_dev_t), GFP_KERNEL);
    if(key_devs[index] == NULL )
    {
        pr_err("key_devs[index] kzalloc failed! \n");
        return -ENOMEM;
    }
    printk("key_dev kzalloc successfully!\n");

    key_devs[index]->key_pdev = device;

    //get key_code
    retvalue = of_property_read_u32_array(np, "key_code", &key_devs[index]->kcode, 1);
    if(retvalue < 0)
    {
        pr_err("can not get kcode !\n");
        goto free_key_dev;
    }
    printk("get kcode %d !\n", key_devs[index]->kcode);
    //get active
    retvalue = of_property_read_u32_index(np, "key-gpio", 2, &key_devs[index]->active);
    if(retvalue < 0)
    {
        pr_err("can not get active !\n");
        goto free_key_dev;
    }
    printk("get active %d !\n", key_devs[index]->active);


    retvalue = key_dev_init(key_devs[index]);
    if(retvalue != 0)
    {
        goto free_key_dev;
    }

    printk("%s probe() success!\r\n",key_devs[index]->key_pdev->name);
    return 0;

free_key_dev:
    if (key_devs[index] != NULL)
    {
        kfree(key_devs[index]);
        printk("kfree(key_devs[index])\n");
    }
    return retvalue;
}

static int key_drv_remove(struct platform_device *device)
{
    u32 index;

    for(index = 0 ; index <= DEV_COUNT; index++)
    {
        if(key_devs[index] != NULL)
        {
            if(!strcmp(key_devs[index]->key_pdev->name, device->name))
                break;
        }
    }
    if(index >= DEV_COUNT)
    {
        pr_err("can not find dev in dev list!\n");
        return -EIO;
    }
    /* 注销字符设备 */
    printk("%s remove() key_dev address %d!\r\n",key_devs[index]->key_pdev->name,(int)key_devs[index]);
    key_drv_deinit(key_devs[index]);
    printk("%s remove() success!\r\n",key_devs[index]->key_pdev->name);
    return 0;
}

static int __init key_drv_init(void)
{
    int retvalue = 0;

    /* 注册platform驱动 */
    retvalue = platform_driver_register(&key_platform_driver);
    if(retvalue != 0)
    {
        pr_err("key platform driver register failed!\n");
        return -EIO;
    }
    printk("key platform driver register success!\n");
    return 0;
}

static void __exit key_drv_exit(void)
{
    /* 注销platform驱动 */
    platform_driver_unregister(&key_platform_driver);
    printk("key platform driver unregister success!\n");
}


irqreturn_t key_irq_handler(int irq, void *dev)
{
    u8 value;
    struct key_dev_t *key_dev = (struct key_dev_t *)dev;
    printk("%s irq_handler!\n", (key_dev)->key_pdev->name);

    spin_lock(&(key_dev->lock));

    value = gpio_get_value(key_dev->gpio);
    if (key_dev->active == GPIO_ACTIVE_LOW) {
        value = !value;
    }
    atomic_set(&(key_dev->key_value_temp), value);
    printk("gpio %d key_value_temp %d!\n",(key_dev)->gpio, value);

    mod_timer(&(key_dev->timer), jiffies + msecs_to_jiffies(15));//延迟15ms

    spin_unlock(&(key_dev->lock));
    return IRQ_RETVAL(IRQ_HANDLED);
}

module_init(key_drv_init);
module_exit(key_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");
MODULE_INFO(intree, "Y");