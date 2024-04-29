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
#include <linux/miscdevice.h> //miscdevice头文件
#include <linux/platform_device.h> //platform_device头文件
#include <linux/of_platform.h> //platform of函数
#include <asm/string.h>
#include <linux/slab.h> //kzalloc头文件
#include <linux/string.h>
#include <linux/interrupt.h> //中断相关头文件
#include <linux/timer.h>

#define KEY_MAJOR 234
#define DEV_COUNT 2
#define KEY_PUSH 1
#define KEY_RELEASED 0
/*private date*/
static u8 key_dev_count = 0; /* 设备计数 */
static struct class *key_cls;//设备类
/* device struct 设备结构体*/
struct key_dev_t
{
    dev_t dt;
    struct platform_device *key_pdev;
    int gpio; /* key 所使用的 GPIO 编号 */
    int irq; //中断号
    spinlock_t lock; /*设备互斥访问自旋锁*/
    bool dev_status; /*设备状态*/
    atomic_t key_value; /*按键状态*/
    atomic_t key_value_temp; /*按键暂时状态*/
    struct cdev *key_cdev; /*字符设备结构体*/
    struct class *cls;
    struct device *dev;
    struct timer_list timer; //用于消抖的计时器
    wait_queue_head_t wait_list; //等待队列
};
// match table
const struct of_device_id keys_of_match_table[] = {
    {.compatible = "key_gpio",},
    {},
};
struct key_dev_t *key_devs[DEV_COUNT] = {0};


/* private function declear */
// file_operation functions
static int key_drv_open(struct inode *inode, struct file *filp);
static int key_drv_release(struct inode *inode, struct file *filp);
static ssize_t key_drv_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt);
static ssize_t key_drv_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt);
static int key_drv_probe(struct platform_device *device);
static int key_drv_remove(struct platform_device *device);
//module init and exit
static int __init key_drv_init(void);
static void __exit key_drv_exit(void);
//private key init and deinit function
static int key_dev_init(struct key_dev_t **key_devs, u32 index);
static int key_drv_deinit(struct key_dev_t **key_devs, u32 index);
//irq handle function
irqreturn_t key_irq_handler(int irq, void *dev);

/* 驱动提供的文件操作结构体 */
static struct file_operations key_drv_fop = {
    .owner = THIS_MODULE,
    .open = key_drv_open,
    .release = key_drv_release,
    .write = key_drv_write,
    .read = key_drv_read,
};
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
    if(value == atomic_read(&key_dev->key_value_temp))
    {
        //操作有效
        printk("key status changed value %d\n", value);
        atomic_set(&key_dev->key_value, value);
        wake_up_interruptible(&((struct key_dev_t *)key_dev)->wait_list);
    }
    else
    {
        printk("invalid %d\n", value);
        //操作无效不更改键值
    }
    spin_unlock_irqrestore(&key_dev->lock, flags);
}
static int key_drv_open(struct inode *inode, struct file *filp)
{
    unsigned long flags;/*中断标记*/
    u32 index;

    for(index = 0; index < DEV_COUNT; index++)
    {
        if(key_devs[index] != NULL)
        {
            if(key_devs[index]->dt == inode->i_cdev->dev)
                break;
        }
    }
    if(index >= DEV_COUNT)
    {
        pr_err("can not find dev data in dev list!\n");
        return -EIO;
    }
    filp->private_data = key_devs[index];
    //struct key_dev_t *key_dev = container_of(inode->i_cdev, struct key_dev_t, key_cdev);//获取当前打开设备文件对应的设备结构体变量指针
    printk("open device file: %s",key_devs[index]->key_pdev->name);

    spin_lock_irqsave(&(key_devs[index]->lock), flags);//上锁
    if(key_devs[index]->dev_status != true) //设备忙
    {
        spin_unlock_irqrestore(&(key_devs[index]->lock), flags);//释放锁
        pr_err("key_drv busy!\n");
        return -EBUSY;
    }
    key_devs[index]->dev_status = false;//占用设备
    spin_unlock_irqrestore(&(key_devs[index]->lock), flags);//释放锁
    printk("key_drv open!\r\n");
    return 0;
}

static int key_drv_release(struct inode *inode, struct file *filp)
{
    unsigned long flags;/*中断标记*/
    
    struct key_dev_t *key_dev = filp->private_data;
    spin_lock_irqsave(&(key_dev->lock), flags);//上锁
    key_dev->dev_status = true;//释放设备
    spin_unlock_irqrestore(&(key_dev->lock), flags);//释放锁
    printk("key_drv release!\r\n");
    return 0;
}

static ssize_t key_drv_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    int retvalue = 0;
    struct key_dev_t *key_dev = filp->private_data;
    static atomic_t last_value = ATOMIC_INIT(KEY_RELEASED);

    /*判断当前文件描述符是阻塞还是非阻塞*/
    if(filp->f_flags & O_NONBLOCK)
    {
        /*非阻塞访问*/
        printk(" noblock read!\r\n");

    }
    else
    {
        /* 阻塞访问 */
        /* 加入等待队列，当event不为0时,即按键状态改变时，才会被唤醒 */
        printk(" block read!\r\n");
        retvalue = wait_event_interruptible(key_dev->wait_list, atomic_read(&key_dev->key_value) != atomic_read(&last_value));
        if(retvalue)
            return retvalue;
    }
    atomic_set(&last_value, atomic_read(&key_dev->key_value));//记录键值
    /* 向用户空间发送数据 */
    retvalue = copy_to_user(buf, &key_dev->key_value, sizeof(key_dev->key_value));
    if(retvalue == 0)
    {
        printk("key_drv send data ok!\r\n");
    }
    else
    {
        printk("key_drv send data failed!\r\n");
        return -EIO;
    }
    printk("read key_drv finish!\r\n");
    return 0;
}

static ssize_t key_drv_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
    //empty
    return 0;
}
/*
key_dev: 按键设备的结构体指针
*/
static int key_dev_init(struct key_dev_t **key_devs, u32 index)
{
    int retvalue;
    struct device_node *np = key_devs[index]->key_pdev->dev.of_node;

    /* 获取gpio号 */
    key_devs[index]->gpio = of_get_named_gpio(np, "key-gpio", 0);
    if (key_devs[index]->gpio <= 0)
    {
        pr_err("get gpio failed\n");
        return -EIO;
    }
    printk("get gpio %d successfully! \n", key_devs[index]->gpio);

    /* 申请gpio */
    retvalue = gpio_request(key_devs[index]->gpio, "key_gpio");
    if (retvalue != 0)
    {
        pr_err("request gpio failed\n");
        return -EIO;
    }
    printk("request gpio %d successfully! \n", key_devs[index]->gpio);
    /* 设置为输入 */
    retvalue = gpio_direction_input(key_devs[index]->gpio);
    if (retvalue != 0)
    {
        pr_err("set gpio %d input failed! \n", key_devs[index]->gpio);
        goto freegpio;
    }
    printk("set gpio %d input successfully! \n", key_devs[index]->gpio);


    
    /* 使用cdev注册字符设备 */
    key_devs[index]->key_cdev = cdev_alloc();//申请cdev字符设备的空间
    if(key_devs[index]->key_cdev == NULL )
    {
        pr_err("key_dev key_cdev kzalloc failed! \n");
        goto freegpio;
    }
    printk("key_dev key_cdev kzalloc successfully!\n");

    /*生成设备号*/
    #ifdef KEY_MAJOR
        key_devs[index]->dt = MKDEV(KEY_MAJOR, index);
        retvalue = register_chrdev_region(key_devs[index]->dt, 1, key_devs[index]->key_pdev->name);
        if(retvalue != 0)
        {
            pr_err("key_dev dev_t register failed! start alloc register!\n");
            retvalue = alloc_chrdev_region(&(key_devs[index]->dt), 0, 1, key_devs[index]->key_pdev->name);
            if(retvalue != 0)
            {
                pr_err("key_dev dev_t register failed! \n");
                goto freecdev;
            }
        }
    #else
        retvalue = alloc_chrdev_region(&(key_devs[index]->dt), 0, 1, key_devs[index]->key_pdev->name);
        if(retvalue != 0)
        {
            pr_err("key_dev dev_t register failed! \n");
            goto freecdev;
        }
    #endif
    printk("key register dev_t success! major=%d,minor=%d\r\n", MAJOR(key_devs[index]->dt), MINOR(key_devs[index]->dt));
    /*注册字符设备*/
    key_devs[index]->key_cdev->owner = THIS_MODULE;
    cdev_init(key_devs[index]->key_cdev, &key_drv_fop);
    printk("cdev_init success!\n");
    retvalue = cdev_add(key_devs[index]->key_cdev, key_devs[index]->dt, 1);
    if(retvalue != 0) 
    {
        pr_err("cannot register cdev driver\n");
        goto freedevt;
    }
    printk("cdev_add success!\n");
    /*生成设备节点*/
    key_devs[index]->dev = device_create(key_devs[index]->cls, NULL,  key_devs[index]->dt,  NULL,  "key_dev_%d", index);
    if(key_devs[index]->dev == NULL)
    {
        pr_err("device_create failed!\n");
        goto delcdev;
    }
    printk("key_dev_%d create success!\r\n", index);
    //irq init
    key_devs[index]->irq = gpio_to_irq(key_devs[index]->gpio);//get irq num
    if(key_devs[index]->irq < 0)
    {
        pr_err("get irq num failed!\n");
        goto destroydev;
    }
    printk("get irq num %d success!\r\n", key_devs[index]->irq);
    retvalue = request_irq(key_devs[index]->irq, key_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, key_devs[index]->key_pdev->name, key_devs[index]);
    if(retvalue != 0) 
    {
        pr_err("request irq failed!\n");
        if(retvalue == -EBUSY) 
        {
            pr_err("irq already requset!\n");
        }
        goto destroydev;
    }
    /* 初始化设备自旋锁*/
    spin_lock_init(&key_devs[index]->lock);
    key_devs[index]->dev_status = true;
    //初始化等待队列
    init_waitqueue_head(&key_devs[index]->wait_list);
    atomic_set(&key_devs[index]->key_value, KEY_RELEASED);
    return 0;

//错误处理
destroydev:
    device_destroy(key_devs[index]->cls, key_devs[index]->dt);
delcdev:
    cdev_del(key_devs[index]->key_cdev);
freedevt:
    unregister_chrdev_region(key_devs[index]->dt, 1);
freecdev:
    if (key_devs[index]->key_cdev != NULL)
    {
        kfree(key_devs[index]->key_cdev);
    }
freegpio:
    gpio_free(key_devs[index]->gpio);
    return -EIO;
}

static int key_drv_deinit(struct key_dev_t **key_devs, u32 index)
{
    free_irq(key_devs[index]->irq,key_devs[index]);
    printk("free_irq success!\n");
    device_destroy(key_devs[index]->cls, key_devs[index]->dt);
    printk("device_destroy success!\n");
    cdev_del(key_devs[index]->key_cdev);
    printk("cdev_del success!\n");
    unregister_chrdev_region(key_devs[index]->dt, 1);
    printk("unregister_chrdev_region success!\n");
    if (key_devs[index]->key_cdev != NULL)
    {
        kfree(key_devs[index]->key_cdev);
    }
    gpio_free(key_devs[index]->gpio);
    printk("gpio_free success!\n");
    if (key_devs[index] != NULL)
    {
        kfree(key_devs[index]);
        printk("kfree(key_devs[index]) success!\n");
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
        key_drv_deinit(key_devs, index);
    }
    //分配设备结构体空间
    key_devs[index] = kzalloc(sizeof(struct key_dev_t), GFP_KERNEL);
    if(key_devs[index] == NULL )
    {
        pr_err("key_devs[index] kzalloc failed! \n");
        return -EIO;
    }
    printk("key_dev kzalloc successfully!\n");
    key_devs[index]->key_pdev = device;
    key_devs[index]->cls = key_cls;
    init_timer(&key_devs[index]->timer); /* 初始化定时器 */
    key_devs[index]->timer.function = key_time_function; /* 设置定时处理函数 */
    key_devs[index]->timer.data = (unsigned long)(key_devs[index]); //设置定时器处理函数的传入参数为设备结构体地址
    retvalue = key_dev_init(key_devs, index);
    if(retvalue != 0)
    {
        goto freekey_dev;
    }
    printk("%s probe() success!\r\n",key_devs[index]->key_pdev->name);
    key_dev_count++;
    return 0;

freekey_dev:
    if (key_devs[index] != NULL)
        kfree(key_devs[index]);
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
    key_drv_deinit(key_devs, index);
    printk("%s remove() success!\r\n",key_devs[index]->key_pdev->name);
    key_dev_count--;
    return 0;
}

static int __init key_drv_init(void)
{
    int retvalue = 0;

    key_cls = class_create(THIS_MODULE, "key_drv");
    if(key_cls == NULL)
    {
        pr_err("create key_drv class failed! \n");
        return -EIO;
    }
    printk("class_create success!\n");
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
    class_destroy(key_cls);
    printk("class_destroy success!\n");
}


irqreturn_t key_irq_handler(int irq, void *dev)
{
    u8 value;

    printk("%s irq_handler!\n", ((struct key_dev_t *)dev)->key_pdev->name);
    spin_lock(&((struct key_dev_t *)dev)->lock);
    value = gpio_get_value(((struct key_dev_t *)dev)->gpio);
    atomic_set(&((struct key_dev_t *)dev)->key_value_temp, value);
    printk("gpio %d key_value_temp %d!\n",((struct key_dev_t *)dev)->gpio, value);
    mod_timer(&((struct key_dev_t *)dev)->timer, jiffies + msecs_to_jiffies(15));//延迟15ms
    spin_unlock(&((struct key_dev_t *)dev)->lock);
    return IRQ_RETVAL(IRQ_HANDLED);
}

module_init(key_drv_init);
module_exit(key_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");
MODULE_INFO(intree, "Y");