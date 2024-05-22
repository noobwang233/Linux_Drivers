#include <linux/kernel.h>
#include <linux/types.h>    // 定义了ssize_t的头文件
#include <linux/ide.h>
#include <linux/init.h>     // 模块加载init和卸载exit相关头文件
#include <linux/module.h>
#include <linux/device.h>   //设备号dev_t相关头文件
#include <linux/errno.h>    //错误相关头文件
#include <linux/slab.h>     //kzalloc头文件
#include <linux/cdev.h>     // cdev相关头文件
#include <linux/i2c.h>      //i2c子系统相关头文件
#include <linux/miscdevice.h> //使用miscdev
#include <linux/delay.h>      //usleep_range 头文件
#include "linux/err.h"
#include "linux/printk.h"
#include "at24_drv.h" 


#define IOC_AT24C02_READ  100
#define IOC_AT24C02_WRITE 101

/* macro definition*/
#define AT24C02_SIZE 			256
#define AT24C02_PAGE_SIZE    	16
#define AT24C02_WRITE_TIMEOUT 	25
#define ERR_DBUG(fmt, ...)  	printk(KERN_ERR "%s, LINE %d :  " pr_fmt(fmt), __func__, __LINE__, ##__VA_ARGS__)
#define loop_until_timeout(tout, op_time)				\
	for (tout = jiffies + msecs_to_jiffies(AT24C02_WRITE_TIMEOUT), op_time = 0; \
	     op_time ? time_before(op_time, tout) : true;		\
	     usleep_range(1000, 1500), op_time = jiffies) // 休眠时间在 1000 到 1500 微秒之间的随机时长

/* 变量声明 */
/* private function declear */
// probe and remove
static int at24_drv_probe(struct i2c_client *client, const struct i2c_device_id *dt);
static int at24_drv_remove(struct i2c_client *client);
//file_operations
static long at24_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int at24_release(struct inode *inode, struct file *filp);
static int at24_open(struct inode *inode, struct file *filp);
//module init and exit
static int __init at24_drv_init(void);
static void __exit at24_drv_exit(void);
// 使用i2c_transfer 读取
static ssize_t at24_read_i2c(struct at24_dev_data_t *at24, char *buf, unsigned int offset, size_t count);
static ssize_t at24_write_i2c(struct at24_dev_data_t *at24, const char *buf,unsigned int offset, size_t count);
/* private date definition */
static struct at24_dev_data_t at24_dev = {
	.name = "at24c02",
};
static struct class *at24_cls;//设备类

//必须有这个，不然装载驱动不成功
static const struct i2c_device_id at24c02_ids[] = {
	{ "wt,at24c02",},
	{ /* END OF LIST */ }
};
static const struct of_device_id at24_dev_match_table[] = 
{
    {.compatible = "wt,at24c02",}, //.data 可以放具体设备是参数吗
    {}
};
// i2c_driver结构体
static struct i2c_driver at24_i2c_drv = 
{
    .probe = at24_drv_probe,
    .remove = at24_drv_remove,
    .driver = {
        .name = "at24_drv",
        .owner = THIS_MODULE,
		.of_match_table = at24_dev_match_table,
    },
	.id_table = at24c02_ids,
};

struct at24_buf{
    int addr;
    int len;
    char *data;
};

static struct file_operations at24_fop = 
{
	.owner = THIS_MODULE,
	.release = at24_release,
	.open = at24_open,
	.unlocked_ioctl = at24_ioctl,
};

static ssize_t at24_read_i2c(struct at24_dev_data_t *at24, char *buf,
				    unsigned int offset, size_t count)
{
	unsigned long timeout, read_time;
	struct i2c_client *client;
	struct i2c_msg msg[2];
	int status;
	u8 msgbuf[1];

    // clear msg
	memset(msg, 0, sizeof(msg));
	client = at24->client;

	if (count > AT24C02_PAGE_SIZE)
		count = AT24C02_PAGE_SIZE;

	msgbuf[0] = offset;

	msg[0].addr = client->addr;
	msg[0].buf = msgbuf;
	msg[0].len = 1;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = count;

	loop_until_timeout(timeout, read_time) {
		status = i2c_transfer(client->adapter, msg, 2);
		if (status == 2)
			status = count;

		dev_dbg(&client->dev, "read %zu@%d --> %d (%ld)\n",
				count, offset, status, jiffies);

		if (status == count)
			return count;
	}

	return -ETIMEDOUT;
}

static ssize_t at24_write_i2c(struct at24_dev_data_t *at24, const char *buf,
				    unsigned int offset, size_t count)
{
	unsigned long timeout, write_time;
	struct i2c_client *client;
	struct i2c_msg msg;
	ssize_t status = 0;

	client = at24->client;

	ERR_DBUG("client->addr : %d!\n", client->addr);
	msg.addr = client->addr;
	msg.flags = 0;

	/* msg.buf is u8 and casts will mask the values */
	msg.buf = at24->writebuf;
	msg.buf[0] = offset;
	memcpy(&msg.buf[1], buf, count);
	msg.len = 1 + count;
	ERR_DBUG("&msg.buf[1]= %s, len = %d\r\n", &msg.buf[1], msg.len);
	ERR_DBUG("msg.buf= %s, len = %d\r\n", msg.buf, msg.len);
	loop_until_timeout(timeout, write_time) {
		status = i2c_transfer(client->adapter, &msg, 1);
		if (status == 1)
			status = count;

		dev_dbg(&client->dev, "write %zu@%d --> %zd (%ld)\n",
				count, offset, status, jiffies);

		if (status == count)
			return count;
	}

	return -ETIMEDOUT;
}

static int at24_open(struct inode *inode, struct file *filp)
{
	printk("open at24_dev:%s major=%d,minor=%d\r\n", at24_dev.client->name, MAJOR(at24_dev.dt), MINOR(at24_dev.dt));
	filp->private_data = &at24_dev;

	return 0;

}
static int at24_release(struct inode *inode, struct file *filp)
{
	printk("close at24_dev:%s major=%d,minor=%d\r\n", at24_dev.client->name, MAJOR(at24_dev.dt), MINOR(at24_dev.dt));
	return 0;
}

static long at24_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retvalue;
	uint8_t unalign_len_f;
	uint8_t unalign_len_b;
	uint8_t align_pages;
	uint8_t counts = 0;
	int addr;
	char *data;
	uint8_t i;
	struct at24_buf *user_buf = (struct at24_buf *)arg;
	struct at24_buf ker_buf;
	struct at24_dev_data_t *at24 = (struct at24_dev_data_t *)filp->private_data;

	retvalue = copy_from_user(&ker_buf, user_buf, (sizeof(struct at24_buf)));
	if(retvalue < 0)
	{
		ERR_DBUG("copy_from_user failed!\n");
		return retvalue;
	}

	switch (cmd) {
		case IOC_AT24C02_READ:
		{
			ker_buf.data = kzalloc(ker_buf.len, GFP_KERNEL);
			retvalue = at24_read_i2c(at24, ker_buf.data, ker_buf.addr, ker_buf.len);
			if (retvalue != ker_buf.len) {
				ERR_DBUG("read at24 address %d error, read count %d", ker_buf.addr, retvalue);
			}
			retvalue = copy_to_user(user_buf->data, ker_buf.data, ker_buf.len);
			if(retvalue < 0)
			{
				ERR_DBUG("copy_to_user failed!\n");
				return retvalue;
			}
			kfree(ker_buf.data);
			break;
		}
		case IOC_AT24C02_WRITE:
		{
			if(ker_buf.len > AT24C02_SIZE)
				ker_buf.len = AT24C02_SIZE;
			retvalue = copy_from_user(ker_buf.data, user_buf->data, ker_buf.len);
			if(retvalue < 0)
			{
				ERR_DBUG("copy_from_user failed!\n");
				return retvalue;
			}

			if (ker_buf.addr % AT24C02_PAGE_SIZE !=0)
			{
				unalign_len_f = AT24C02_PAGE_SIZE - (ker_buf.addr % AT24C02_PAGE_SIZE);
				if(unalign_len_f > ker_buf.len)
				{
					unalign_len_f = ker_buf.len;
				}
			}
			else 
			{
				unalign_len_f = 0;
			}
			align_pages = (ker_buf.len - unalign_len_f) / AT24C02_PAGE_SIZE;
			unalign_len_b = (ker_buf.len - unalign_len_f) % AT24C02_PAGE_SIZE;

			addr = 	ker_buf.addr;
			data = ker_buf.data;

			if (unalign_len_f != 0){
				ERR_DBUG("data = %s \r\n", data);
				retvalue = at24_write_i2c(at24, data, addr, unalign_len_f);
				if (retvalue != unalign_len_f) {
					ERR_DBUG("write at24 address %d error, read count %d", addr, retvalue);
					return retvalue;
				}
				counts +=  unalign_len_f;
				if (counts < ker_buf.len) {
					addr = addr + unalign_len_f;
					data = data + unalign_len_f;	
				}
			}

			if (align_pages != 0)
			{
				for(i = 0; i < align_pages; i++)
				{
					ERR_DBUG("data = %s \r\n", data);
					retvalue = at24_write_i2c(at24, data, addr, AT24C02_PAGE_SIZE);
					if (retvalue != AT24C02_PAGE_SIZE) {
						ERR_DBUG("write at24 address %d error, read count %d", addr, retvalue);
						return retvalue;
					}
					counts +=  AT24C02_PAGE_SIZE;
					if (counts < ker_buf.len) {
						addr = addr + AT24C02_PAGE_SIZE;
						data = data + AT24C02_PAGE_SIZE;	
					}
					ERR_DBUG("data = %s \r\n", data);
				}
			}

			if (unalign_len_b != 0){
				ERR_DBUG("data = %s \r\n", data);
				retvalue = at24_write_i2c(at24, data, addr, unalign_len_b);
				if (retvalue != unalign_len_b) {
					ERR_DBUG("write at24 address %d error, read count %d", addr, retvalue);
					return retvalue;
				}
				counts +=  unalign_len_b;
				if (counts < ker_buf.len) {
					addr = addr + unalign_len_b;
					data = data + unalign_len_b;	
				}
			}
			break;
		}
		default:
			// assert
			break;
	}
	return 0;
}


/* function definition */
static int at24_drv_probe(struct i2c_client *client, const struct i2c_device_id *dt)
{
    int retvalue;

    // check the functionality if it supports i2c
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ERR_DBUG("i2c adapter doesn't support i2c func!\n");
        return -EPFNOSUPPORT;
    }
	
	at24_dev.writebuf = kzalloc(sizeof(char) * (AT24C02_PAGE_SIZE + 1), GFP_KERNEL);
	if(IS_ERR_OR_NULL(at24_dev.writebuf))
	{
		ERR_DBUG("kzalloc error!\n");
		return -ENOMEM;
	}

    /* 使用cdev注册字符设备 */
    at24_dev.at24_cdev = cdev_alloc();//申请cdev字符设备的空间
    if(at24_dev.at24_cdev == NULL)
    {
        pr_err("at24_cdev cdev_alloc failed! \n");
        retvalue = -ENOMEM;
		goto freebuffer;
    }
    printk("at24_cdev cdev_alloc successfully!\n");

    /*生成设备号*/
    #ifdef AT24_MAJOR
        at24_dev.dt = MKDEV(AT24_MAJOR, 0);
        retvalue = register_chrdev_region(at24_dev.dt, 1, at24_dev.name);
        if(retvalue != 0)
        {
            pr_err("at24_dev dev_t register failed! start alloc register!\n");
            retvalue = alloc_chrdev_region(&(at24_dev.dt), 0, 1, at24_dev.name);
            if(retvalue != 0)
            {
                pr_err("at24_dev dev_t register failed! \n");
                goto freecdev;
            }
        }
    #else
        retvalue = alloc_chrdev_region(&(at24_dev.dt), 0, 1, at24_dev.name);
        if(retvalue != 0)
        {
            pr_err("at24_dev dev_t register failed! \n");
            goto freecdev;
        }
    #endif
    printk("at24 register dev_t success! major=%d,minor=%d\r\n", MAJOR(at24_dev.dt), MINOR(at24_dev.dt));
    /*注册字符设备*/
    at24_dev.at24_cdev->owner = THIS_MODULE;
    cdev_init(at24_dev.at24_cdev, &at24_fop);
    printk("cdev_init success!\n");
    retvalue = cdev_add(at24_dev.at24_cdev, at24_dev.dt, 1);
    if(retvalue != 0) 
    {
        pr_err("cannot register cdev driver\n");
        goto freedevt;
    }
    printk("cdev_add success!\n");
    /*生成设备节点*/
    at24_dev.dev = device_create(at24_cls, NULL,  at24_dev.dt,  NULL,  "at24_dev");
    if(at24_dev.dev == NULL)
    {
        pr_err("device_create failed!\n");
		retvalue = -ENOMEM;
        goto freedevt;
    }
    printk("at24_dev create success!\r\n");

	at24_dev.client = client;
	memcpy(client->name, (const char *)at24_dev.name, strlen(at24_dev.name));
	printk("at24_drv_probe successfully!\n");
    return 0;

freedevt:
	unregister_chrdev_region(at24_dev.dt, 1); /* 注销设备号 */
	ERR_DBUG("free_misc successfully\n");
freecdev:
	if(at24_dev.at24_cdev != NULL)
		cdev_del(at24_dev.at24_cdev);
	ERR_DBUG("free cdev successfully\n");
freebuffer:
	if(at24_dev.writebuf != NULL)
		kfree(at24_dev.writebuf);
	ERR_DBUG("freebuffer successfully\n");
	return retvalue;
}

static int at24_drv_remove(struct i2c_client *client)
{
	device_destroy(at24_cls, at24_dev.dt);
	printk("device_destroy success!\n");
	if(at24_dev.at24_cdev != NULL)
	{
		cdev_del(at24_dev.at24_cdev);
		printk("cdev_del success!\n");
	}
	unregister_chrdev_region(at24_dev.dt, 1);
	printk("unregister_chrdev_region success!\n");
	if(at24_dev.writebuf != NULL)
		kfree(at24_dev.writebuf);
	return 0;
}

static int __init at24_drv_init(void)
{
	int retvalue;
    at24_cls = class_create(THIS_MODULE, "led_drv");
    if(at24_cls == NULL)
    {
        pr_err("create at24 class failed! \n");
        return -EIO;
    }
    printk("class_create success!\n");
    //注册i2c_driver
	retvalue = i2c_add_driver(&at24_i2c_drv);
	if(!retvalue)
		ERR_DBUG("successfully\n");
	else
	 	ERR_DBUG("failed\n");
    return retvalue;
}

static void __exit at24_drv_exit(void)
{
    //注销i2c_driver
    i2c_del_driver(&at24_i2c_drv);
	ERR_DBUG("successfully\n");
    class_destroy(at24_cls);
    printk("class_destroy success!\n");
}

module_init(at24_drv_init);
module_exit(at24_drv_exit);



MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");
MODULE_INFO(intree, "Y");
