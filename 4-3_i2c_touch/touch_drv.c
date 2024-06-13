#include "linux/delay.h"
#include "linux/err.h"
#include "linux/input/mt.h"
#include "linux/interrupt.h"
#include "linux/of_gpio.h"
#include "linux/printk.h"
#include <linux/types.h>   // 定义了ssize_t的头文件
#include <linux/i2c.h>      //i2c子系统相关头文件
#include <linux/module.h>
#include <linux/init.h>     // 模块加载init和卸载exit相关头文件
#include <linux/input.h>

/***************************************** 宏定义 ******************************************/
#define MAX_SUPPORT_POINTS 5 /* 5点触摸 */

/* TOUCH IC寄存器相关宏定义 */
#define GT_CTRL_REG                 0X8040  /* GT9147控制寄存器         */
#define GT_MODSW_REG                0X804D  /* GT9147模式切换寄存器        */
#define GT_CFGS_REG                 0X8047  /* GT9147配置起始地址寄存器    */
#define GT1151_CFGS_REG             0X8050  /* GT1151配置起始地址寄存器    */
#define GT_CHECK_REG                0X80FF  /* GT9147校验和寄存器       */
#define GT_PID_REG                  0X8140  /* GT9147产品ID寄存器       */

#define GT_GSTID_REG                0X814E  /* GT9147当前检测到的触摸情况 */
#define GT_TP1_REG                  0X8150  /* 第一个触摸点数据地址 */
#define GT_TP2_REG                  0X8158  /* 第二个触摸点数据地址 */
#define GT_TP3_REG                  0X8160  /* 第三个触摸点数据地址 */
#define GT_TP4_REG                  0X8168  /* 第四个触摸点数据地址  */
#define GT_TP5_REG                  0X8170  /* 第五个触摸点数据地址   */

#define GT9147_XYCOORDREG_NUM    30        /* 触摸点坐标寄存器数量 */

#define ERR_DBUG(dev, fmt, ...)      dev_err(dev, "%s, LINE %d :  " pr_fmt(fmt), __func__, __LINE__, ##__VA_ARGS__)

/***************************************** touch设备结构体 ******************************************/
struct i2c_touch_dev{
    int irq_pin;
    int reset_pin;
    int irqnum;
    struct input_dev *inputdev;
    struct i2c_client *client;
};
static struct i2c_touch_dev touch_dev;
const unsigned char GT9147_CT[]=
{
	0x48,0xe0,0x01,0x10,0x01,0x05,0x0d,0x00,0x01,0x08,
	0x28,0x05,0x50,0x32,0x03,0x05,0x00,0x00,0xff,0xff,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x89,0x28,0x0a,
	0x17,0x15,0x31,0x0d,0x00,0x00,0x02,0x9b,0x03,0x25,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,
	0x00,0x0f,0x94,0x94,0xc5,0x02,0x07,0x00,0x00,0x04,
	0x8d,0x13,0x00,0x5c,0x1e,0x00,0x3c,0x30,0x00,0x29,
	0x4c,0x00,0x1e,0x78,0x00,0x1e,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x08,0x0a,0x0c,0x0e,0x10,0x12,0x14,0x16,
	0x18,0x1a,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0x00,0x02,0x04,0x05,0x06,0x08,0x0a,0x0c,
	0x0e,0x1d,0x1e,0x1f,0x20,0x22,0x24,0x28,0x29,0xff,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,
};
/***************************************** 函数申明 ******************************************/

// i2c驱动 probe and remove
static int i2c_touch_drv_probe(struct i2c_client *client, const struct i2c_device_id *dt);
static int i2c_touch_drv_remove(struct i2c_client *client);

// 驱动模块 init and exit
static int __init i2c_touch_drv_init(void);
static void __exit i2c_touch_drv_exit(void);

// 读取和写入touch ic的寄存器
static u8 touch_write_reg(struct i2c_touch_dev *dev, u16 reg, u8 *data, u8 len);
static int touch_read_reg(struct i2c_touch_dev *dev, u16 reg, u8 *data, int len);

// 复位touch ic
static int rest_touch_dev(struct i2c_touch_dev *dev);
void touch_send_cfg(struct i2c_touch_dev *dev, unsigned char mode);


/***************************************** 变量定义 ******************************************/
// 必须有这个，不然装载驱动不成功
static const struct i2c_device_id touch_ids[] = {
    { "touch",},
    { /* END OF LIST */ }
};
// 设备树匹配
static const struct of_device_id touch_dev_match_table[] = 
{
    {.compatible = "wt,touch",},
    {}
};

// i2c_driver 结构体
static struct i2c_driver i2c_touch_drv = 
{
    .probe = i2c_touch_drv_probe,
    .remove = i2c_touch_drv_remove,
    .driver = {
        .name = "i2c_touch_drv",
        .owner = THIS_MODULE,
        .of_match_table = touch_dev_match_table,
    },
    .id_table = touch_ids,
};


/***************************************** 函数定义 ******************************************/
/*
 * @description	: 发送GT9147配置参数
 * @param - client: i2c_client
 * @param - mode: 0,参数不保存到flash
 *                1,参数保存到flash
 * @return 		: 无
 */
void touch_send_cfg(struct i2c_touch_dev *dev, unsigned char mode)
{
    unsigned char buf[2];
    unsigned int i = 0;

    buf[0] = 0;
    buf[1] = mode;                              /* 是否写入到GT9147 FLASH?  即是否掉电保存 */
    for(i = 0; i < (sizeof(GT9147_CT)); i++)    /* 计算校验和 */
        buf[0] += GT9147_CT[i];
    buf[0] = (~buf[0]) + 1;

    /* 发送寄存器配置 */
    touch_write_reg(dev, GT_CFGS_REG, (u8 *)GT9147_CT, sizeof(GT9147_CT));
    touch_write_reg(dev, GT_CHECK_REG, buf, 2);/* 写入校验和,配置更新标记 */
}


static void init_touch_dev(struct i2c_touch_dev *dev)
{
    u8 data;

    /* 1. 初始化GT9147 */
    data = 0x02;
    touch_write_reg(dev, GT_CTRL_REG, &data, 1); /* 软复位 */
    mdelay(100);
    data = 0x0;
    touch_read_reg(dev, GT_CTRL_REG, &data, 1); /* 停止软复位 */
    mdelay(100);

    /* 2. 初始化GT9147，烧写固件 */
    touch_read_reg(dev, GT_CFGS_REG, &data, 1);
    printk("GT9147 ID =%#X\r\n", data);
    if(data <  GT9147_CT[0]) {
        touch_send_cfg(dev, 0);
    }
}

static int rest_touch_dev(struct i2c_touch_dev *dev)
{
    int retvalue = 0;                       // 返回值
    struct i2c_client *client = dev->client;

    /* 1. 申请reset的gpio */
    if (gpio_is_valid(dev->reset_pin)) {
        retvalue = devm_gpio_request_one(&client->dev, dev->reset_pin, GPIOF_OUT_INIT_HIGH, "touch_irq");
        if (retvalue != 0)
        {
            ERR_DBUG(&client->dev, "Failed to request GPIO %d, error %d\n",dev->reset_pin, retvalue);
            goto fail;
        }
        printk("request gpio %d successfully! \n", dev->reset_pin);
    }

    /* 2. 申请中断的gpio */
    if (gpio_is_valid(dev->irq_pin)) {
        retvalue = devm_gpio_request_one(&client->dev, dev->irq_pin, GPIOF_OUT_INIT_HIGH, "touch_irq");
        if (retvalue != 0)
        {
            ERR_DBUG(&client->dev, "Failed to request GPIO %d, error %d\n",dev->irq_pin, retvalue);
            goto fail;
        }
        printk("request gpio %d successfully! \n", dev->irq_pin);
    }

    /* 3. 复位 */
    gpio_set_value(dev->reset_pin, 0);        /* 输出低电平，复位 */
    msleep(10);
    gpio_set_value(dev->reset_pin, 1);        /* 输出高电平，停止复位 */
    msleep(10);

    /* 4. 设置中断引脚状态 */
    gpio_set_value(dev->irq_pin, 0);          /* 拉低INT引脚 */
    msleep(50);
    gpio_direction_input(dev->irq_pin);             /* INT引脚设置为输入 */

    return 0;

fail:
    return retvalue;
}

static void goodix_read_config(struct goodix_ts_data *ts)
{
	u8 config[GOODIX_CONFIG_MAX_LENGTH];
	int error;

	error = goodix_i2c_read(ts->client, GOODIX_REG_CONFIG_DATA,
			      config,
			   GOODIX_CONFIG_MAX_LENGTH);
	if (error) {
		dev_warn(&ts->client->dev,
			 "Error reading config (%d), using defaults\n",
			 error);
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		ts->int_trigger_type = GOODIX_INT_TRIGGER;
		ts->max_touch_num = GOODIX_MAX_CONTACTS;
		return;
	}

	ts->abs_x_max = get_unaligned_le16(&config[RESOLUTION_LOC]);
	ts->abs_y_max = get_unaligned_le16(&config[RESOLUTION_LOC + 2]);
	ts->int_trigger_type = config[TRIGGER_LOC] & 0x03;
	ts->max_touch_num = config[MAX_CONTACTS_LOC] & 0x0f;
	if (!ts->abs_x_max || !ts->abs_y_max || !ts->max_touch_num) {
		dev_err(&ts->client->dev,
			"Invalid config, using defaults\n");
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		ts->max_touch_num = GOODIX_MAX_CONTACTS;
	}
}


static u8 touch_write_reg(struct i2c_touch_dev *dev, u16 reg, u8 *data, u8 len)
{
    u8 buffer[256];
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client *)dev->client;

    buffer[0] = reg >> 8;               /* 寄存器首地址低8位 */
    buffer[1] = reg & 0XFF;             /* 寄存器首地址高8位 */

    memcpy(&buffer[2], data, len);      /* 将要写入的数据拷贝到数组b里面 */

    msg.addr = client->addr;
    msg.flags = !I2C_M_RD;              /* 标记为发送数据 */

    msg.buf = buffer;                   /* 要写入的数据缓冲区 */
    msg.len = len + 2;                  /* 要写入的数据长度 */

    return i2c_transfer(client->adapter, &msg, 1);
}

static int touch_read_reg(struct i2c_touch_dev *dev, u16 reg, u8 *data, int len)
{
    int ret;
    u8 regdata[2];
    struct i2c_msg msg[2];
    struct i2c_client *client = (struct i2c_client *)dev->client;

   /* GT9147寄存器长度为2个字节 */
    regdata[0] = reg >> 8;
    regdata[1] = reg & 0xFF;

    /* msg[0]为发送要读取的首地址 */
    msg[0].addr = client->addr;
    msg[0].flags = !I2C_M_RD;               /* 标记为发送数据 */
    msg[0].buf = &regdata[0];               /* 读取的首地址 */
    msg[0].len = 2;                         /* reg长度*/

    /* msg[1]读取数据 */
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;                /* 标记为读取数据*/
    msg[1].buf = data;                      /* 读取数据缓冲区 */
    msg[1].len = len;                       /* 要读取的数据长度*/

    ret = i2c_transfer(client->adapter, msg, 2);
    if(ret == 2) {
        ret = 0;
    } else {
        ret = -EREMOTEIO;
    }
    return ret;

}

static irqreturn_t touch_irq_handler(int irq, void *dev_id)
{
    int touch_num = 0;
    int input_x, input_y;
    int id = 0;
    int ret = 0;
    u8 data;
    u8 touch_data[5];
    struct i2c_touch_dev *dev = (struct i2c_touch_dev *)dev_id;

    ret = touch_read_reg(dev, GT_GSTID_REG, &data, 1);
    if (data == 0x00)  {     /* 没有触摸数据，直接返回 */
        goto fail;
    } else {                 /* 统计触摸点数据 */
        touch_num = data & 0x0f;
    }

    /* 由于GT9147没有硬件检测每个触摸点按下和抬起，因此每个触摸点的抬起和按
     * 下不好处理，尝试过一些方法，但是效果都不好，因此这里暂时使用单点触摸 
     */
    if(touch_num) {         /* 单点触摸按下 */
        touch_read_reg(dev, GT_TP1_REG, touch_data, 5);
        id = touch_data[0] & 0x0F;
        if(id == 0) {
            input_x  = touch_data[1] | (touch_data[2] << 8);
            input_y  = touch_data[3] | (touch_data[4] << 8);

            input_mt_slot(dev->inputdev, id);
            input_mt_report_slot_state(dev->inputdev, MT_TOOL_FINGER, true);
            input_report_abs(dev->inputdev, ABS_MT_POSITION_X, input_x);
            input_report_abs(dev->inputdev, ABS_MT_POSITION_Y, input_y);
        }
    } else if(touch_num == 0){                /* 单点触摸释放 */
        input_mt_slot(dev->inputdev, id);
        input_mt_report_slot_state(dev->inputdev, MT_TOOL_FINGER, false);
    }

    input_mt_report_pointer_emulation(dev->inputdev, true);
    input_sync(dev->inputdev);

    data = 0x00;                /* 向0X814E寄存器写0 */
    touch_write_reg(dev, GT_GSTID_REG, &data, 1);

fail:
	return IRQ_HANDLED;
}


static int i2c_touch_drv_probe(struct i2c_client *client, const struct i2c_device_id *dt)
{
    int retvalue = 0;                       // 返回值
    struct i2c_touch_dev *dev = &touch_dev; // 设备结构体变量

    /* 1. 设置i2c client */
    dev->client = client;
	i2c_set_clientdata(client, dev);

    /* 2.1 获取中断的gpio号 */
    dev->irq_pin = of_get_named_gpio(client->dev.of_node, "interrupt-gpios", 0);
    if (dev->irq_pin < 0)
    {
        dev_err(&client->dev, "get interrupt gpio failed, error %d\n", dev->reset_pin);
        retvalue = dev->irq_pin;
        goto fail;
    }
    printk("get interrupt gpio %d successfully! \n", dev->irq_pin);

    /* 2.2 获取reset的gpio号 */
    dev->reset_pin = of_get_named_gpio(client->dev.of_node, "reset-gpios", 0);
    if (dev->reset_pin < 0)
    {
        ERR_DBUG(&client->dev, "get reset gpio failed, error %d\n", dev->reset_pin);
        retvalue = dev->reset_pin;
        goto fail;
    }
    printk("get reset gpio %d successfully! \n", dev->reset_pin);

    /* 3. 复位touch IC */
    retvalue = rest_touch_dev(dev);
    if (retvalue != 0)
    {
        ERR_DBUG(&client->dev, "reset touch ic failed, error %d\n", retvalue);
        goto fail;
    }
    printk("reset touch ic successfully! \n");

    /* 4. 初始化touch ic */
    init_touch_dev(dev);

    /* 5. 注册input设备 */
    dev->inputdev = devm_input_allocate_device(&client->dev);
    if(IS_ERR(dev->inputdev))
    {
        ERR_DBUG(&client->dev, "allocate input dev error");
        return PTR_ERR(dev->inputdev);
    }
    dev->inputdev->name = client->name;
    dev->inputdev->id.bustype = BUS_I2C;
    dev->inputdev->dev.parent = &client->dev;

    __set_bit(EV_KEY, dev->inputdev->evbit);
    __set_bit(EV_ABS, dev->inputdev->evbit);
    __set_bit(BTN_TOUCH, dev->inputdev->keybit);

    input_set_abs_params(dev->inputdev, ABS_X, 0, 800, 0, 0);
    input_set_abs_params(dev->inputdev, ABS_Y, 0, 480, 0, 0);
    input_set_abs_params(dev->inputdev, ABS_MT_POSITION_X,0, 800, 0, 0);
    input_set_abs_params(dev->inputdev, ABS_MT_POSITION_Y,0, 480, 0, 0);	     
    retvalue = input_mt_init_slots(dev->inputdev, MAX_SUPPORT_POINTS, 0);
    if (retvalue) {
        goto fail;
    }
    retvalue = input_register_device(dev->inputdev);
    if (retvalue)
        goto fail;
    printk("register input dev successfully! \n");

    /* 6. 申请中断函数 */
    retvalue = devm_request_threaded_irq(&client->dev, client->irq, NULL,touch_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "touch_irq_handler",dev);
    if (retvalue != 0)
    {
        ERR_DBUG(&client->dev, "request touch irq failed, error %d\n", retvalue);
        goto fail;
    }
    printk("request touch irq successfully! \n");

    return 0;

fail:
    return retvalue;
}
static int i2c_touch_drv_remove(struct i2c_client *client)
{
    int retvalue = 0;

    input_unregister_device(touch_dev.inputdev);

    return retvalue;
}


static int __init i2c_touch_drv_init(void)
{
    int retvalue;
    //注册i2c_driver
    retvalue = i2c_add_driver(&i2c_touch_drv);
    if(!retvalue)
        printk("init successfully\n");
    else
        printk("init failed\n");
    return retvalue;
}
module_init(i2c_touch_drv_init);

static void __exit i2c_touch_drv_exit(void)
{
    //注销i2c_driver
    i2c_del_driver(&i2c_touch_drv);
    printk("exit successfully\n");
}
module_exit(i2c_touch_drv_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");
MODULE_INFO(intree, "Y");
