/* 
 * Goodix GT9xx touchscreen driver
 * 
 * Copyright  (C)  2010 - 2014 Goodix. Ltd.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Version: 2.4
 * Release Date: 2014/11/28
 */
#include <linux/irq.h>
#include "gt9xx_drv.h"
#define FT_REG_THGROUP          0x80
#define FT_REG_POINT_RATE       0x88

static const char *goodix_ts_name = "goodix-ts";
static const char *goodix_input_phys = "input/ts";
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL; 
int gtp_rst_gpio;
int gtp_int_gpio;
u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_send_cfg(struct i2c_client *client);
void gtp_int_sync(s32 ms);

static int gtp_register_powermanger(struct goodix_ts_data *ts);
static int gtp_unregister_powermanger(struct goodix_ts_data *ts);


int gtp_parse_dt_cfg(struct device *dev, u8 *cfg, int *cfg_len, u8 sid);


/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GTP_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer: 
      2: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.
    
    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if((retries >= 5))
    {
        GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
        {
            gtp_reset_guitar(client, 10);  
        }
    }
    return ret;
}



/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer: 
        1: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
        GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
        {
            gtp_reset_guitar(client, 10);  
        }
    }
    return ret;
}

/*******************************************************
Function:
    i2c read twice, compare the results
Input:
    client:  i2c device
    addr:    operate address
    rxbuf:   read data to store, if compare successful
    len:     bytes to read
Output:
    FAIL:    read failed
    SUCCESS: read successful
*********************************************************/
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;
    
    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, buf, len + 2);
        
        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, confirm_buf, len + 2);
        
        if (!memcmp(buf, confirm_buf, len+2))
        {
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }    
    GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
    return FAIL;
}

/*******************************************************
Function:
    Send config.
Input:
    client: i2c device.
Output:
    result of i2c write operation. 
        1: succeed, otherwise: failed
*********************************************************/

s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 2;

#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    if (ts->pnl_init_error)
    {
        GTP_INFO("Error occured in init_panel, no config sent");
        return 0;
    }
    
    GTP_INFO("Driver send config.");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }
#endif
    return ret;
}
/*******************************************************
Function:
    Disable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
    unsigned long irqflags;

    GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1; 
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
    unsigned long irqflags = 0;

    GTP_DEBUG_FUNC();
    
    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable) 
    {
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0; 
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


/*******************************************************
Function:
    Report touch point event 
Input:
    ts: goodix i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{

#if GTP_CHANGE_X2Y
    GTP_SWAP(x, y);
#endif
  
    input_report_key(ts->input_dev, BTN_TOUCH, 1);
    //input_report_abs(ts->input_dev, ABS_X, ts->abs_x_max - x);
    //input_report_abs(ts->input_dev, ABS_Y, ts->abs_y_max - y);
    input_report_abs(ts->input_dev, ABS_X, x);
    input_report_abs(ts->input_dev, ABS_Y, y);
    input_report_abs(ts->input_dev, ABS_Z, 0);
    input_report_abs(ts->input_dev, ABS_PRESSURE, 1);
    input_mt_sync(ts->input_dev);

    GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{

    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
    input_sync(ts->input_dev);

}

/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;

    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;

    GTP_DEBUG_FUNC();
    ts = container_of(work, struct goodix_ts_data, work);
    if (ts->enter_update)
    {
        return;
    }

    ret = gtp_i2c_read(ts->client, point_data, 12);
    if (ret < 0)
    {
        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }
    
    finger = point_data[GTP_ADDR_LENGTH];

    if (finger == 0x00)
    {
        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }
        return;
    }

    if((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1)); 
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

    pre_key = key_value;

    //GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

    if (touch_num)
    {
        for (i = 0; i < touch_num; i++)
        {
            coor_data = &point_data[i * 8 + 3];

            id = coor_data[0] & 0x0F;
            input_x  = coor_data[1] | (coor_data[2] << 8);
            input_y  = coor_data[3] | (coor_data[4] << 8);
            input_w  = coor_data[5] | (coor_data[6] << 8);
            {
                gtp_touch_down(ts, id, input_x, input_y, input_w);
            }
        }
    }
    else if (pre_touch)
    {
        {
            GTP_DEBUG("Touch Release!");
            gtp_touch_up(ts, 0);
        }
    }

    pre_touch = touch_num;

    {
        input_sync(ts->input_dev);
    }

exit_work_func:
    if(!ts->gtp_rawdiff_mode)
    {
        ret = gtp_i2c_write(ts->client, end_cmd, 3);
        if (ret < 0)
        {
            GTP_INFO("I2C write end_cmd error!");
        }
    }
    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode. 
        HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    GTP_DEBUG_FUNC();

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
        IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
    struct goodix_ts_data *ts = dev_id;

    GTP_DEBUG_FUNC();
 
    gtp_irq_disable(ts);

    queue_work(goodix_wq, &ts->work);
    
    return IRQ_HANDLED;
}
/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millisecond.
Output:
    None.
*******************************************************/
void gtp_int_sync(s32 ms)
{
    GTP_GPIO_OUTPUT(gtp_int_gpio, 0);
    msleep(ms);
    GTP_GPIO_AS_INT(gtp_int_gpio);
}


/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millisecond
Output:
    None.
*******************************************************/
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
    GTP_DEBUG_FUNC();
    GTP_INFO("Guitar reset");
#if GTP_RST_PORT
    GTP_GPIO_OUTPUT(gtp_rst_gpio, 0);   // begin select I2C slave addr
#endif
    msleep(ms);                         // T2: > 10ms
    // HIGH: 0x28/0x29, LOW: 0xBA/0xBB
    GTP_GPIO_OUTPUT(gtp_int_gpio, client->addr == 0x14);
#if GTP_RST_PORT
    msleep(2);                          // T3: > 100us
    GTP_GPIO_OUTPUT(gtp_rst_gpio, 1);
    
    msleep(6);                          // T4: > 5ms

    GTP_GPIO_AS_INPUT(gtp_rst_gpio);    // end select I2C slave addr
#endif

    gtp_int_sync(50);
}

/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

    
    GTP_DEBUG_FUNC();

    GTP_GPIO_OUTPUT(gtp_int_gpio, 0);
    msleep(5);
    
    while(retry++ < 5)
    {
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            GTP_INFO("GTP enter sleep!");
            
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send sleep cmd failed.");
    return ret;
}

/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
    u8 retry = 0;
    s8 ret = -1;
    
    GTP_DEBUG_FUNC();

#if GTP_POWER_CTRL_SLEEP
    while(retry++ < 5)
    {
        gtp_reset_guitar(ts->client, 20);
        
        //GTP_INFO("GTP wakeup sleep.");
        return 1;
    }
#else
    while(retry++ < 10)
    {
        GTP_GPIO_OUTPUT(gtp_int_gpio, 1);
        msleep(5);
    
        ret = gtp_i2c_test(ts->client);
        if (ret > 0)
        {
            //GTP_INFO("GTP wakeup sleep.");
            gtp_int_sync(25);
            return ret;
        }
        gtp_reset_guitar(ts->client, 20);
    }
#endif

    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
        0: succeed, otherwise: failed
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
    s32 ret = -1;

#if GTP_DRIVER_SEND_CFG
    s32 i = 0;
    u8 check_sum = 0;
    u8 opr_buf[16] = {0};
    u8 sensor_id = 0;
    u8 drv_cfg_version;
    u8 flash_cfg_version;

    /* check firmware */
    ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
    if (SUCCESS == ret)
    {
        if (opr_buf[0] != 0xBE)
        {
            ts->fw_error = 1;
            GTP_ERROR("Firmware error, no config sent!");
            return -1;
        }
    }

    /* read sensor id */
    ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
    if (SUCCESS == ret)
    {
        if (sensor_id >= 0x06)
        {
            GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
            ts->pnl_init_error = 1;
            return -1;
        }
    }
    else
    {
        GTP_ERROR("Failed to get sensor_id, No config sent!");
        ts->pnl_init_error = 1;
        return -1;
    }
    GTP_INFO("Sensor_ID: %d", sensor_id);

    /* parse config data*/
    GTP_DEBUG("Get config data from device tree.");
    ret = gtp_parse_dt_cfg(&ts->client->dev, &config[GTP_ADDR_LENGTH], &ts->gtp_cfg_len, sensor_id);
    if (ret < 0) {
        GTP_ERROR("Failed to parse config data form device tree.");
        ts->pnl_init_error = 1;
        return -1;
    }

    printk("Config group%d used,length: %d", sensor_id, ts->gtp_cfg_len);

    if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH)
    {
        GTP_ERROR("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id, ts->gtp_cfg_len);
        ts->pnl_init_error = 1;
        return -1;
    }

    {
        ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
        if (ret == SUCCESS) {
            GTP_DEBUG("Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X",
                        config[GTP_ADDR_LENGTH], config[GTP_ADDR_LENGTH], opr_buf[0], opr_buf[0]);

            flash_cfg_version = opr_buf[0];
            drv_cfg_version = config[GTP_ADDR_LENGTH];
            
            if (flash_cfg_version < 90 && flash_cfg_version > drv_cfg_version) {
                config[GTP_ADDR_LENGTH] = 0x00;
            }
        } else {
            GTP_ERROR("Failed to get ic config version!No config sent!");
            return -1;
        }
    }

#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);
    
    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe; 
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
#endif  // GTP_CUSTOM_CFG
    
    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else // driver not send config

    ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("Read Config Failed, Using Default Resolution & INT Trigger!");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }
    
#endif // GTP_DRIVER_SEND_CFG

    if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0))
    {
        ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03; 
    }

    {
#if GTP_DRIVER_SEND_CFG
        ret = gtp_send_cfg(ts->client);
        if (ret < 0)
        {
            GTP_ERROR("Send config error.");
        }
    {
        if (flash_cfg_version < 90 && flash_cfg_version > drv_cfg_version) {
            check_sum = 0;
            config[GTP_ADDR_LENGTH] = drv_cfg_version;
            for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++) {
                check_sum += config[i];
            }
            config[ts->gtp_cfg_len] = (~check_sum) + 1;
        }
    }

#endif
        GTP_INFO("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
    }

    msleep(10);
    return 0;

}

/*******************************************************
Function:
    Read chip version.
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
        2: succeed, otherwise: failed
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};
    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }
    if (buf[5] == 0x00)
    {
        GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        GTP_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    return ret;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
        2: succeed, otherwise failed.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;
  
    GTP_DEBUG_FUNC();
  
    while(retry++ < 5)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        GTP_ERROR("GTP i2c test failed time %d.",retry);
        msleep(10);
    }
    return ret;
}

/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >= 0: succeed, < 0: failed
*******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
    s32 ret = 0;
    GTP_DEBUG_FUNC();
    ret = GTP_GPIO_REQUEST(gtp_int_gpio, "GTP INT IRQ");
    if (ret < 0) 
    {
        GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32)gtp_int_gpio, ret);
        ret = -ENODEV;
    }
    else
    {
        GTP_GPIO_AS_INT(gtp_int_gpio);  
        ts->client->irq = gpio_to_irq(gtp_int_gpio);
    }
#if GTP_RST_PORT
    ret = GTP_GPIO_REQUEST(gtp_rst_gpio, "GTP RST PORT");
    if (ret < 0) 
    {
        GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",(s32)gtp_rst_gpio,ret);
        ret = -ENODEV;
    }

    GTP_GPIO_AS_INPUT(gtp_rst_gpio);
#endif

    gtp_reset_guitar(ts->client, 20);
    
    if(ret < 0)
    {
#if GTP_RST_PORT
        GTP_GPIO_FREE(gtp_rst_gpio);
#endif
        GTP_GPIO_FREE(gtp_int_gpio);
    }

    return ret;
}

/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
        0: succeed, -1: failed.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    const u8 irq_table[] = GTP_IRQ_TAB;

    GTP_DEBUG_FUNC();
    GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);

    ret  = request_irq(ts->client->irq, 
                       goodix_ts_irq_handler,
                       irq_table[ts->int_trigger_type],
                       ts->client->name,
                       ts);
    if (ret)
    {
        /* 再申请中断失败的情况下， 使用定时器中断轮询 */
        GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
        GTP_GPIO_AS_INPUT(gtp_int_gpio);
        GTP_GPIO_FREE(gtp_int_gpio);

        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else 
    {
        gtp_irq_disable(ts);
        ts->use_irq = 1;
        return 0;
    }
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
        0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
    s8 ret = -1;
  
    GTP_DEBUG_FUNC();
  
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device.");
        return -ENOMEM;
    }

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;

    ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#if GTP_CHANGE_X2Y
    GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif
/*
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
*/
    input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

    ts->input_dev->name = goodix_ts_name;
    ts->input_dev->phys = goodix_input_phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;
    
    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", ts->input_dev->name);
        return -ENODEV;
    }

    return 0;
}

/* 
 * Devices Tree support, 
*/
/**
 * gtp_parse_dt - parse platform infomation form devices tree.
 */
static void gtp_parse_dt(struct device *dev)
{
    struct device_node *np = dev->of_node;

    gtp_int_gpio = of_get_named_gpio(np, "goodix,irq-gpio", 0);
#if GTP_RST_PORT
    gtp_rst_gpio = of_get_named_gpio(np, "goodix,rst-gpio", 0);
#endif

}

/**
 * gtp_parse_dt_cfg - parse config data from devices tree.
 * @dev: device that this driver attached.
 * @cfg: pointer of the config array.
 * @cfg_len: pointer of the config length.
 * @sid: sensor id.
 * Return: 0-succeed, -1-faileds
 */
int gtp_parse_dt_cfg(struct device *dev, u8 *cfg, int *cfg_len, u8 sid)
{
    struct device_node *np = dev->of_node;
    struct property *prop;
    char cfg_name[18];

    snprintf(cfg_name, sizeof(cfg_name), "goodix,cfg-group%d", (sid % 10));
    prop = of_find_property(np, cfg_name, cfg_len);
    if (!prop || !prop->value || *cfg_len == 0 || *cfg_len > GTP_CONFIG_MAX_LENGTH) {
        if (!prop) {
            printk("can not find %s \n", cfg_name);
        }
        printk("cfg_name %s \n", cfg_name);
        printk("cfg_len %d \n", *cfg_len);
        return -1;/* failed */
    } else {
        memcpy(cfg, prop->value, *cfg_len);
        return 0;
    }
}

/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes. 
        0: succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    struct goodix_ts_data *ts;
    u16 version_info;
    
    GTP_DEBUG_FUNC();
    
    //do NOT remove these logs
    GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
    GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

    i2c_connect_client = client;
    
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        GTP_ERROR("I2C check functionality failed.");
        return -ENODEV;
    }

    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        GTP_ERROR("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }

    /* device tree support */
    if (client->dev.of_node) {
        gtp_parse_dt(&client->dev);
    }

    INIT_WORK(&ts->work, goodix_ts_work_func);
    ts->client = client;
    spin_lock_init(&ts->irq_lock);          // 2.6.39 later
    // ts->irq_lock = SPIN_LOCK_UNLOCKED;   // 2.6.39 & before
    i2c_set_clientdata(client, ts);   
    ts->gtp_rawdiff_mode = 0;
    ret = gtp_request_io_port(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request IO port failed.");
        kfree(ts);
        //return ret;
        goto free_ts;
    }

    ret = gtp_i2c_test(client);
    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
        goto free_ts;
    }

    ret = gtp_read_version(client, &version_info);
    if (ret < 0)
    {
        GTP_ERROR("Read version failed.");
    }
    
    ret = gtp_init_panel(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
        goto free_ts;
    }

    ret = gtp_request_input_dev(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request input dev failed");
    }
    
    ret = gtp_request_irq(ts); 
    if (ret < 0)
    {
        GTP_INFO("GTP works in polling mode.");
    }
    else
    {
        GTP_INFO("GTP works in interrupt mode.");
    }

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
    
    /* register suspend and resume fucntion*/
    gtp_register_powermanger(ts);

    return 0;
free_ts:
    kfree(ts);
    return ret;
}


/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
    
    GTP_DEBUG_FUNC();

    gtp_unregister_powermanger(ts);

    if (ts) 
    {
        if (ts->use_irq)
        {
            GTP_GPIO_AS_INPUT(gtp_int_gpio);
            GTP_GPIO_FREE(gtp_int_gpio);
            free_irq(client->irq, ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
    #if GTP_RST_PORT
        GTP_GPIO_FREE(gtp_rst_gpio);
    #endif
    }

    GTP_INFO("GTP driver removing...");
    i2c_set_clientdata(client, NULL);
    input_unregister_device(ts->input_dev);
    kfree(ts);

    return 0;
}


/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_suspend(struct goodix_ts_data *ts)
{
    s8 ret = -1;    
    
    GTP_DEBUG_FUNC();
    if (ts->enter_update) {
        return;
    }
    GTP_INFO("System suspend.");

    ts->gtp_is_suspend = 1;

    if (ts->use_irq)
    {
        gtp_irq_disable(ts);
    }
    else
    {
        hrtimer_cancel(&ts->timer);
    }
    ret = gtp_enter_sleep(ts);

    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.");
    }
    // to avoid waking up while not sleeping
    //  delay 48 + 10ms to ensure reliability    
    msleep(58);   
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_resume(struct goodix_ts_data *ts)
{
    s8 ret = -1; 
    GTP_DEBUG_FUNC();
    if (ts->enter_update) {
        return;
    }
    //GTP_INFO("System resume.");
    
    ret = gtp_wakeup_sleep(ts);

    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }

    {
        gtp_send_cfg(ts->client);
    }

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    ts->gtp_is_suspend = 0;
}


#if   defined(CONFIG_FB)    
/* frame buffer notifier block control the suspend/resume procedure */
static int gtp_fb_notifier_callback(struct notifier_block *noti, unsigned long event, void *data)
{
    struct fb_event *ev_data = data;
    struct goodix_ts_data *ts = container_of(noti, struct goodix_ts_data, notifier);
    int *blank;
    
    if (ev_data && ev_data->data && event == FB_EVENT_BLANK && ts) {
        blank = ev_data->data;
        if (*blank == FB_BLANK_UNBLANK) {
            GTP_DEBUG("Resume by fb notifier.");
            goodix_ts_resume(ts);
                
        }
        else if (*blank == FB_BLANK_POWERDOWN) {
            GTP_DEBUG("Suspend by fb notifier.");
            goodix_ts_suspend(ts);
        }
    }

    return 0;
}
#endif

static int gtp_register_powermanger(struct goodix_ts_data *ts)
{
#if   defined(CONFIG_FB)
    ts->notifier.notifier_call = gtp_fb_notifier_callback;
    fb_register_client(&ts->notifier);
#endif

    return 0;
}

static int gtp_unregister_powermanger(struct goodix_ts_data *ts)
{
#if   defined(CONFIG_FB)
        fb_unregister_client(&ts->notifier);
#endif
    return 0;
}

/* end */


static const struct of_device_id goodix_match_table[] = {
        {.compatible = "goodix,gt9xx",},
        { },
};

static const struct i2c_device_id goodix_ts_id[] = {
    { GTP_I2C_NAME, 0 },
    { }
};

static struct i2c_driver goodix_ts_driver = {
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
    .id_table   = goodix_ts_id,
    .driver = {
        .name     = GTP_I2C_NAME,
        .owner    = THIS_MODULE,
        .of_match_table = goodix_match_table,
    },
};

/*******************************************************    
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int __init goodix_ts_init(void)
{
    s32 ret;

    GTP_DEBUG_FUNC();   
    GTP_INFO("GTP driver installing...");
    goodix_wq = create_singlethread_workqueue("goodix_wq");
    if (!goodix_wq)
    {
        GTP_ERROR("Creat workqueue failed.");
        return -ENOMEM;
    }
    ret = i2c_add_driver(&goodix_ts_driver);
    return ret; 
}

/*******************************************************    
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
    GTP_DEBUG_FUNC();
    GTP_INFO("GTP driver exited.");
    i2c_del_driver(&goodix_ts_driver);
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }
}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
