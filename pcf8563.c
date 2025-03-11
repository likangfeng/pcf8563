
/*
 * Copyright (c) 2020 panrui <https://github.com/Prry/rtt-pcf8563>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-01     panrui      the first version
 */
 
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include "pcf8563.h"

#ifdef PKG_USING_PCF8563

#define 	PCF8563_ARRD			(0xA2 >> 1)	/* slave address */

/* register */
#define		REG_PCF8563_STATE1		0x00
#define		REG_PCF8563_STATE2		0x01
#define		REG_PCF8563_SEC			0x02
#define		REG_PCF8563_MIN			0x03
#define		REG_PCF8563_HOUR		0x04
#define		REG_PCF8563_DAY			0x05
#define		REG_PCF8563_WEEK		0x06
#define		REG_PCF8563_MON			0x07
#define		REG_PCF8563_YEAR		0x08
#define		REG_PCF8563_CLKOUT		0x0d

/* offset */
#define 	SHIELD_PCF8563_STATE1   (unsigned char)0xa8
#define 	SHIELD_PCF8563_STATE2   (unsigned char)0x1f
#define 	SHIELD_PCF8563_SEC      (unsigned char)0x7f
#define 	SHIELD_PCF8563_MIN      (unsigned char)0x7f
#define 	SHIELD_PCF8563_HOUR     (unsigned char)0x3f
#define 	SHIELD_PCF8563_DAY      (unsigned char)0x3f
#define 	SHIELD_PCF8563_WEEK     (unsigned char)0x07
#define 	SHIELD_PCF8563_MON      (unsigned char)0x1f
#define 	SHIELD_PCF8563_YEAR     (unsigned char)0xff


/* pcf8563 device structure */
struct pcf8563_device
{
    struct rt_device rtc_parent;
    struct rt_i2c_bus_device *i2c_device;
};

#define	PCF8563_I2C_BUS		"i2c1"		/* i2c linked */
#define	PCF8563_DEVICE_NAME	"rtc"		/* register device name */

static struct pcf8563_device pcf8563_dev;	/* pcf8563 device */

/* bcd to hex */
static unsigned char bcd_to_hex(unsigned char data)
{
    unsigned char temp;

    temp = ((data>>4)*10 + (data&0x0f));
    return temp;
}

/* hex_to_bcd */
static unsigned char hex_to_bcd(unsigned char data)
{
    unsigned char temp;

    temp = (((data/10)<<4) + (data%10));
    return temp;
}

/* pcf8563 read register */
rt_err_t pcf8563_read_reg(rt_uint8_t reg,rt_uint8_t *data,rt_uint8_t data_size)
{
    struct rt_i2c_msg msg[2];
		
    msg[0].addr  = PCF8563_ARRD;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 1;
    msg[0].buf   = &reg;
    msg[1].addr  = PCF8563_ARRD;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = data_size;
    msg[1].buf   = data;

    if (rt_i2c_transfer(pcf8563_dev.i2c_device, msg, 2) == 2)
    {
		return RT_EOK;
	}
 	else
    {
	  	rt_kprintf("i2c bus write failed!\r\n");
        return -RT_ERROR;
    }
}

/* pcf8563 write register */
rt_err_t pcf8563_write_reg(rt_uint8_t reg, rt_uint8_t *data,rt_uint8_t data_size)
{
    struct rt_i2c_msg msg[2];

    msg[0].addr   	= PCF8563_ARRD;
    msg[0].flags	= RT_I2C_WR;
    msg[0].len   	= 1;
    msg[0].buf   	= &reg;
    msg[1].addr  	= PCF8563_ARRD;
    msg[1].flags	= RT_I2C_WR | RT_I2C_NO_START;
    msg[1].len   	= data_size;
    msg[1].buf   	= data;
	
    if (rt_i2c_transfer(pcf8563_dev.i2c_device, msg, 2) == 2)
	{
		return RT_EOK;
	}
 	else
    {
	  	rt_kprintf("i2c bus write failed!\r\n");
        return -RT_ERROR;
    }
}

static rt_err_t rt_pcf8563_open(rt_device_t dev, rt_uint16_t flag)
{
    if (dev->rx_indicate != RT_NULL)
    {
        /* open interrupt */
    }

    return RT_EOK;
}

static rt_ssize_t rt_pcf8563_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    return RT_EOK;
}

static rt_err_t rt_pcf8563_control(rt_device_t dev, int cmd, void *args)
{
	rt_err_t	ret = RT_EOK;
    time_t 		*time;
    struct tm 	time_temp;	
    rt_uint8_t 	buff[7];
	
    RT_ASSERT(dev != RT_NULL);
    rt_memset(&time_temp, 0, sizeof(struct tm));

    switch (cmd)
    {
    	/* read time */
        case RT_DEVICE_CTRL_RTC_GET_TIME:
	        time = (time_t *)args;
            ret = pcf8563_read_reg(REG_PCF8563_SEC,buff,7);

            if(ret == RT_EOK)
            {
                time_temp.tm_year  = bcd_to_hex(buff[6]&SHIELD_PCF8563_YEAR) + 2000 - 1900;
                time_temp.tm_mon   = bcd_to_hex(buff[5]&SHIELD_PCF8563_MON) - 1;
                time_temp.tm_mday  = bcd_to_hex(buff[3]&SHIELD_PCF8563_DAY);
                time_temp.tm_hour  = bcd_to_hex(buff[2]&SHIELD_PCF8563_HOUR);
                time_temp.tm_min   = bcd_to_hex(buff[1]&SHIELD_PCF8563_MIN);
                time_temp.tm_sec   = bcd_to_hex(buff[0]&SHIELD_PCF8563_SEC);

                *time = mktime(&time_temp);
            }
        break;

		/* set time */
        case RT_DEVICE_CTRL_RTC_SET_TIME:
        {
        	struct tm *time_new;
					
            time = (time_t *)args;
            time_new = localtime(time);
            buff[6] = hex_to_bcd(time_new->tm_year + 1900 - 2000);
            buff[5] = hex_to_bcd(time_new->tm_mon + 1);
            buff[3] = hex_to_bcd(time_new->tm_mday);
            buff[4] = hex_to_bcd(time_new->tm_wday+1);
            buff[2] = hex_to_bcd(time_new->tm_hour);
            buff[1] = hex_to_bcd(time_new->tm_min);
            buff[0] = hex_to_bcd(time_new->tm_sec);
            ret = pcf8563_write_reg(REG_PCF8563_SEC,buff,7);
        }
        break;
	#ifdef RT_USING_ALARM
		/* get alarm time */
		case RT_DEVICE_CTRL_RTC_GET_ALARM:
		{ 	
		  	struct rt_rtc_wkalarm *alm_time;
					
		  	ret = pcf8563_read_reg(REG_ALM1_SEC, buff, 4);
			if(ret == RT_EOK)
			{
			  	alm_time = (struct rt_rtc_wkalarm *)args;
				alm_time->tm_hour  = bcd_to_hex(buff[2]);
				alm_time->tm_min   = bcd_to_hex(buff[1]);
				alm_time->tm_sec   = bcd_to_hex(buff[0]);
			}
		}
		break;
		
		/* set alarm time */
		case RT_DEVICE_CTRL_RTC_SET_ALARM:
		{
			struct rt_rtc_wkalarm *alm_time;
					
            alm_time = (struct rt_rtc_wkalarm *)args;
            buff[3] = 0x80;	/* enable, alarm when hours, minutes, and seconds match */
            buff[2] = hex_to_bcd(alm_time->tm_hour);
            buff[1] = hex_to_bcd(alm_time->tm_min);
            buff[0] = hex_to_bcd(alm_time->tm_sec);
            ret = pcf8563_write_reg(REG_ALM1_SEC, buff, 4);
		}
		break;
	#endif
        default:
        break;
	}
    return ret;
}

int rt_hw_pcf8563_init(void)
{		
    struct rt_i2c_bus_device *i2c_device;
    uint8_t data;
	
    i2c_device = rt_i2c_bus_device_find(PCF8563_I2C_BUS);
    if (i2c_device == RT_NULL)
    {
        LOG_E("i2c bus device %s not found!\r\n", PCF8563_I2C_BUS);
        return -RT_ERROR;
    }				 	
    pcf8563_dev.i2c_device = i2c_device;
	
    /* register rtc device */
    pcf8563_dev.rtc_parent.type   		= RT_Device_Class_RTC;
    pcf8563_dev.rtc_parent.init    		= RT_NULL;
    pcf8563_dev.rtc_parent.open    		= rt_pcf8563_open;
    pcf8563_dev.rtc_parent.close   		= RT_NULL;
    pcf8563_dev.rtc_parent.read   		= rt_pcf8563_read;
    pcf8563_dev.rtc_parent.write  	 	= RT_NULL;
    pcf8563_dev.rtc_parent.control 		= rt_pcf8563_control;
    pcf8563_dev.rtc_parent.user_data 	= RT_NULL;			/* no private */
    rt_device_register(&pcf8563_dev.rtc_parent, PCF8563_DEVICE_NAME, RT_DEVICE_FLAG_RDWR);
	/* init pcf8563 */
    data = 0x7f;	/* close clock out */
    if (pcf8563_write_reg(REG_PCF8563_CLKOUT, &data, 1) != RT_EOK)
	{
		return -RT_ERROR;
	}

    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_pcf8563_init);

#endif /* PKG_USING_PCF8563 */
