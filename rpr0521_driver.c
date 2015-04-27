/* drivers/i2c/chips/rpr521_driver.c - ROHM RPR521 Linux kernel driver
 *
 * Copyright (C) 2012 
 * Written by Andy Mi <andy-mi@rohm.com.cn>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 *  This is Linux kernel modules for ambient light + proximity sensor
 *  Revision History
 *  2012-7-19:	Ver. 1.0	New release together with a porting guide.
 *  2012-8-14:	Ver. 1.1	Added calibration and set thresholds methods. Besides, the thresholds are automatically changed if a ps int is triggered to avoid constant interrupts.
 *  2014-1-09:	Ver. 1.2	Modified some functions for rpr521
 */
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h> 
#include <linux/bbk_alsps_para.h>
#include <linux/sensors_io.h>

#include "rpr0521_driver.h"
/*************** Global Data ******************/
/* parameter for als calculation */
#define COEFFICIENT               (4)
const unsigned long data0_coefficient[COEFFICIENT] = {839, 935, 666, 405};
const unsigned long data1_coefficient[COEFFICIENT] = {0, 520,  278,  144};
const unsigned long judge_coefficient[COEFFICIENT] = { 184,  1107,  1955, 2810}; 


#define _AUTO_THRESHOLD_CHANGE_
//#define _INIT_CALIB_  //zero modify in 20140505
u8 init_calib_flag = 0;   //grace modify in 2014.12.18
u8 init_ps_high = 0;  //grace modify in 2014.12.18
u8 init_ps_low = 0;  //grace modify in 2014.12.18
u8 calib_status = 0; //grace modify in 2014.12.18

/* mode control table */
#define MODE_CTL_FACTOR (16)
static const struct MCTL_TABLE {
    short ALS;
    short PS;
} mode_table[MODE_CTL_FACTOR] = {
    {  0,   0},   /*  0 */
    {  0,  10},   /*  1 */
    {  0,  40},   /*  2 */
    {  0, 100},   /*  3 */
    {  0, 400},   /*  4 */
    {100,  50},   /*  5 */
    {100, 100},   /*  6 */
    {100, 400},   /*  7 */
    {400,   0},   /*  8 */
    {400, 100},   /*  9 */
    {400,   0},   /* 10 */
    {400, 400},   /* 11 */
    {  50,  50},   /* 12 */
    {  0,   0},   /* 13 */
    {  0,   0},   /* 14 */
    {  0,   0}    /* 15 */
};

typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
    CMC_BIT_ENG_ALS = 3,
    CMC_BIT_ENG_PS  =4,
//	CMC_BIT_PS_ACROSS	   = 5,
} CMC_BIT;

/* gain table */
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
    /*char DATA0;
    char DATA1;*/
    unsigned char DATA0; //grace modify in 2014.5.7
    unsigned char DATA1; //grace modify in 2014.5.7
} gain_table[GAIN_FACTOR] = {
    {  1,   1},   /*  0 */
    {  0,   0},   /*  1 */
    {  0,   0},   /*  2 */
    {  0,   0},   /*  3 */
    {  2,   1},   /*  4 */
    {  2,   2},   /*  5 */
    {  0,   0},   /*  6 */
    {  0,   0},   /*  7 */
    {  0,   0},   /*  8 */
    {  0,   0},   /*  9 */
    { 64,  64},   /* 10 */
    {  0,   0},   /* 11 */
    {  0,   0},   /* 12 */
    {  0,   0},   /* 13 */
    {128,  64},   /* 14 */
    {128, 128}    /* 15 */
};

static struct sensors_classdev sensors_light_cdev1 = {
	.name = "rpr521-light",
	.vendor = "Rohm",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "6553",
	.resolution = "0.0125",
	.sensor_power = "0.15",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 50,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev1 = {
	.name = "rpr521-proximity",
	.vendor = "Rohm",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "0.18",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 50,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct platform_driver rpr521_alsps_driver;
static struct ALS_PS_DATA *rpr521_obj = NULL;
static struct i2c_client   *rpr521_i2c_client = NULL;

const struct ps_para  rpr521_ps_para[]  = bbk_tmp_ps_para();
static int ps_para_num  = 0;          //the value must select early

/*************** Functions ******************/
/******************************************************************************
 * NAME       : rpr521_set_enable
 * FUNCTION   : set measurement time according to enable
 * REMARKS    : this function will overwrite the work mode. if it is called improperly, 
 *			   you may shutdown some part unexpectedly. please check als_ps->enable first.
 *			   I assume it is run in normal mode. If you want low noise mode, the code should be modified.
 *****************************************************************************/
static void set_ps_para_num(int para)
{
	if( para >= 0 && para<= (sizeof(rpr521_ps_para)/sizeof(struct ps_para) -1))
		ps_para_num=para;
	else
		printk(KERN_ERR"%d is wrong ps index,[0,%d]\n",para,sizeof(rpr521_ps_para)/sizeof(struct ps_para) -1);
}
/*
static void set_als_para_num(int para)
{
	if( para >= 0 && para <= (sizeof(rpr521_als_para)/sizeof(struct als_para) -1 ))
		als_para_num=para;
	else
		printk(KERN_ERR"%d is wrong als index,[0,%d]\n",para,sizeof(rpr521_als_para)/sizeof(struct als_para) -1);
	
}*/
 
 /*----------------------------------------------------------------------------*/
static int rpr521_ps_select_para(struct ALS_PS_DATA *obj)         //
{
    int out_threshold, in_threshold;   
    int base = atomic_read(&obj->ps_base_value);
    int step_num;
    const struct ps_step *step;
    const struct ps_para *para;
    
    //TODO: get different parameter here
    para = &rpr521_ps_para[ps_para_num];
    step = &para->step[0];
    
    //count the the threshold from base value
    if(!(base >= para->base_value_min && base <= para->base_value_max))
        base = para->base_value;
    
    for(step_num = 0; step_num < para->step_num; step_num++, step++)
    {
        if(base >= step->min && base <= step->max)
            break;
    }
    
 //   if(device_id == 1)
    {
        in_threshold  = base + step->in_coefficient;
        out_threshold = base + step->out_coefficient;
    }
 //   else
 //   {
 //       in_threshold  = base * (step->in_coefficient) / 100;
 //       out_threshold = base * (step->out_coefficient) / 100;
 //   }
    printk("%s ,select_threshold in_coefficient %d out_coefficient %d\n",__func__, step->in_coefficient, step->out_coefficient);


    printk("rpr521 select_threshold in %d out %d base %d\n", in_threshold, out_threshold, base);

//    atomic_set(&obj->ps_in_threshold,  in_threshold);
//    atomic_set(&obj->ps_out_threshold, out_threshold);
	
	obj->ps_th_h = in_threshold;
	obj->ps_th_l = out_threshold;
    atomic_set(&obj->ps_base_value,    base);           //reset base value
	printk("rpr521 ps_in_threshold %d ps_out_threshold %d\n", obj->ps_th_h, obj->ps_th_l);

	return 0; 
}

static int rpr521_read_reg(struct i2c_client *client, u8 addr, u8 *data, int count)
{
    u8 temp_address;
    u8 *temp_data;
    int res = 0;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
    mutex_lock(&als_ps->update_lock); 

    temp_address = addr;
    temp_data = data;

    res = i2c_master_send(client, &temp_address, 0x1);
    if(res < 0)
    {
        printk(KERN_ERR"rpr521 read 0x%x fail %d\n", temp_address, res);
        mutex_unlock(&als_ps->update_lock); 
        return res;
    }
    
    res = i2c_master_recv(client, temp_data, count);
	if(res < 0)
	{
        printk(KERN_ERR"rpr521 read 0x%x fail %d\n", temp_address, res);
        mutex_unlock(&als_ps->update_lock); 
        return res;
	}
    
    mutex_unlock(&als_ps->update_lock); 
    
    return count;
}

/*
static int rpr521_write_reg(struct i2c_client *client, u8 addr, u8 *data, int count)
{
    u8 buffer[10] = {0};
    int res = 0;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

    mutex_lock(&als_ps->update_lock); 

    buffer[0] = addr;
    if(data != NULL && count > 0 && count < 5)
    {
        memcpy(&buffer[1], data, count);
    }

    res = i2c_master_send(client, buffer, (0x01 + count));
    if(res < 0)
    {
        printk(KERN_ERR"rpr521 write 0x%x fail %d\n", buffer[0], res);
        mutex_unlock(&als_ps->update_lock); 
        return res;
    }

    mutex_unlock(&als_ps->update_lock); 
    
    return (count + 1);
}*/

static int rpr521_set_enable(struct i2c_client *client, int enable)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;

	if(enable > 0xFb)
	{
		printk(KERN_ERR "%s: invalid measurement time setting.\n", __func__);
		return -EINVAL;
	}
	else
	{
		mutex_lock(&als_ps->update_lock);
		ret = i2c_smbus_write_byte_data(client, REG_MODECONTROL, enable);
		mutex_unlock(&als_ps->update_lock);

		als_ps->enable_stat = enable;
		als_ps->als_time = mode_table[(enable & 0xF)].ALS;
		als_ps->ps_time = mode_table[(enable & 0xF)].PS;

		return ret;
	}
}

static int rpr521_set_ps_threshold_low(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;

    /* check whether the parameter is valid */
	if(threshold > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	/*if(threshold > als_ps->ps_th_h)
	{
		printk(KERN_ERR "%s: higher than threshold high.\n", __func__);
		return -EINVAL;
	}*/ //grace modify in 2014.5.6
	
    /* write register to rpr521 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTL, sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
//	als_ps->ps_th_l = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}

static int rpr521_set_ps_threshold_high(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;

    /* check whether the parameter is valid */
	if(threshold > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	/*if(threshold < als_ps->ps_th_l)
	{
		printk(KERN_ERR "%s: lower than threshold low.\n", __func__);
		return -EINVAL;
	}*///grace modify in 2014.5.6
	
    /* write register to rpr521 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTH, sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
//	als_ps->ps_th_h = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}
/*
static int rpr521_calibrate(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int average;
 	unsigned int i, tmp, ps_th_h, ps_th_l;	
 	u8 infrared_data; //grace modify in 2014.12.18

	average = 0;
	
	//grace modify in 2014.12.18 begin
	calib_status = 0;
	tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR&0XFE); //disable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}	
  //grace modify in 2014.12.18 end
  
	//rpr521_set_enable(client, 0x41);	//PS 10ms
	rpr521_set_enable(client, PS_EN|PS_DOUBLE_PULSE|PS10MS);        //PS 10ms

  //grace modify in 2014.12.18 begin	
	 mdelay(20); 
	 tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
	 if(tmp < 0)	
	 {		
		goto err_exit;	
	 }	
	 infrared_data = tmp;	

	 if(infrared_data>>6)  
	 {		
		goto err_exit;
	 }
	 //grace modify in 2014.12.18 end
	 
	for(i = 0; i < 10; i ++)
	{
		mdelay(20);
		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			//goto err_exit;
		}
		average += tmp & 0xFFF;	// 12 bit data
	}
	average /= 10;

//	ps_th_h = average + PS_ALS_SET_PS_TH;
//	ps_th_l = average + PS_ALS_SET_PS_TL;
	ps_th_h = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	ps_th_l = average + THRES_TOLERANCE;

	if(ps_th_h < 0)
	{
		printk(KERN_ERR "%s: high threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_h > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: high threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l < 0)
	{
		printk(KERN_ERR "%s: low threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: low threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	
	//grace modify in 2014.12.18 begin
	if (average > PS_CROSSTALK)
	{
		if(init_calib_flag)
		{
			ps_th_h = init_ps_high;
			ps_th_l = init_ps_low;
		}
		else
		{
			ps_th_h = PS_ALS_SET_PS_TH;
			ps_th_l = PS_ALS_SET_PS_TL;
		}		
	}
//	else
//	{
		calib_status = 1;
//	}
	//grace modify in 2014.12.18 end

	if(!(rpr521_set_ps_threshold_high(client, ps_th_h)))
		als_ps->ps_th_h_back = ps_th_h;
	else 
		goto err_exit;
	if(!(rpr521_set_ps_threshold_low(client, ps_th_l)))
		als_ps->ps_th_l_back = ps_th_l;
	else
		goto err_exit;

	//rpr521_set_enable(client, 0);	//disable ps
  rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);//grace modify in 2014.5.6 
  //grace modify in 2014.12.18 begin
  tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}
	//grace modify in 2014.12.18 end
	return 0;		

err_exit:
	//rpr521_set_enable(client, 0);	//disable ps
	rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);//grace modify in 2014.5.6 
	//grace modify in 2014.12.18 begin
  tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}
	//grace modify in 2014.12.18 end

	return -1;
	
}*/

#if _FUNCTION_USED_	//masked because they are claimed but not used, which may cause error when compilling if the warning level is high enough. These functions provides some methods.
static int rpr521_set_persist(struct i2c_client *client, int persist)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	
    /* check whether the parameter is valid */
	if(persist > PERSISTENCE_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to rpr521 via i2c */
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_PERSISTENCE, persist);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->persistence = persist;	//Update the value after successful i2c write to avoid difference. 

	return 0;
}

static int rpr521_set_control(struct i2c_client *client, int control)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned char gain, led_current;
	
	if(control > REG_ALSPSCTL_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
	gain = (control & 0x3C) >> 2;	//gain setting values
	led_current = control & 0x03;		//led current setting value

	if(!((gain == ALSGAIN_X1X1) || (gain == ALSGAIN_X1X2) || (gain == ALSGAIN_X2X2) || (gain == ALSGAIN_X64X64)
		|| (gain == ALSGAIN_X128X64) || (gain == ALSGAIN_X128X128)))
	{
		printk(KERN_ERR "%s: invalid gain setting. \n", __func__);
		return -EINVAL;
	}
	
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_ALSPSCONTROL, control);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->control = control;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = led_current;

	return ret;
}
#endif

#if 0
/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static void long_long_divider(unsigned long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile unsigned long long divier;
    volatile unsigned long      unit_sft;

    if ((long long)data < 0)	// . If data MSB is 1, it may go to endless loop. 
    	{
    	data /= 2;	//0xFFFFFFFFFFFFFFFF / 2 = 0x7FFFFFFFFFFFFFFF
	base_divier /= 2;
    	}
    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while (data > divier) {
            unit_sft++;
            divier = divier << 1;
        }
        while (data > base_divier) {
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }
}

#else
/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static int long_long_divider(long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile long long divier;
    volatile long      unit_sft;

    if ((data < 0) || (base_divier == 0)) {
        *answer   = 0;
        *overplus = 0;
        return (CALC_ERROR);
    }

    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while ((data > divier) && (divier > 0)) {
            unit_sft++;
            divier = divier << 1;
        }
        //while ((data > base_divier) && (unit_sft > 0)) {
        while ((data > base_divier) && (unit_sft >= 0)) { //grace modify in 2014.8.19
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }

    return (0);
}
#endif

/******************************************************************************
 * NAME       : calc_rohm_als_data
 * FUNCTION   : calculate illuminance data for rpr521
 * REMARKS    : final_data is 1000 times, which is defined as CUT_UNIT, of the actual lux value
 *****************************************************************************/
//static int calc_rohm_als_data(unsigned short data0, unsigned short data1, unsigned char gain0, unsigned char gain1, unsigned char time)
static int calc_rohm_als_data(unsigned short data0, unsigned short data1, unsigned char gain0, unsigned char gain1, unsigned short time) //grace modify in 2014.5.7
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (1000)
#define MAX_OUTRANGE    (65535)
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)

	int                result; //grace modified in 2014.4.2
	int                final_data;
	CALC_DATA          calc_data;
	CALC_ANS           calc_ans;
	unsigned long      calc_judge;
	unsigned char      set_case;
	unsigned long      div_answer;
	unsigned long long div_overplus;
	unsigned long long overplus;
	unsigned long      max_range;

	/* set the value of measured als data */
	calc_data.als_data0  = data0;
	calc_data.als_data1  = data1;
	calc_data.gain_data0 = gain0;

	/* set max range */
	if (calc_data.gain_data0 == 0) 
	{
		/* issue error value when gain is 0 */
		return (CALC_ERROR);
	}
	else
	{
		max_range = MAX_OUTRANGE / calc_data.gain_data0;
	}
	
	/* calculate data */
	if (calc_data.als_data0 == MAXRANGE_NMODE) 
	{
		calc_ans.positive = max_range;
		calc_ans.decimal  = 0;
	} 
	else 
	{
		/* get the value which is measured from power table */
		calc_data.als_time = time;
		if (calc_data.als_time == 0) 
		{
			/* issue error value when time is 0 */
			return (CALC_ERROR);
		}

		calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
		if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])) 
		{
			set_case = 0;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1]))
		{
			set_case = 1;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2])) 
		{
			set_case = 2;
		}
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3])) 
		{
			 set_case = 3;
		} 
		else
		{
			set_case = MAXSET_CASE;
		}
		calc_ans.positive = 0;
		if (set_case >= MAXSET_CASE) 
		{
			calc_ans.decimal = 0;	//which means that lux output is 0
		}
		else
		{
			calc_data.gain_data1 = gain1;
			if (calc_data.gain_data1 == 0) 
			{
				/* issue error value when gain is 0 */
				return (CALC_ERROR);
			}
			calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
			calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
			if(calc_data.data0 < calc_data.data1)	//In this case, data will be less than 0. As data is unsigned long long, it will become extremely big.
			{
				return (CALC_ERROR);
			}
			else
			{
				calc_data.data       = (calc_data.data0 - calc_data.data1);
			}
			calc_data.dev_unit   = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;	//24 bit at max (128 * 128 * 100 * 10)
			if (calc_data.dev_unit == 0) 
			{
				/* issue error value when dev_unit is 0 */
				return (CALC_ERROR);
			}

			/* calculate a positive number */
			div_answer   = 0;
			div_overplus = 0;
#if 0  //grace modified in 2014.4.2
			long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus); 
#else
			result = long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus);
      if (result == CALC_ERROR)
      {
         return (result);
      }
#endif
			calc_ans.positive = div_answer;
			/* calculate a decimal number */
			calc_ans.decimal = 0;
			overplus         = div_overplus;
			if (calc_ans.positive < max_range)
			{
				if (overplus != 0)
				{
					overplus     = overplus << DECIMAL_BIT;
					div_answer   = 0;
					div_overplus = 0;
#if 0 //grace modified in 2014.4.2
					long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
#else
					result = long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
					if (result == CALC_ERROR)
      		{
         		return (result);
      		}
#endif
					calc_ans.decimal = div_answer;
				}
			}

			else
			{
				calc_ans.positive = max_range;
			}
		}
	}
	
	final_data = calc_ans.positive * CUT_UNIT + ((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT);
					
	return (final_data);

#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
}


/******************************************************************************
 * NAME       : calculate_ps_data
 * FUNCTION   : calculate proximity data for rpr521
 * REMARKS    : 12 bit output
 *****************************************************************************/
static int calc_rohm_ps_data(unsigned short ps_reg_data)
{
    return (ps_reg_data & 0xFFF);
}

static unsigned int rpr521_als_data_to_level(unsigned int als_data)
{
#if 0
#define ALS_LEVEL_NUM 15
	int als_level[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
	int als_value[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
    	unsigned char idx;

	for(idx = 0; idx < ALS_LEVEL_NUM; idx ++)
	{
		if(als_data < als_value[idx])
		{
			break;
		}
	}
	if(idx >= ALS_LEVEL_NUM)
	{
		printk(KERN_ERR "rpr521 als data to level: exceed range.\n");
		idx = ALS_LEVEL_NUM - 1;
	}
	
	return als_level[idx];
#undef ALS_LEVEL_NUM
#else
	return als_data;
#endif
}

static void rpr521_reschedule_work(struct ALS_PS_DATA *als_ps,
					  unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&als_ps->dwork);
	schedule_delayed_work(&als_ps->dwork, delay);

	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
}

static int rpr521_read_als(struct i2c_client *client, unsigned int *data){
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client); 
	int tmp = 0;
	
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read data0 fail. \n", __func__);
		return tmp;
	}
	als_ps->als_data0_raw = (unsigned short)tmp;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA1);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read data1 fail. \n", __func__);
		return tmp;
	}
	als_ps->als_data1_raw = (unsigned short)tmp;

// Theorically it is not necesssary to do so, but I just want to avoid any potential error.  -- Andy 2012.6.6
	tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read gain fail. \n", __func__);
		return tmp;
	}
	tmp = (tmp & 0x3C) >> 2;
	als_ps->gain0 = gain_table[tmp].DATA0;
	als_ps->gain1 = gain_table[tmp].DATA1;	
	
	tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
	printk("REG_MODECONTROL=%x\n", tmp);

	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read time fail. \n", __func__);
		return tmp;
	}
	tmp = tmp & 0xF;
	als_ps->als_time = mode_table[tmp].ALS;	
	
	als_ps->als_data = calc_rohm_als_data(als_ps->als_data0_raw, als_ps->als_data1_raw, als_ps->gain0, als_ps->gain1, als_ps->als_time);	
//	if(als_ps->als_data == 0)
//		als_ps->als_data ++;

	als_ps->als_level = rpr521_als_data_to_level(als_ps->als_data);

	printk(KERN_INFO "rpr521 als report: data0 = %d, data1 = %d, gain0 = %d, gain1 = %d, time = %d, lux = %d, level = %d.\n", als_ps->als_data0_raw, 
		als_ps->als_data1_raw, als_ps->gain0, als_ps->gain1, als_ps->als_time, als_ps->als_data, als_ps->als_level);

//	tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
//	printk("REG_PSDATA=%x\n", tmp);
	//tmp = i2c_smbus_read_byte_data(client, REG_INTERRUPT);
	//printk("REG_INTERRUPT=%x\n", tmp);
	*data = als_ps->als_data;

	return tmp;
}

static int rpr521_read_ps(struct i2c_client *client, unsigned int *data){
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client); 
	int tmp;
	
	tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
	
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
		return -1;
	}
	als_ps->ps_data_raw = (unsigned short)tmp;

	als_ps->ps_data = calc_rohm_ps_data(als_ps->ps_data_raw);
	
	return 0;
}

int rpr521_enable_eint(struct i2c_client *client,int enable){
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client); 
	
	if(als_ps->ps_direction == 0){
		printk("rpr521 wait far away\n");
		printk("rpr521 out is %d,in is %d\n",als_ps->ps_th_l,als_ps->ps_th_h);
		rpr521_set_ps_threshold_high(client, REG_PSTH_MAX);
		rpr521_set_ps_threshold_low(client, als_ps->ps_th_l);		
	}
	else if(als_ps->ps_direction == 1){
		printk("rpr521 wait close\n");     
		printk("rpr521 out is %d,in is %d\n",als_ps->ps_th_l,als_ps->ps_th_h);
		rpr521_set_ps_threshold_high(client, als_ps->ps_th_h);
		rpr521_set_ps_threshold_low(client, 0);
	}
	
	return 0;
}

int rpr521_get_ps_value(struct i2c_client *client){
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client); 
//	int tmp;
	int val;
	
	if(als_ps->ps_data > als_ps->ps_th_h){
		val = 0;
	}
	else if(als_ps->ps_data < als_ps->ps_th_l){
		val = 1;
	}
	else{
		val = als_ps->ps_direction;
	}
	
	if(val != als_ps->ps_direction){
	
		printk(KERN_ERR"change ps from %d to %d \n",als_ps->ps_direction,val);
		if(val == 0)//close
			als_ps->ps_direction = 0;
		if(val == 1)//far away
			als_ps->ps_direction = 1;
		rpr521_enable_eint(client,1);
	}
		
/*		if(val == 0)//close
		{		
			//grace modify in 2014.5.7 begin	
			tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
			if(tmp < 0)
			{
				printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
				return tmp; 
			}
			if ((tmp>>6) == INFRARED_LOW)
			{
				als_ps->ps_direction = 0;
	#ifdef _AUTO_THRESHOLD_CHANGE_
				rpr521_set_ps_threshold_high(client, REG_PSTH_MAX);
				rpr521_set_ps_threshold_low(client, als_ps->ps_th_l_back);
	#endif
			}
			else
			{
				return -1;
			}
			//grace modify in 2014.5.7 end			
		}
		else if(val == 1)//far away
		{
			als_ps->ps_direction = 1;
	#ifdef _AUTO_THRESHOLD_CHANGE_
			rpr521_set_ps_threshold_high(client, als_ps->ps_th_h_back);
			rpr521_set_ps_threshold_low(client, 0);
	#endif
		}
		
	}*/

/*	tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read persistence fail. \n", __func__);
		//return;
		goto err_exit; //grace modify in 2014.9.5
	}*/
	
	return 0;
}
/* ALS PS polling routine */
static void rpr521_als_polling_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of(work, struct ALS_PS_DATA, als_dwork.work);
	struct i2c_client *client=als_ps->client;
	static u32 index = 0; 
	unsigned int als_data;
	int tmp;
	
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// restart timer
	
	tmp = rpr521_read_als(client,&als_data);
	
	if(als_data != CALC_ERROR && tmp >= 0)
	{
		input_report_abs(als_ps->input_dev_als, ABS_X, als_data); // report als data. maybe necessary to convert to lux level
		input_report_abs(als_ps->input_dev_als, ABS_Y, index%2); 
		input_sync(als_ps->input_dev_als);	
		index++;
	}	
	
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// restart timer
}

static void rpr521_ps_polling_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of(work, struct ALS_PS_DATA, ps_dwork.work);
	struct i2c_client *client=als_ps->client;
	int tmp;
	
	tmp = rpr521_read_ps(client,&als_ps->ps_data);
	if(tmp < 0)
		goto restart_poll;
	
	tmp = rpr521_get_ps_value(client);
	if(tmp < 0)
		goto restart_poll;
	
	printk(KERN_INFO "rpr521 ps report: raw_data = %d, data = %d, direction = %d. \n", 
		als_ps->ps_data_raw, als_ps->ps_data, als_ps->ps_direction);
	printk(KERN_INFO "rpr521 ps report: ps_high = %d, ps_low = %d. \n",
        i2c_smbus_read_word_data(client, REG_PSTH), i2c_smbus_read_word_data(client, REG_PSTL));

	input_report_rel(als_ps->input_dev_ps, REL_X, (als_ps->ps_direction + 1)); 
	input_sync(als_ps->input_dev_ps);	
	
restart_poll:
	schedule_delayed_work(&als_ps->ps_dwork, msecs_to_jiffies(als_ps->ps_poll_delay));	// restart timer
}
/* PS interrupt routine */
static void rpr521_ps_int_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of((struct delayed_work *)work, struct ALS_PS_DATA, dwork);
	struct i2c_client *client=als_ps->client;
	int tmp;

	printk("rpr521_ps_int_work_handler\n");

	tmp =  i2c_smbus_read_byte_data(client, REG_INTERRUPT);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read interrupt status fail. \n", __func__);
		//return;
		goto err_exit; //grace modify in 2014.9.5
	}
	if(tmp & PS_INT_MASK)	//Interrupt is caused by PS
	{
/*		tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			//return;
			goto err_exit; //grace modify in 2014.5.7
		}
		als_ps->ledcurrent = tmp & 0x3;*/
		
		schedule_delayed_work(&als_ps->ps_dwork, 0);	// restart timer
	}
	else
	{
		printk(KERN_ERR "%s: unknown interrupt source.\n", __func__);
	}
	
	enable_irq(client->irq);
	
//grace modify in 2014.5.7 begin
err_exit:
	enable_irq(client->irq);
//grace modify in 2014.5.7 end
}

/* assume this is ISR */
static irqreturn_t rpr521_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	disable_irq_nosync(client->irq);
	printk("%s\n", __func__);
	rpr521_reschedule_work(als_ps, 0);

	return IRQ_HANDLED;
}

/*************** SysFS Support ******************/
static ssize_t show_read_register(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR"i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "REG:%s\n", obj->read_register);
}

static ssize_t store_read_register(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *obj=i2c_get_clientdata(client);	 
	u8 register_value[1];
	u8 register_addr[1];
	int res = 0;
	
	if(client == NULL )
	{
		printk("CLIENT CANN'T EQUL NULL\n");
		sprintf(obj->read_register,"CLIENT=NULL");
		return -1;
	}

    register_addr[0]=simple_strtoul(buf, NULL, 16)&0xff;
    res = rpr521_read_reg(client,register_addr[0], register_value, 0x1);
    if(res <= 0)
    {
        goto error_fs;
    }
    sprintf(obj->read_register,"0x%02x=0x%02x",register_addr[0],register_value[0]);
	return count;

error_fs:
	sprintf(obj->read_register,"I2C error");
	return count;
}
/*
static ssize_t show_als_para_index(struct device_driver *ddri, char *buf)
{
    //struct als_para *para;
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *obj=i2c_get_clientdata(client);
    //int base;

    rpr521_als_select_para(obj);

	if(NULL == obj)
	{
		printk("i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", als_para_num);
}

static ssize_t store_als_para_index(struct device_driver *ddri, const char *buf, size_t count)
{
	//do nothing
	//struct i2c_client *client = rpr521_i2c_client;
	//struct ALS_PS_DATA *obj=i2c_get_clientdata(client);	 
	int num =0;
    num=simple_strtoul(buf, NULL, 10);
    set_als_para_num(num);
	return count;
}
static DRIVER_ATTR(als_para_index,   S_IWUSR | S_IRUGO, show_als_para_index, store_als_para_index);
*/

static ssize_t store_ps_para_index(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *obj=i2c_get_clientdata(client);	 
	int num =0;
    num=simple_strtoul(buf, NULL, 10);
    set_ps_para_num(num);
    rpr521_ps_select_para(obj);
	return count;
}
static ssize_t show_ps_para_index(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", ps_para_num);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_ps_base_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *obj=i2c_get_clientdata(client);

    rpr521_ps_select_para(obj);
	if(NULL == obj)
	{
		printk(KERN_ERR"i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", atomic_read(&obj->ps_base_value));
}

static ssize_t store_ps_base_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *obj=i2c_get_clientdata(client);
	u16 thd_value = (u16)simple_strtoul(buf, NULL, 10);
	//unsigned char databuf[2];
	//int res;
	if( obj != NULL && client != NULL && thd_value >= 0 )
    {
        const struct ps_para *para;
    
        //check the value
        para = &rpr521_ps_para[ps_para_num];
        if(!(thd_value >= para->base_value_min && thd_value <= para->base_value_max))    //if the value is error, do nothing.
            return count;

        printk(KERN_INFO"mean to set ps_base_value=%d\n", thd_value);
        atomic_set(&obj->ps_base_value, thd_value);
        //xuling: force re-enable the proximit sensor
        //comment by dengweicheng :store ps_base_value anyway

    //    {
            rpr521_ps_select_para(obj);     //reselect the threshold agian 
			//enable 
//			if(test_bit(CMC_BIT_PS,&obj->enable) || test_bit(CMC_BIT_ENG_PS,&obj->enable) || test_bit(CMC_BIT_PS_ACROSS,&obj->enable)){
//				rpr521_enable_eint(obj,0);
				rpr521_enable_eint(client,1);
//			}
            //rpr521_enable_ps(client,0);
			//mdelay(5);
			//rpr521_enable_ps(client,1);
    //    }

        printk("%s finnal ps_base_value=%d\n",__func__,atomic_read(&obj->ps_base_value));
	}
	else 
      {
		printk(KERN_ERR"set LTR558 ps threshold FAIL!!\n");
	}
    
	return count;
}

static ssize_t rpr521_show_enable_ps_sensor(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->enable_ps_sensor);
}

static int rpr521_enable_ps_sensor(struct i2c_client *client,int enable){
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long flags;
	int tmp;
	if(enable) 
	{
		//turn on p sensor
		//wake_lock(&ps_lock);

		if (als_ps->enable_ps_sensor == 0) 
		{
			als_ps->enable_ps_sensor = 1;
		
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			printk("%s: tmp=%x\n", __func__, tmp);
			if(tmp < 0){
				printk(KERN_ERR"fail enable:%d in %s",enable,__func__);
				return -1;
			}
			//initial ps int
			als_ps->ps_direction = 1;
			rpr521_ps_select_para(als_ps);  
			rpr521_enable_eint(client,1); 
			
			tmp = tmp | PS_EN;
			rpr521_set_enable(client, tmp);	//PS on and ALS off
			
			
			spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
			cancel_delayed_work(&als_ps->ps_dwork);
			schedule_delayed_work(&als_ps->ps_dwork, msecs_to_jiffies(als_ps->ps_poll_delay));	// 125ms
			spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
		}
	} 
	else 
	{
		if(als_ps->enable_ps_sensor == 1)
		{
			als_ps->enable_ps_sensor = 0;
			
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			printk("%s: tmp=%x\n", __func__, tmp);
			if(tmp < 0){
				printk(KERN_ERR"fail enable:%d in %s",enable,__func__);
				return -1;
			}
			tmp = tmp & (~PS_EN);
			rpr521_set_enable(client, tmp);	//PS on and ALS off
			
			//wake_unlock(&ps_lock);
			spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
			cancel_delayed_work(&als_ps->ps_dwork);
			spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
		}
	}
	
	return 0;
}
static ssize_t rpr521_store_enable_ps_sensor(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
// 	unsigned long flags;
//	struct input_dev *input_dev = als_ps->input_dev_ps;

	printk(KERN_INFO "rpr521 enable PS sensor -> %ld \n", val);
		
	if ((val != 0) && (val != 1)) 
	{
		printk("%s:store unvalid value=%ld\n", __func__, val);
		return count;
	}
	
	if(val == 1){
		rpr521_enable_ps_sensor(client,1);
		set_bit(CMC_BIT_PS, &als_ps->enable);
	}
	else{
		rpr521_enable_ps_sensor(client,0);
		clear_bit(CMC_BIT_PS, &als_ps->enable);
	}
	
	return count;
}

static int ps_enable_set(struct sensors_classdev *sensors_cdev,unsigned int enable){
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int err = 0;
	
	if ((enable != 0) && (enable != 1)) 
	{
		printk("%s:set unvalid value=%d\n", __func__, enable);
		return -1;
	}
	
	err = rpr521_enable_ps_sensor(client,enable);
	
	if(enable)
		set_bit(CMC_BIT_PS, &als_ps->enable);
	else
		clear_bit(CMC_BIT_PS, &als_ps->enable);
		
	return err;
}

static ssize_t rpr521_show_enable_als_sensor(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->enable_als_sensor);
}

static int rpr521_enable_als_sensor(struct i2c_client *client,int enable){
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long flags;
	int tmp;
	if(enable)
	{
		//turn on light  sensor
		if (als_ps->enable_als_sensor==0)
		{
			als_ps->enable_als_sensor = 1;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
            printk("%s: tmp=%x\n", __func__, tmp);
			if(tmp < 0){
				printk(KERN_ERR"fail enable:%d in %s",enable,__func__);
				return -1;
			}
			tmp = tmp | ALS_EN;
			rpr521_set_enable(client, tmp);	//PS on and ALS off
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	else
	{
		if(als_ps->enable_als_sensor == 1)
		{
			als_ps->enable_als_sensor = 0;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			if(tmp < 0){
				printk(KERN_ERR"fail enable:%d in %s",enable,__func__);
				return -1;
			}
			tmp = tmp & (~ALS_EN);
			rpr521_set_enable(client, tmp);	//PS on and ALS off
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		cancel_delayed_work(&als_ps->als_dwork);
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	
	return 0;
}
static ssize_t rpr521_store_enable_als_sensor(struct device_driver *ddri,const char *buf, size_t count)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	printk(KERN_INFO "rpr521 enable ALS sensor -> %ld\n", val);

	if ((val != 0) && (val != 1))
	{
		printk("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}
	
	if(val == 1){
		rpr521_enable_als_sensor(client,1);
		set_bit(CMC_BIT_ALS, &als_ps->enable);
	}
	else{
		rpr521_enable_als_sensor(client,0);
		clear_bit(CMC_BIT_ALS, &als_ps->enable);
	}
		
	return count;
}

static int als_enable_set(struct sensors_classdev *sensors_cdev,unsigned int enable){
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int err = 0;
	
	if ((enable != 0) && (enable != 1)) 
	{
		printk("%s:set unvalid value=%d\n", __func__, enable);
		return -1;
	}
	
	err = rpr521_enable_als_sensor(client,1);
	
	if(enable){
		set_bit(CMC_BIT_ALS, &als_ps->enable);
	}
	else{
		clear_bit(CMC_BIT_ALS, &als_ps->enable);
	}
	return err;
}

static ssize_t rpr521_show_als_poll_delay(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->als_poll_delay*1000);	// return in micro-second
}

static ssize_t rpr521_store_als_poll_delay(struct device_driver *ddri,const char *buf, size_t count)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
//	int ret;
//	int poll_delay = 0;
 	unsigned long flags;
	
	if (val < PS_ALS_SET_MIN_DELAY_TIME * 1000)
		val = PS_ALS_SET_MIN_DELAY_TIME * 1000;	
	
	als_ps->als_poll_delay = val /1000;	// convert us => ms
	
	if (als_ps->enable_als_sensor == 1)//grace modified in 2013.10.09
	{
	
	/* we need this polling timer routine for sunlight canellation */
	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&als_ps->als_dwork);
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
			
	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	
	}
	return count;
}

//grace add in 2013.10.09 begin
static ssize_t rpr521_show_als_data(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	int tmp;
	int tmp1;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	tmp1 = i2c_smbus_read_word_data(client, REG_ALSDATA1);

	return sprintf(buf, "%d %d\n", tmp, tmp1);
}

static ssize_t rpr521_show_ps_data(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	int tmp = 0;
	tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
	return sprintf(buf, "%d\n", tmp);
}
//grace add in 2013.10.09 end

static ssize_t rpr521_show_ps_thres_high(struct device_driver *ddri,char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	//struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	//grace test in 2013.10.09 begin
	int ps_data = 0, ps_high = 0, ps_low = 0;
	ps_data = i2c_smbus_read_word_data(client, REG_PSDATA);
	if(ps_data < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	ps_high = i2c_smbus_read_word_data(client, REG_PSTH);
	if(ps_high < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	ps_low = i2c_smbus_read_word_data(client, REG_PSTL);
	if(ps_low < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	//return sprintf(buf, "%d\n", als_ps->ps_th_h);	
	return sprintf(buf, "%d %d %d\n", ps_data, ps_high, ps_low);
	//grace test in 2013.10.09 end
}

static ssize_t rpr521_store_ps_thres_high(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	if(!(rpr521_set_ps_threshold_high(client, val)))
		als_ps->ps_th_h_back = als_ps->ps_th_h;
	
	return count;
}

static ssize_t rpr521_show_ps_thres_low(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->ps_th_l);	
}

static ssize_t rpr521_store_ps_thres_low(struct device_driver *ddri,const char *buf, size_t count)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	if(!(rpr521_set_ps_threshold_low(client, val)))
		als_ps->ps_th_l_back = als_ps->ps_th_l;
	
	return count;
}

/*
static ssize_t rpr521_show_ps_calib(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\t%d\n", als_ps->ps_th_h, als_ps->ps_th_l);	
}


static ssize_t rpr521_store_ps_calib(struct device_driver *ddri,const char *buf, size_t count)
{
#define SET_LOW_THRES	1
#define SET_HIGH_THRES	2
#define SET_BOTH_THRES	3

	struct i2c_client *client = rpr521_i2c_client;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	unsigned int i, tmp, ps_th_h, ps_th_l;
	int average;	//This should be signed to avoid error.
	
	switch(val)
	{
		case SET_LOW_THRES:		
		//Take 20 average for noise. use noise + THRES_TOLERANCE as low threshold.
		//If high threshold is lower than the new low threshold + THRES_DIFF, make the high one equal low + THRES_DIFF
		//Please make sure that there is NO object above the sensor, otherwise it may cause the high threshold too high to trigger which make the LCD NEVER shutdown.
		//If the noise is too large, larger than 4065, it will return -1. If so, the mechanical design MUST be redo. It is quite unlikely. 
			average = 0;
			ps_th_h = als_ps->ps_th_h_back;
			ps_th_l = als_ps->ps_th_l_back;
			for(i = 0; i < 20; i ++)
			{
				tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
				if(tmp < 0)
				{
					printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
					return -1;
				}
				average += tmp & 0xFFF;	// 12 bit data
			}
			average /= 20;		//This is the average noise
			ps_th_l = average + THRES_TOLERANCE;	
			if(ps_th_l > REG_PSTL_MAX)
			{
			//printk(KERN_ERR "%d in %s: low threshold is too high. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_l < 0)
			{
			//printk(KERN_ERR "%d in %s: low threshold is too low. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_h < ps_th_l + THRES_DIFF)
			{
				ps_th_h = ps_th_l + THRES_DIFF;	//It will not be minus or an error should have occured earlier. 
				if(ps_th_h > REG_PSTH_MAX)
				{
			//printk(KERN_ERR "%d in %s: high threshold is too high. \n", __line__, __func__);
					return -1;
				}
				if(!rpr521_set_ps_threshold_high(client, ps_th_h))
					als_ps->ps_th_h_back = ps_th_h;
				else
					return -1;
			}
			if(!rpr521_set_ps_threshold_low(client, ps_th_l))
				als_ps->ps_th_l_back = ps_th_l;
			else
				return -1;
			break;
		
		case SET_HIGH_THRES:	
		//Take 20 average for signal. use signal -THRES_TOLERANCE as high threshold. 
		//If low threshold is higher than the new high one - THRES_DIFF, make the low one equal high - THRES_DIFF
		//Please make sure that there IS AN object above the sensor, otherwise it will be a disaster. The high threshold will be too low which will cause the panel ALWAYS shutdown
		//Customer can use their own standard to set the test scenario. For example, a 18% grey card @ 2cm, or another example, a 90% white card 4cm, and etc. 
		//If the signal is too weak, less than 30, it will return -1. If so, the mechanical design MUST be redo. It shall not happen very frequently. 
			average = 0;
			ps_th_h = als_ps->ps_th_h_back;
			ps_th_l = als_ps->ps_th_l_back;
			for(i = 0; i < 20; i ++)
			{
				tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
				if(tmp < 0)
				{
					printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
					return -1;
				}
				average += tmp & 0xFFF;	// 12 bit data
			}
			average /= 20;		//This is the average signal
			ps_th_h = average - THRES_TOLERANCE;
			if(ps_th_h > REG_PSTH_MAX)
			{
			//printk(KERN_ERR "%d in %s: high threshold is too high. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_h < 0)
			{
			//printk(KERN_ERR "%d in %s: high threshold is too low. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_l > ps_th_h - THRES_DIFF)
			{
				ps_th_l = ps_th_h - THRES_DIFF;	//Given that REG_PSTH_MAX = REG_PSTL+MAX, it will not be greater than REG_PSTL_MAX or an error should have occured earlier.
				if(ps_th_l < 0)
				{
					//printk(KERN_ERR "%d in %s: low threshold is too low. \n", __line__, __func__);
					return -1;
				}
				if(!rpr521_set_ps_threshold_low(client, ps_th_l))
					als_ps->ps_th_l_back = ps_th_l;
				else
					return -1;
			}
			if(!rpr521_set_ps_threshold_high(client, ps_th_h))
				als_ps->ps_th_h_back = ps_th_h;
			else
				return -1;
			break;
		
		case SET_BOTH_THRES:	//Take 20 average for noise. use noise + PS_ALS_SET_PS_TL as low threshold, noise + PS_ALS_SET_PS_TH as high threshold
			rpr521_calibrate(client);
			break;

		default:
			return -EINVAL;	//NOT supported!
	}
			
	return count;

#undef SET_BOTH_THRES
#undef SET_HIGH_THRES
#undef SET_LOW_THRES
}*/

static ssize_t rpr521_show_type(struct device_driver *ddri, char *buf){
    struct i2c_client *client = rpr521_i2c_client;
    struct ALS_PS_DATA *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->type);
}
static DRIVER_ATTR(read_register,  0660,
				    show_read_register, store_read_register);
					
static DRIVER_ATTR(als_poll_delay,  0660,
				    rpr521_show_als_poll_delay, rpr521_store_als_poll_delay);

static DRIVER_ATTR(enable_als_sensor,  0660 ,
				  rpr521_show_enable_als_sensor, rpr521_store_enable_als_sensor);

static DRIVER_ATTR(enable_ps_sensor,  0660 ,
				   rpr521_show_enable_ps_sensor, rpr521_store_enable_ps_sensor);

static DRIVER_ATTR(ps_thres_high,  0660 ,
				  rpr521_show_ps_thres_high, rpr521_store_ps_thres_high);

static DRIVER_ATTR(ps_thres_low,  0660 ,
				   rpr521_show_ps_thres_low, rpr521_store_ps_thres_low);

//static DRIVER_ATTR(ps_calib,  0660 ,
//				   rpr521_show_ps_calib, rpr521_store_ps_calib);
static DRIVER_ATTR(als_data, S_IRUGO, rpr521_show_als_data, NULL); //grace add in 2013.10.09
static DRIVER_ATTR(ps_data, S_IRUGO, rpr521_show_ps_data, NULL);//grace add in 2013.10.09
static DRIVER_ATTR(type, S_IRUGO, rpr521_show_type, NULL);//Add for EngineerMode

static DRIVER_ATTR(ps_para_index,  0660 ,
				   show_ps_para_index, store_ps_para_index);
				   
static DRIVER_ATTR(ps_base_value,  0660 ,
				   show_ps_base_value, store_ps_base_value);

static struct driver_attribute *rpr521_attributes[] = {
	&driver_attr_read_register,
	&driver_attr_enable_ps_sensor,
	&driver_attr_enable_als_sensor,
	&driver_attr_als_poll_delay,
	&driver_attr_ps_thres_high,
	&driver_attr_ps_thres_low,
//	&driver_attr_ps_calib,
	&driver_attr_als_data,  //grace add in 2013.10.09
	&driver_attr_ps_data,   //grace add in 2013.10.09
	&driver_attr_type,
	&driver_attr_ps_para_index,
	&driver_attr_ps_base_value,
};

/*
static const struct attribute_group rpr521_attr_group = {
	.attrs = rpr521_attributes,
};*/

static int rpr521_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(rpr521_attributes)/sizeof(rpr521_attributes[0]));
//	APS_LOG("[%s-%d]num = %d!\n",__func__,__LINE__,num);
	printk("start create attr %s\n",__func__);
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)	
	{
		printk("start create attr %d %s\n",idx,__func__);
		if((err = driver_create_file(driver, rpr521_attributes[idx])))
		{
			printk(KERN_ERR"driver_create_file (%s) = %d\n", rpr521_attributes[idx]->attr.name, err);
			break;
		}
	} 
	printk("end create attr %s\n",__func__);
	return err;
}

static int rpr521_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(rpr521_attributes)/sizeof(rpr521_attributes[0]));
	if(driver == NULL)
	{
		return -EINVAL;	
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, rpr521_attributes[idx]);
	}
	return err;
}
/* misc device*/
/*----------------------------------------------------------------------------*/
static int rpr521_open(struct inode *inode, struct file *file)
{
	file->private_data = rpr521_i2c_client;

	if (!file->private_data)
	{
		printk("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int rpr521_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long rpr521_ioctl( struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ALS_PS_DATA *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
			printk("%s-----enable ps\n",__func__);
				if(!test_bit(CMC_BIT_ENG_PS,&obj->enable) && !test_bit(CMC_BIT_PS,&obj->enable)){
	
					if((err = rpr521_enable_ps_sensor(obj->client, 1)))
					{
						printk(KERN_ERR"enable ps fail: %d\n", err); 
						goto err_out;
					}
					//rpr521_enable_eint(obj,0);
					rpr521_enable_eint(client,1);
				}
				else{
					printk("not enable ps as CMC_BIT_ENG_PS=%d CMC_BIT_PS=%d\n",test_bit(CMC_BIT_ENG_PS,&obj->enable)?(1):(0),
						test_bit(CMC_BIT_PS,&obj->enable)?(1):(0));
				}
				set_bit(CMC_BIT_ENG_PS, &obj->enable);
                printk("%s, set ps enable bit here\n",__func__);
			}
			else
			{
				printk("%s-----disable ps\n",__func__);
				if(!test_bit(CMC_BIT_PS,&obj->enable)){
					if((err = rpr521_enable_ps_sensor(obj->client, 0)))
					{
						printk("disable ps fail: %d\n", err); 
						goto err_out;
					}
//					rpr521_enable_eint(obj,0);
				}
//				if(!test_bit(CMC_BIT_ALS,&obj->enable) && !test_bit(CMC_BIT_PS,&obj->enable) && !test_bit(CMC_BIT_PS_ACROSS,&obj->enable)){
//					rpr521_enable_als(obj->client,0);
//				}
				
				clear_bit(CMC_BIT_ENG_PS, &obj->enable);
                printk("%s, clear ps enable bit here",__func__);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_ENG_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if((err = rpr521_read_ps(obj->client, &obj->ps_data)))
			{
				goto err_out;
			}
	/*begin liujie 2011-11-16 modify*/		
			dat = rpr521_get_ps_value(client);
	       // dat = obj->ps;
	/*end liujie 2011-11-16 modify*/			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = rpr521_read_ps(obj->client, &obj->ps_data)))
			{
				goto err_out;
			}
			
			dat = obj->ps_data;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(!test_bit(CMC_BIT_ENG_ALS,&obj->enable) && !test_bit(CMC_BIT_ALS,&obj->enable)){
					if((err = rpr521_enable_als_sensor(obj->client, 1)))
					{
						printk(KERN_ERR"enable als fail: %d\n", err); 
						goto err_out;
					}
				}
				set_bit(CMC_BIT_ENG_ALS, &obj->enable);
			}
			else
			{
				if( !test_bit(CMC_BIT_ALS,&obj->enable)) {
					if((err = rpr521_enable_als_sensor(obj->client, 0)))
					{
						printk(KERN_ERR"disable als fail: %d\n", err); 
						goto err_out;
					}
				}
				clear_bit(CMC_BIT_ENG_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ENG_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = rpr521_read_als(obj->client, &obj->als_data)))
			{
				goto err_out;
			}
/*begin liujie 2011-11-16 modify*/
	//		dat = rpr521_get_als_value(obj, obj->als);			
/*end liujie 2011-11-16 modify*/	
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = rpr521_read_als(obj->client, &obj->als_data)))
			{
				goto err_out;
			}

			dat = obj->als_data;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		default:
			printk("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}

/*----------------------------------------------------------------------------*/
static struct file_operations rpr521_fops = {
	.owner = THIS_MODULE,
	.open = rpr521_open,
	.release = rpr521_release,
	.unlocked_ioctl = rpr521_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice rpr521_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &rpr521_fops,
};
/*************** Initialze Functions ******************/
static int rpr521_init_client(struct i2c_client *client)
{
    struct init_func_write_data {
        unsigned char mode_ctl;
        unsigned char psals_ctl;
        unsigned char persist;
        unsigned char reserved0;
        unsigned char reserved1;
        unsigned char reserved2;
        unsigned char reserved3;
        unsigned char reserved4;
        unsigned char reserved5;
        unsigned char intr;
        unsigned char psth_hl;
        unsigned char psth_hh;
        unsigned char psth_ll;
        unsigned char psth_lh;
        unsigned char alsth_hl;
        unsigned char alsth_hh;
        unsigned char alsth_ll;
        unsigned char alsth_lh;
    } write_data;
    int result;
    unsigned char gain;

    unsigned char mode_ctl    = PS_ALS_SET_MODE_CONTROL;
    unsigned char psals_ctl   = PS_ALS_SET_ALSPS_CONTROL;
    unsigned char persist     = PS_ALS_SET_INTR_PERSIST;
    unsigned char intr        = PS_ALS_SET_INTR;
    unsigned short psth_upper  = PS_ALS_SET_PS_TH;
    unsigned short psth_low    = PS_ALS_SET_PS_TL;
    unsigned short alsth_upper = PS_ALS_SET_ALS_TH;
    unsigned short alsth_low   = PS_ALS_SET_ALS_TL;

    struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
    /* execute software reset */
    result =  i2c_smbus_write_byte_data(client, REG_SYSTEMCONTROL, 0xC0);	//soft-reset
    if (result != 0) {
        return (result);
    }

    write_data.mode_ctl  = mode_ctl;
    write_data.psals_ctl = psals_ctl;
    write_data.persist   = persist;
    write_data.reserved0 = 0;
    write_data.reserved1 = 0;
    write_data.reserved2 = 0;
    write_data.reserved3 = 0;
    write_data.reserved4 = 0;
    write_data.reserved5 = 0;
    write_data.intr      = intr;
    write_data.psth_hl   = CONVERT_TO_BE(psth_upper) & MASK_CHAR;
    write_data.psth_hh   = CONVERT_TO_BE(psth_upper) >> 8;
    write_data.psth_ll   = CONVERT_TO_BE(psth_low) & MASK_CHAR;
    write_data.psth_lh   = CONVERT_TO_BE(psth_low) >> 8;
    write_data.alsth_hl  = CONVERT_TO_BE(alsth_upper) & MASK_CHAR;
    write_data.alsth_hh  = CONVERT_TO_BE(alsth_upper) >> 8;
    write_data.alsth_ll  = CONVERT_TO_BE(alsth_low) & MASK_CHAR;
    write_data.alsth_lh  = CONVERT_TO_BE(alsth_low) >> 8;
    result               = i2c_smbus_write_i2c_block_data(client, REG_MODECONTROL, sizeof(write_data), (unsigned char *)&write_data);

	if(result < 0)
	{
		printk(KERN_ERR "%s: i2c write fail. \n", __func__);
		return result;
	}

	gain = (psals_ctl & 0x3C) >> 2;	//gain setting values
	
	als_ps->enable_stat = mode_ctl;
	als_ps->als_time = mode_table[(mode_ctl & 0xF)].ALS;
	als_ps->ps_time = mode_table[(mode_ctl & 0xF)].PS;
	als_ps->persistence = persist;
	als_ps->ps_th_l = psth_low;
	als_ps->ps_th_h = psth_upper;
	als_ps->als_th_l = alsth_low;
	als_ps->als_th_h = alsth_upper;
	als_ps->control = psals_ctl;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = psals_ctl & 0x03;
		
	als_ps->enable_als_sensor = 0; //grace add in 2013.10.09
	als_ps->enable_ps_sensor = 0;  //grace add in 2013.10.09

#ifdef _INIT_CALIB_
	//rpr521_calibrate(client);
	//grace modify in 2014.12.18 begin
	if ((!rpr521_calibrate(client))&&calib_status)
	{
		init_ps_high = als_ps->ps_th_h;
		init_ps_low = als_ps->ps_th_l;
		init_calib_flag =1;
	}
	else
	{
		als_ps->ps_th_h_back = als_ps->ps_th_h;
		als_ps->ps_th_l_back = als_ps->ps_th_l;
	}
	//grace modify in 2014.12.18 end
#else
	als_ps->ps_th_h_back = als_ps->ps_th_h;
	als_ps->ps_th_l_back = als_ps->ps_th_l;
#endif

    return (result);
}

static int rpr_power_on(struct ALS_PS_DATA *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vddio);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vddio enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vddio);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vddio disable failed rc=%d\n", rc);
	}

	return rc;
}
static int rpr_power_init(struct ALS_PS_DATA *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vddio = regulator_get(&data->client->dev, "vddio");
	if (IS_ERR(data->vddio)) {
		rc = PTR_ERR(data->vddio);
		dev_err(&data->client->dev,
			"Regulator get failed vddio rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vddio) > 0) {
		rc = regulator_set_voltage(data->vddio, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vddio rc=%d\n", rc);
			goto reg_vddio_put;
		}
	}

	return 0;

reg_vddio_put:
	regulator_put(data->vddio);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vddio) > 0)
		regulator_set_voltage(data->vddio, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vddio);
	return 0;
}

static int rpr521_i2c_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
#define ROHM_PSALS_ALSMAX (65535)
#define ROHM_PSALS_PSMAX  (4095)

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ALS_PS_DATA *als_ps;
	//unsigned long flags;
	struct device_node *np = client->dev.of_node;
	
	int err = 0;
	int dev_id;
	printk("%s started.\n",__func__);

	//wake_lock_init(&ps_lock,WAKE_LOCK_SUSPEND,"ps wakelock");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	als_ps = kzalloc(sizeof(struct ALS_PS_DATA), GFP_KERNEL);
	if (!als_ps) {
		err = -ENOMEM;
		goto exit;
	}
	rpr521_obj = als_ps;
	als_ps->client = client;
	i2c_set_clientdata(client, als_ps);
	rpr521_i2c_client = client;

	err = rpr_power_init(als_ps, true);
	if (err) {
		dev_err(&client->dev, "power init failed");
	}
	
	err = rpr_power_on(als_ps, true);
	if (err) {
		dev_err(&client->dev, "power on failed");
	}

//	printk("enable = %x\n", als_ps->enable);

	//check whether is rpr521 or apds9930
	dev_id = i2c_smbus_read_byte_data(client, REG_MANUFACT_ID);
	if(dev_id != 0xE0){
	    kfree(als_ps);
	    return -1;
	}
	als_ps->type = 1;
	printk("%s: id(0x%x), this is rpr521!\n", __func__, dev_id);

	mutex_init(&als_ps->update_lock);

	INIT_DELAYED_WORK(&als_ps->dwork, rpr521_ps_int_work_handler);
	INIT_DELAYED_WORK(&als_ps->ps_dwork, rpr521_ps_polling_work_handler); 
	INIT_DELAYED_WORK(&als_ps->als_dwork, rpr521_als_polling_work_handler); 
	
//	printk("%s interrupt is hooked\n", __func__);

	/* Initialize the RPR400 chip */
	err = rpr521_init_client(client);
	if (err)
		goto exit_kfree;

	als_ps->ps_poll_delay = PS_ALS_SET_MIN_DELAY_TIME;	
	als_ps->als_poll_delay = PS_ALS_SET_MIN_DELAY_TIME;	

	/* Register to Input Device */
	als_ps->input_dev_als = input_allocate_device();
	if (!als_ps->input_dev_als) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device als\n", __func__);
		goto exit_free_irq;
	}

	als_ps->input_dev_ps = input_allocate_device();
	if (!als_ps->input_dev_ps) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device ps\n", __func__);
		goto exit_free_dev_als;
	}
	
	input_set_drvdata(als_ps->input_dev_ps, als_ps); //grace modified in 2013.9.24
	input_set_drvdata(als_ps->input_dev_als, als_ps); //grace modified in 2013.9.24
	
	set_bit(EV_ABS, als_ps->input_dev_als->evbit);
	set_bit(EV_REL, als_ps->input_dev_ps->evbit);
	set_bit(REL_X, als_ps->input_dev_ps->relbit);

	input_set_abs_params(als_ps->input_dev_als, ABS_X, 0, 65535, 0, 0); 
	input_set_abs_params(als_ps->input_dev_als, ABS_Y, 0, 1, 0, 0);

//	input_set_abs_params(als_ps->input_dev_ps, ABS_DISTANCE, 0, ROHM_PSALS_PSMAX, 0, 0);

	als_ps->input_dev_als->name = "light";
	als_ps->input_dev_ps->name = "proximity";
	//grace modified in 2013.3.22 begin
	als_ps->input_dev_als->id.bustype = BUS_I2C;
  als_ps->input_dev_als->dev.parent =&als_ps->client->dev;
  als_ps->input_dev_ps->id.bustype = BUS_I2C;
  als_ps->input_dev_ps->dev.parent =&als_ps->client->dev;
  //grace modified in 2013.3.22 begin


	err = input_register_device(als_ps->input_dev_als);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device als: %s\n", __func__, 
		       als_ps->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(als_ps->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device ps: %s\n", __func__, 
		       als_ps->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	/* Register sysfs hooks */
//	err = sysfs_create_group(&client->dev.kobj, &rpr521_attr_group);
	if((err = rpr521_create_attr(&rpr521_alsps_driver.driver)))
	{
		printk(KERN_ERR"create attribute err = %d\n", err);
		goto exit_unregister_dev_ps;
	}
	
	//initial ps als para
	als_ps->ps_direction = 1;
	atomic_set(&als_ps->ps_base_value, rpr521_ps_para[ps_para_num].base_value);  
	rpr521_ps_select_para(als_ps);  
	
    //regsister sensor class
    als_ps->als_cdev = sensors_light_cdev1;
	als_ps->als_cdev.sensors_enable = als_enable_set;
	als_ps->als_cdev.sensors_poll_delay = NULL;
	als_ps->als_cdev.min_delay = 50 * 1000;

	als_ps->ps_cdev = sensors_proximity_cdev1;
	als_ps->ps_cdev.sensors_enable = ps_enable_set;
	als_ps->ps_cdev.sensors_poll_delay = NULL;
	als_ps->ps_cdev.min_delay = 50 * 1000;

    err = sensors_classdev_register(&client->dev, &als_ps->als_cdev);
	if(err)
		goto exit_unregister_dev_ps;

	err = sensors_classdev_register(&client->dev, &als_ps->ps_cdev);
	if(err)
		goto exit_unregister_dev_ps;

	als_ps->irq_gpio = of_get_named_gpio_flags(np, "rpr,irq-gpio",
				0, &als_ps->irq_gpio_flags);
	if (als_ps->irq_gpio < 0)
		return als_ps->irq_gpio;
	
	if (gpio_is_valid(als_ps->irq_gpio)) {
		err = gpio_request(als_ps->irq_gpio, "rpr_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
		}

		err = gpio_direction_input(als_ps->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
		}
	}
	
	printk("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);
	if (request_irq(client->irq, rpr521_interrupt, IRQF_TRIGGER_FALLING/*IRQF_DISABLED|IRQ_TYPE_EDGE_FALLING*/,
		RPR521_DRV_NAME, (void *)client)) {
		printk("%s Could not allocate rpr521_INT !\n", __func__);
	
		goto exit_unregister_dev_ps;
	}

	irq_set_irq_wake(client->irq, 1);
	
	if(misc_register(&rpr521_device))
	{
		printk("rpr521_misc_device register failed\n");
		goto exit_unregister_dev_ps;
	}

	printk(KERN_INFO "%s: INT No. %d", __func__, client->irq);
	return 0;


exit_unregister_dev_ps:
	input_unregister_device(als_ps->input_dev_ps);	
exit_unregister_dev_als:
	printk("%s exit_unregister_dev_als:\n", __func__);
	input_unregister_device(als_ps->input_dev_als);
exit_free_dev_ps:
	input_free_device(als_ps->input_dev_ps);
exit_free_dev_als:
	input_free_device(als_ps->input_dev_als);
exit_free_irq:
	free_irq(client->irq, client);	
exit_kfree:
	kfree(als_ps);
exit:
	return err;

#undef ROHM_PSALS_ALSMAX
#undef ROHM_PSALS_PSMAX
}

static int rpr521_i2c_remove(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	input_unregister_device(als_ps->input_dev_als);
	input_unregister_device(als_ps->input_dev_ps);
	
	input_free_device(als_ps->input_dev_als);
	input_free_device(als_ps->input_dev_ps);

	free_irq(client->irq, client);

//	sysfs_remove_group(&client->dev.kobj, &rpr521_attr_group);
	rpr521_delete_attr(&rpr521_alsps_driver.driver);
	
	/* Power down the device */
	rpr521_set_enable(client, 0);
	
	misc_deregister(&rpr521_device);

	kfree(als_ps);

	return 0;
}

static int rpr521_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("%s\n", __func__);

	disable_irq(client->irq);
	//wake_unlock(&ps_lock);
	//return rpr521_set_enable(client, 0);
	return rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);//grace modify in 2014.5.7
}

static int rpr521_i2c_resume(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int tmp = PS_ALS_SET_MODE_CONTROL;
	
	printk("%s \n", __func__);
	
	enable_irq(client->irq);
	if(als_ps->enable_als_sensor == 1)
		tmp = tmp | ALS_EN;
	if(als_ps->enable_ps_sensor == 1)	
		tmp = tmp | PS_EN;
	return rpr521_set_enable(client, tmp);
}


MODULE_DEVICE_TABLE(i2c, rpr521_id);

static const struct i2c_device_id rpr521_id[] = {
	{ "rpr521", 0 },
	{ }
};
#ifdef CONFIG_OF
static struct of_device_id rpr_match_table[] = {
                { .compatible = "rpr,rpr410",},
                { },
        };
#else
#define rpr_match_table NULL
#endif 

static struct i2c_driver rpr521_i2c_driver = {
	.driver = {
		.name	= RPR521_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table =rpr_match_table,
	},
	.suspend = rpr521_i2c_suspend,
	.resume	= rpr521_i2c_resume,
	.probe	= rpr521_i2c_probe,
	.remove	= rpr521_i2c_remove,
	.id_table = rpr521_id,
};

/*----------------------------------------------------------------------------*/
static int rpr521_probe(struct platform_device *pdev) 
{
	if(i2c_add_driver(&rpr521_i2c_driver))
	{
		printk(KERN_ERR"add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int rpr521_remove(struct platform_device *pdev)
{  
	i2c_del_driver(&rpr521_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver rpr521_alsps_driver = {
	.probe      = rpr521_probe,
	.remove     = rpr521_remove,    
	.driver     = {
		.name  = "als_ps",
		.owner = THIS_MODULE,
	}
};

struct platform_device rpr521_alsps_device = {	
	.name 	= "als_ps",	
	.id		= -1,
};
/*----------------------------------------------------------------------------*/
static int __init rpr521_init(void)
{
//	return i2c_add_driver(&rpr521_driver);
	if(platform_driver_register(&rpr521_alsps_driver))
	{
		printk(KERN_ERR"failed to register rpr521_als_ps driver");
		return -ENODEV;
	}
	if(platform_device_register(&rpr521_alsps_device))
	{
		printk(KERN_ERR"failed to register rpr521_als_ps device\n");
		return -ENODEV;
	}
	
	return 0;
}

static void __exit rpr521_exit(void)
{
//	i2c_del_driver(&rpr521_driver);
	platform_driver_unregister(&rpr521_alsps_driver);
}

MODULE_AUTHOR("Andy Mi @ ROHM");
MODULE_DESCRIPTION("rpr521 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(rpr521_init);
module_exit(rpr521_exit);
