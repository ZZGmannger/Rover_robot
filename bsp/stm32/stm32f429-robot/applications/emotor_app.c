
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "mcp4725.h"

#define DBG_TAG "emotor"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define ENCODER_RIGHT_NAME   "pulse1"
#define ENCODER_LEFT_NAME    "pulse2"

#define MCP_LEFT_I2C    "i2c2"
#define MCP_RIGHT_I2C   "i2c3"

#define MOTOR_LEFT_DIR_PIN      GET_PIN(B, 12)
#define MOTOR_RIGHT_DIR_PIN     GET_PIN(B, 13)


#define ENCODE_PULSE_NUM    (600*4)
#define PULSE_DISTANCE_TRANSFORM   (100)

#define ONE_PULSE_TIME   (ENCODE_PULSE_NUM/100)


rt_thread_t motor_speed_query_thread;
rt_thread_t motor_speed_update_thread;
rt_thread_t motor_speed_acc_thread;

void motor_speed_query_entry(void* param);
void motor_speed_update_entry(void* param);
void motor_speed_acc_entry(void* param);


typedef struct
{
    float kp;
    float ki;
    float kd;

    int32_t err;
    int32_t last_err;
    int32_t prev_err;

}algo_pid_t;

typedef struct
{
    struct rt_device *encode_dev;
	mcp4725_t dac;
 
    int16_t real_speed;
    int16_t final_expect_speed;
    int8_t  acceleration;
    int16_t acc_expect_speed;

    uint32_t odometer;

    algo_pid_t pid;
}robot_motor_t;


static robot_motor_t motor_left;
static robot_motor_t motor_right;
 

int16_t motor_get_speed(robot_motor_t* motor)
{
    if(motor == RT_NULL)
    {
        LOG_E("motor get speed param err");
        return 0;
    }
    return motor->real_speed;
}

int16_t motor_set_speed(robot_motor_t* motor , int16_t speed , int8_t acc)
{
    if(motor == RT_NULL)
    {
        LOG_E("motor set speed param err");
        return 0;
    }
    motor->final_expect_speed = speed;
    motor->acceleration = acc;

    return motor->real_speed;
}

void motor_set_pid(robot_motor_t* motor , float kp , float ki , float kd)
{
    if(motor == RT_NULL)
    {
        LOG_E("motor set speed param err");
        return;
    }
    motor->pid.kp = kp;
    motor->pid.ki = ki;
    motor->pid.kd = kd;
}

int motor_init(void)
{
	int8_t ret=0;
    motor_left.encode_dev = rt_device_find(ENCODER_LEFT_NAME);
    motor_right.encode_dev = rt_device_find(ENCODER_RIGHT_NAME);
	
	if(motor_left.encode_dev  == RT_NULL || motor_right.encode_dev == RT_NULL)
    {
        LOG_E("encoder not found");
        return -RT_ERROR;
    }
	
	ret |= mcp4725_register(&motor_left.dac , MCP_LEFT_I2C);
	ret |= mcp4725_register(&motor_right.dac ,MCP_RIGHT_I2C);
	if(ret != RT_EOK)
	{
		LOG_E("mcp4725 dac not found");
		return -RT_ERROR;
	}

    if(rt_device_init(motor_left.encode_dev)!= RT_EOK || rt_device_init(motor_right.encode_dev )!= RT_EOK)
    {
        LOG_E("encoder init faild:encode");
        return -RT_ERROR;
    }

    rt_device_open(motor_left.encode_dev,RT_NULL);
    rt_device_open(motor_right.encode_dev ,RT_NULL);

    rt_pin_mode(MOTOR_LEFT_DIR_PIN  , PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR_RIGHT_DIR_PIN , PIN_MODE_OUTPUT);

    motor_speed_query_thread  = rt_thread_create("motor query" , motor_speed_query_entry , RT_NULL, 512, 1, 10);
    motor_speed_update_thread = rt_thread_create("motor update", motor_speed_update_entry, RT_NULL, 512, 2, 10);
	motor_speed_acc_thread 	  = rt_thread_create("motor acc"   , motor_speed_acc_entry, RT_NULL, 512, 3, 10);

    if(motor_speed_query_thread == RT_NULL || motor_speed_update_thread == RT_NULL)
    {
        LOG_E("motor thread create faild");
        return -RT_ERROR;
    }
    rt_thread_startup(motor_speed_query_thread);
	rt_thread_startup(motor_speed_update_thread);
	
	return RT_EOK;
}
INIT_APP_EXPORT(motor_init);


void motor_dac_output(uint8_t ch , int16_t value)
{
    if(ch == 0)
    {
        rt_pin_write(MOTOR_LEFT_DIR_PIN, value >= 0?  PIN_HIGH:PIN_LOW);
        //set out dac
		mcp4725_set_voltage(&motor_left.dac , abs(value) , 0);
    }
    else
    {
        rt_pin_write(MOTOR_RIGHT_DIR_PIN, value >= 0?  PIN_HIGH:PIN_LOW);
        //set out dac
		mcp4725_set_voltage(&motor_right.dac , abs(value) , 0);
    }
}

int16_t inc_pid(algo_pid_t* pid , int32_t real , int32_t expect)
{
    int32_t inc;
    if(pid == RT_NULL)
    {
        LOG_E("pid param err");
        return 0;
    }
    pid->err = expect - real;

    inc = pid->kp * (pid->err - pid->last_err) +
             pid->ki * (pid->err)  +
             pid->kd * (pid->err - 2*pid->last_err + pid->prev_err);

    pid->last_err = pid->err;
    pid->prev_err = pid->last_err;

    return inc;
}

void motor_speed_query_entry(void* param)
{
    int32_t count1 = 0;
	int32_t count2 =0;
    int32_t last_cnt1=0 , last_cnt2=0;
    while(1)
    {
        rt_device_read(motor_left.encode_dev , 0, &count1, 1);
        rt_device_read(motor_right.encode_dev, 0, &count2, 1);
		count1 = -count1;

        motor_left.real_speed  = 100*(count1 - last_cnt1)/ ENCODE_PULSE_NUM;
        motor_right.real_speed = 100*(count2 - last_cnt2)/ ENCODE_PULSE_NUM;

        LOG_I("motor_left  cnt is : %d , speed is %d  r/s" , count1 , motor_left.real_speed);
		LOG_I("motor_right cnt is : %d , speed is %d  r/s" , count2 , motor_right.real_speed);
		
        last_cnt1 = count1;
        last_cnt2 = count2;
 
        rt_thread_mdelay(10);
    }
}


void motor_speed_update_entry(void* param)
{
    int16_t dac_left = 0;
    int16_t dac_right = 0;

    motor_left.pid.kp = 1;
    motor_left.pid.ki = 0.1;
    motor_left.pid.kd = 0;

    motor_right.pid.kp = 1;
    motor_right.pid.ki = 0.1;
    motor_right.pid.kd = 0;

    while(1)
    {
        dac_left  += inc_pid(&motor_left.pid  , motor_left.real_speed  ,  motor_left.final_expect_speed);
        dac_right += inc_pid(&motor_right.pid , motor_right.real_speed ,  motor_right.final_expect_speed);

        motor_dac_output(0 , dac_left);
        motor_dac_output(1 , dac_right);
        rt_thread_mdelay(10);
    }
}

void motor_speed_acc_entry(void* param)
{
    while(1)
    {
        if (abs(motor_left.final_expect_speed -  motor_left.acc_expect_speed) > abs(motor_left.acceleration))
        {
            motor_left.acc_expect_speed   +=  motor_left.acceleration;
        }
        else
        {
            motor_left.acc_expect_speed = motor_left.final_expect_speed ;
        }

        if (abs(motor_right.final_expect_speed -  motor_right.acc_expect_speed) > abs(motor_right.acceleration))
        {
            motor_right.acc_expect_speed  +=  motor_right.acceleration;
        }
        else
        {
            motor_right.acc_expect_speed = motor_right.final_expect_speed ;
        }
        rt_thread_mdelay(3);
    }
}


