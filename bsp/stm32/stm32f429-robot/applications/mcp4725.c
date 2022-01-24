#include "mcp4725.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_TAG "mcp4725"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/*lint ++flb "Enter library region" */
#define MCP4725_BASE_ADDRESS    0x60        //!< MCP4725 base address

#define MCP4725_DAC_ADDRESS     0x40        //!< MCP4725 write-to-dac register
#define MCP4725_EEPROM_ADDRESS  0x60        //!< MCP4725 write-to-eeprom register

#define RDY_BIT_POS             0x07        //!< Position of RDY bit

static mcp4725_t  mcp4725_left;
static mcp4725_t  mcp4725_right;


int8_t mcp4725_register(mcp4725_t* dev , const char* name)
{
	if(dev == RT_NULL)
	{
		return -RT_ERROR;
	}
    dev->bus = (struct rt_i2c_bus_device*)rt_device_find(name);
	dev->name = name;
	if(dev->bus == RT_NULL)
	{
		return -RT_ERROR;
	}
	return RT_EOK;
}
 
int8_t mcp4725_set_voltage(mcp4725_t* dev , uint16_t val, bool write_eeprom)
{
	if(dev == RT_NULL || dev->bus == RT_NULL)
	{
		return -RT_ERROR;
	}
	val = val>0x0FFF ? 0x0fff: val;
    uint8_t reg[3] = {write_eeprom ? MCP4725_EEPROM_ADDRESS : MCP4725_DAC_ADDRESS,
                      (val>>4), (val<<4)};

    struct rt_i2c_msg msgs;
    msgs.addr = MCP4725_BASE_ADDRESS;       /* 从机地址 */
    msgs.flags = RT_I2C_WR;                /* 写标志 */
    msgs.buf = reg;                         /* 读写数据缓冲区指针　*/
    msgs.len = 3;                           /* 读写数据字节数 */

    if (rt_i2c_transfer(dev->bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}


/* defined the LED0 pin: PH12 */
#define DIR1_PIN    GET_PIN(B, 12)
#define DIR2_PIN    GET_PIN(B, 13)

static void mcp4725_init(void)
{
	rt_pin_mode(DIR1_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(DIR1_PIN , 0);
	mcp4725_register(&mcp4725_left,  "i2c2");
	mcp4725_set_voltage(&mcp4725_left , 0 , 1);

	rt_pin_mode(DIR2_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(DIR2_PIN , 1);
	mcp4725_register(&mcp4725_right,  "i2c3");
	mcp4725_set_voltage(&mcp4725_right , 0 , 1);	
}
#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(mcp4725_init,  mcp4725 init);
#endif

static void mcp4725_set_value(int argc, char **argv)
{
	rt_int16_t val;
	rt_uint8_t eeprom = 0;
		
	if(argc == 3 || argc == 4)
	{
		val = atoi(argv[2]);
		if(argc == 4)
		{
			eeprom = atoi(argv[3]);
		}
		if(atoi(argv[1]) == 0)
		{
			if(mcp4725_left.bus == RT_NULL)
			{
				LOG_W("Please probe mcp4725 first");
				return;
			}
			if(val < 0)
			{
				rt_pin_write(DIR1_PIN , 1);
				val = -val;
			}
			else
			{
				rt_pin_write(DIR1_PIN , 0);
			}
			mcp4725_set_voltage(&mcp4725_left , val , eeprom);
		}
		else
		{
			if(mcp4725_right.bus == RT_NULL)
			{
				LOG_W("Please probe mcp4725 first");
				return;
			}
			if(val < 0)
			{
				rt_pin_write(DIR2_PIN , 0);
				val = -val;
			}
			else
			{
				rt_pin_write(DIR2_PIN , 1);
			}
			mcp4725_set_voltage(&mcp4725_right , val , eeprom);
		}
	}
}
#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(mcp4725_set_value,  param: dev 0 1| val 0 4095|eeprom 0 1);
#endif






