
#ifndef MCP4725_H
#define MCP4725_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @file
* @brief MCP4725 digital DAC driver.
*
*
* @defgroup mcp4725 MCP4725 digital DAC driver
* @{
* @ingroup ext_drivers
* @brief MCP4725 digital DAC driver.
*/
typedef struct
{
	const char* name;
	struct rt_i2c_bus_device *bus;
}mcp4725_t;	


/**
 * @brief Function for setting new value to DAC.
 *
 * @param[in] val               12-bit value. Base on it voltage is set (Vout = (val/4095) * Vcc).
 * @param[in] write_eeprom      Defines if value will be written to DAC only or to EEPROM memmory also.
 *
 * @return
 */
int8_t mcp4725_register(mcp4725_t* dev , const char* name);
int8_t mcp4725_set_voltage(mcp4725_t* dev , uint16_t val, bool write_eeprom);

/**
 *@}
 **/

/*lint --flb "Leave library region" */

#ifdef __cplusplus
}
#endif

#endif //MCP4725_H
