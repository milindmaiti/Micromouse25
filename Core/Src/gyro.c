#define STM32F205RET7

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "ism330dhcx_reg.h"
#include "stm32f2xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "gyro.h"
#include "i2c.h"

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME          10 //ms
#define SENSOR_BUS hi2c1

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
float_t acceleration_mg[3];
float_t angular_rate_mdps[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
stmdev_ctx_t dev_ctx;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/

void ism330dhcx_init(void){
	/* Initialize mems driver interface */
	  dev_ctx.write_reg = platform_write;
	  dev_ctx.read_reg = platform_read;
	  dev_ctx.mdelay = platform_delay;
	  dev_ctx.handle = &SENSOR_BUS;
	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);
	/* Check device ID */
	ism330dhcx_device_id_get(&dev_ctx, &whoamI);

	if (whoamI != ISM330DHCX_ID)
	  while (1);


	/* Restore default configuration */
	  ism330dhcx_reset_set(&dev_ctx, PROPERTY_ENABLE);

	  do {
	    ism330dhcx_reset_get(&dev_ctx, &rst);
	  } while (rst);

	  /* Start device configuration. */
	  ism330dhcx_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
	  /* Enable Block Data Update */
	  ism330dhcx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	  /* Set Output Data Rate */
	  ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_12Hz5);
	  ism330dhcx_gy_data_rate_set(&dev_ctx, ISM330DHCX_GY_ODR_12Hz5);
	  /* Set full scale */
	  ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_2g);
	  ism330dhcx_gy_full_scale_set(&dev_ctx, ISM330DHCX_2000dps);
	  /* Configure filtering chain(No aux interface)
	   *
	   * Accelerometer - LPF1 + LPF2 path
	   */
	  ism330dhcx_xl_hp_path_on_out_set(&dev_ctx, ISM330DHCX_LP_ODR_DIV_100);
	  ism330dhcx_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

}
void ism330dhcx_read_data(void)
{
    uint8_t reg;
    /* Read output only if new xl value is available */
    ism330dhcx_xl_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read acceleration field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      ism330dhcx_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] =
        ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[2]);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    ism330dhcx_gy_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read angular rate field data */
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      ism330dhcx_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      angular_rate_mdps[0] =
        ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
      angular_rate_mdps[1] =
        ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
      angular_rate_mdps[2] =
        ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, ISM330DHCX_I2C_ADD_H, reg,
                      I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, ISM330DHCX_I2C_ADD_H, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&huart4, tx_buffer, len, 1000);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}
