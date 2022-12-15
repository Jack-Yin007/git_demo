#ifndef __ATEL_SLP01_HAL_H
#define __ATEL_SLP01_HAL_H
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_saadc.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_wdt.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_uart.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader_info.h"

#include "nrf_queue.h"
#include "nrf_balloc.h"

#include "user.h"

#define VBAT_ADC_BLE               (3)
#define VIN12V_ADC_BLE             (4)
#define BLE_P005_IN1_ANALOG        (5)
#define MOD_PWRKEY                 (6)
#define BLE_WAKE_MDM               (7)
#define MDM_WAKE_BLE               (8)
#define I2C_SDA                    (9)
#define I2C_SCL                    (10)
#define BLE_UART_TXD               (11)
#define BLE_UART_RXD               (12)
#define ACC_INT1_PIN               (13)
#define P014_DC_DC_EN              (14)
#define V_MODULE_EN                (15)
#define P016_BAT_EN                (16)
#define BLE_P017_OUT1_RELAY        (17)
#define BLE_P018_OUT2_Digital      (18)
#define BLE_P019_IGNI              (19)
#define BLE_RESET_MDM              (20)
#define PWM_CH1_PIN                (22)
#define PWM_CH0_PIN                (25)
#define BLE_LDO_EN                 (26)
#define P027_CHARGE_EN             (27)
#define PIN_NOT_VALID              (0xffffffff)

#define ACC_GYRO_INT_VALID (0)
#define ACC_GYRO_INT_INVALID (1)

#define MDM_PWERER_KEY_TOGGLE_MS (600)
#define MDM_WAKE_MCU_LEVEL (0)

#define ADC_CHANNELS_CNT (2)
#define ADC_REFERENCE_VOLTAGE_FACTOR ((double)(3600.0 / 3300.0))
#define ADC_RESOLUTION_VALUE (1.1375) // 4095/3600, 12 BIT
#define ADC_MAIN_VOL_MULTIPLIER_FACTOR (0.090909) // 1 / (10 + 1)
#define ADC_MAIN_VOL_MODIFICATION_VALUE (80 + 180)
#define ADC_BUB_VOL_MULTIPLIER_FACTOR (0.698324) // 1000 / 1432
#define ADC_BUB_VOL_MODIFICATION_VALUE (0)
#define ADC_REPORT_RAW_VALUE (0)

#define BLE_DATA_CHANNEL_SUPPORT (1)
#define BLE_ADVERTISING_ENABLE_BY_DEFAULT (0)
#define BLE_CONNECTION_SUPPORT_HEARTBEAT (1)
#define BLE_BYPASS_TEST_ENABLE (0)
#define BLE_BYPASS_TEST_ONEWAY_TX (1)

#define GLASS_BREAK_EVENT_PIN_VALID_VALUE (1)

#define UART_BAUDRATE_SELECT (1)

#if (UART_BAUDRATE_SELECT == 0)
#define UART_TO_APP_BAUDRATE NRF_UART_BAUDRATE_9600
#elif (UART_BAUDRATE_SELECT == 1)
#define UART_TO_APP_BAUDRATE NRF_UART_BAUDRATE_115200
#elif (UART_BAUDRATE_SELECT == 2)
#define UART_TO_APP_BAUDRATE NRF_UART_BAUDRATE_230400
#else
#error "UART_BAUDRATE_SELECT Unknown."
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ATEL_SLP01_HAL_H */

