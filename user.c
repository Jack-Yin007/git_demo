/*
 * user.c
 *
 *  Created on: Dec 12, 2015
 *      Author: J
 */

#include <string.h>          /* For memcpy definition */
#include <stdio.h>

#include "user.h"
#include "version.h"

#include "platform_hal_drv.h"
#include "ble_data_c.h"
//#include "ble_data.h"
//#include "acc.h"
#include "rtcclock.h"
#include "mp2762a_drv.h"
#include "nrf_temp.h"
#include "acc_simba_lis2dh12.h"
#include "one_wire_atel.h"
#include "solar_mppt.h"
#include "mux_mcu.h"
#include "peer_manager.h"
#include "ble_aus_c.h"

// @ADC_note
// Example
// adc_vol = (adc_val / ADC_MAX) * Vref / Gain	// This formula comes from nRF52840 Datasheet 
//         = (adc_val / 0x400) * 0.6/(1/6)
//         = (adc_val / 0x400) * 3.6 V
//         = (adc_val / 0x400) * 3600 mV

#define ADC_MAX				(1 << ((SAADC_CONFIG_RESOLUTION)*2 + 8))		// 0x100, 8-bit; 0x400, 10-bit; 0x1000, 12-bit; 0x4000, 14-bit
#define ADC_FACTOR			3600 		// ADC factor, see @ADC_note for explanation

extern uint32_t egpio_config[];
extern uint32_t gresetCause;

/* Define Slave Address  ---------------------------------------------------*/
typedef enum {RESET = 0, SET = !RESET} BitStatus;

static uint8    gFlashLED               = 0;

/*static*/ uint32_t gBWD = 3600;//900;//3600; // LM-7. Note: recover the value to 3600, 20201027, QGH. Refer to @Watchdog_time_change for more info.
static uint16_t gcount = 0;
/*static */uint32_t count1sec = 0;
static uint32_t count_recharge_check = 0; // count for restarting charge check
static uint32_t count_not_recharge_check = 0; // count for not restarting charge check
static bool recharge_start = false; // restarting charge check
//static bool recharge_start = true; // restarting charge check
static uint32_t count_solar_p = 0; // count for solar power
static double sum_solar_p = 0.0;   // sum for solar power of SOLAR_GET_POWER_COUNT
static double aver_solar_p = 0.0;  // average for solar power of SOLAR_GET_POWER_COUNT

static bool eventIReadyFlag = false;

static uint8    accPollingEnableTimer   = 0;

//uint16_t model;
uint32_t model;
uint16_t pu;
uint16_t rst;
uint16_t wdfired;

gpio_conf monet_conf;
gpio_data monet_gpio = {0};
monet_struct monet_data = {{(IoCmdState)0}};

device_reset_info_t reset_info = {0};

uint8_t     gAdcBatCounter          = 0;

paired_ble_info_t paired_ble_info[BLE_CHANNEL_NUM_MAX] = {0};

ble_link_target_t ble_link_target = {0};

uint32_t io_tx_count = 0;

#define GPIO_GET(index) (bool)((monet_gpio.gpiolevel >>index)&0x1)
#define GPIO_SET(index, s)  monet_gpio.gpiolevel = (s>0) ? ((1<<index)|monet_gpio.gpiolevel): (monet_gpio.gpiolevel & (~(1<<index)))

static void ConfigPowerMasks(uint8_t* pParam, uint8_t Length);
void monet_Icommand(void);
void monet_Vcommand(void);

extern bool ble_aus_ready_state_get_c(void);

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/
void IncreaseCount(uint16_t * value)
{
   *value += 1;
}

void disableEventIReadyFlag(void) {
    eventIReadyFlag = false;
}

bool mp_isEventIReadyFlag(void) {
    return eventIReadyFlag;
}

void MCU_TurnOff_MDM(void)
{
    monet_data.bbofftime = count1sec;
    monet_data.phonePowerOn = 0;
    monet_data.SleepState = SLEEP_HIBERNATE;
    monet_data.SleepStateChange = 1;

    if (monet_data.uartMdmTXDEnabled != 0)
    {
        while (atel_io_queue_process()); 	// Sendout all data in queue.
        pf_uart_mdm_deinit();
    }

    pf_mdm_pwr_ctrl(false);
	nrf_delay_ms(2);	// Delay is needed. In P2 board, resistor R226 connected with VDD_MDM_EN is replaced by a capacitor. 
						// The capacitor is for BLE DFU feature. If VDD_MDM_EN is high, BLE reset would not influence its state because of the capacitor. 
						// But when VDD_MDM_EN is set to low, delay is needed to release capacitor's power and guarantee low level state.
	pf_mdm_pwr_deinit();
		
    MCU_Sleep_APP();

	if (monet_data.BUBX != 1)	// Shipping mode. Avoid repeat de-initialization when in Shipping mode which would make BLE crash. Mainly related with advertising_stop()
		pf_cfg_before_hibernation();
}

void MCU_TurnOn_MDM(void)
{
    pf_mdm_pwr_ctrl(true);
    if (monet_data.resetfromDFU)
    {
        pf_mdm_pwr_key_ctrl(false);
    }
    else
    {
        monet_gpio.counter[GPIO_MDM_PWR_KEY] = (MDM_PWKEY_TOGGLE_ON_MS + TIME_UNIT - 1) / TIME_UNIT;
    }
//    if (monet_data.firstPowerUp) {
//        monet_data.firstPowerUp = 0;
//        monet_gpio.WDtimer = 3600; // LM-46
//    } else {
//        monet_gpio.WDtimer = 120;  // LM-29 Make sure to reset WD to align with any wakeup event
//    }
    monet_data.phonePowerOn = 1;
	if (monet_data.resetfromDFU)
		monet_data.phoneLive = INT_MASK_ON;
	else
		monet_data.phoneLive = 0;
}

uint8_t isMDMWakingMCU(void) {
//    return (pf_gpio_read(GPIO_MDM_WAKE_BLE) == MDM_WAKE_MCU_LEVEL) ? 1 : 0;
    return 0;
}

void MCU_Wakeup_MDM(void)
{
//    // TODO: verify whether the setting is correct
//    pf_gpio_write(GPIO_BLE_WAKE_MDM, 0);
//    monet_gpio.Intstatus |= MASK_FOR_BIT(INT_WAKEUP); // LM-32 Indicate this is a wakeup
}

void MCU_Sleep_MDM(void)
{
//    // TODO: verify whether the setting is correct
//    pf_gpio_write(GPIO_BLE_WAKE_MDM, 1);
//    monet_data.bbofftime = count1sec;
//    disableEventIReadyFlag(); // LM-25
}

void MCU_Sleep_APP(void)
{
//    configGPIO(GPIO_BLE_SLEEP_APP, monet_conf.gConf[GPIO_BLE_SLEEP_APP].status);
//    // TODO: SET GPIO_BLE_SLEEP_APP LOW(To keep APP sleep, this pin should stay low)
//    configGPIO(GPIO_BLE_SLEEP_APP1, monet_conf.gConf[GPIO_BLE_SLEEP_APP1].status);

//    monet_data.appActive = 0;
}

//void MCU_Wakeup_APP(void)
//{
//    if ((monet_data.uartMdmTXDEnabled == 0) && (monet_data.uartToBeInit == 0))
//    {
//        monet_data.uartToBeInit = 1;
//        monet_data.uartTickCount = 0;
//    }
//}

// Code moved from Simba code
// But name is conflict with existing code
// This function is mainly for Accelerometer init
void init_config2(void)
{
	config_data.adur[0] = 0;			// Unused
	config_data.adur[1] = 0;			// Duration
	config_data.affcount = 3;			// Unused
	config_data.affinter = 60;			// ?
	config_data.ar[0] = 0;				// Unused
	config_data.ar[1] = LIS_ODR_10Hz;	// Default 10 Hz
	config_data.at[0]= 0;				// Unused
	config_data.at[1]= 10;				// Threshold (0-127)
	config_data.phoneV = 2000;			// Unused
	
	// TODO. EEPROM operation to flash storage
	
	config_data.phonetimer = 0xFFFFFF;
//	value = GetConfiguration(PARAM_TIME_INTERVAL);
//	if(value != 0xFFFFFFFF) config_data.phonetimer = value;
//	config_data.phonetimer = 0xFFFFFF;

//	value = GetConfiguration(PARAM_ACC_DUR0);
//	if(value != 0xFFFFFFFF) config_data.adur[0] = value;

//	value = GetConfiguration(PARAM_ACC_DUR1);
//	if(value != 0xFFFFFFFF) config_data.adur[1] = value;

//	value = GetConfiguration(PARAM_ACC_AFFCOUNT);
//	if(value != 0xFFFFFFFF) config_data.affcount = value;

//	value = GetConfiguration(PARAM_ACC_AFFINTER);
//	if(value != 0xFFFFFFFF) config_data.affinter = value;

//	value = GetConfiguration(PARAM_ACC_AR0);
//	if(value != 0xFFFFFFFF) config_data.ar[0] = value;

//	value = GetConfiguration(PARAM_ACC_AR1);
//	if(value != 0xFFFFFFFF) config_data.ar[1] = value;

//	value = GetConfiguration(PARAM_ACC_AT0);
//	if(value != 0xFFFFFFFF) config_data.at[0] = value;

//	value = GetConfiguration(PARAM_ACC_AT1);
//	if(value != 0xFFFFFFFF) config_data.at[1] = value;

//	value = GetConfiguration(PARAM_ACC_PHONEV);
//	if(value != 0xFFFFFFFF) config_data.phoneV = value;
	
    config_data.bShippingMode = 0;
//    value = GetConfiguration(PARAM_SHIPPING_MODE);
//    if (value != SHIPPING_MODE_OFF) config_data.bShippingMode = 1;

//    config_data.bAllowPowerKey = 0;
    config_data.bAllowPowerKey = 1;
//    value = GetConfiguration(PARAM_ALLOW_POWERKEY);
//    if (value != BLOCK_POWER_KEY) {
//        config_data.bAllowPowerKey = 1;
//    } else { // SIMBAMCU-28
//        config_data.resetAfterUpdate = 1;
//        // Reset the one-time powerkey block
//        SetConfiguration(PARAM_ALLOW_POWERKEY, ALLOW_POWER_KEY); // SIMBAMCU-34
//    }

//    // PUMAMCU-161, mcu reset
//    value = GetConfiguration(PARAM_MCU_RST);
//    if (value != MCU_RST_ON) {
//        config_data.bmcuRst = 1;
//    } else {
//        config_data.bmcuRst = 0;
//    }
}

void mode_selection_subproc(void)
{
	uint32_t input_vol = 0;
	double input_cur = 0.0;
	double input_p = 0.0;
	int32_t input_vol_limt_orig = 8500;
//	bool charger_en = false;
	
//	charger_en = is_charger_power_on();	// Original state of the Charger
	if (is_charger_power_on() == true)
		input_vol_limt_orig = mp2762a_input_vol_limit_get();	// Read the original voltage setting
	setChargerOn();
	mp2762a_input_vol_limit_set(MP2762A_INPUT_VOL_LIMIT_SOL);
	nrf_delay_ms(300);	// Leave time for Charger chip to bring new configuration into effect
	input_vol = mp2762a_input_vol_get();
	input_cur = mp2762a_input_cur_get();
	input_p = input_vol * input_cur;
	NRF_LOG_RAW_INFO("mode_selection_subproc(), V %u mV, I %u mA, P %u uW\r", input_vol, (uint32_t)input_cur, (uint32_t)input_p);
	NRF_LOG_FLUSH();
	if (input_p > SOLAR_POWER_LIMIT)
	{
		solar_chg_mode_set(SOLAR_CHG_MODE1);
//		if (charger_en == true)
//		{
			mp2762a_input_vol_limit_set(input_vol_limt_orig); // Recover Input Voltage Limit setting
//		}
//		else
//			setChargerOff();
		NRF_LOG_RAW_INFO("mode_selection_subproc(), set to Mode 1\r");
		NRF_LOG_FLUSH();
		timer_solar_chg_mode_stop();	// Stop the periodic timer for checking mode switch
	}
	else
	{
		mp2762a_input_vol_limit_set(input_vol_limt_orig);
		setChargerOff();
		solar_chg_mode_set(SOLAR_CHG_MODE2);
		timer_solar_chg_mode_restart();
		NRF_LOG_RAW_INFO("mode_selection_subproc(), set to Mode 2\r");
		NRF_LOG_FLUSH();
	}
}

void adc_conv_force(void)
{
	pf_gpio_write(GPIO_BAT_ADC_TMP_EN, 1);	// Enable ADC power switch. This pin would be set low after next second ADC conversion finished, so setting low here is not needed
	nrf_delay_ms(20);	// Give some time for voltage of pin VBAT_ADC to reach real level after BAT_ADC_TMP_EN is set high.
	nrf_delay_ms(30);	// Add extra time to measure solar voltage
	atel_adc_converion();
}

bool flag_only_solar_check = false;
bool flag_s_v_check = false;
const uint32_t s_v_check_debounce_limit = 3;
uint32_t s_v_check_debounce = s_v_check_debounce_limit;
bool flag_low_solar_vol = false;

bool is_solar_vol_low(void)
{
	return flag_low_solar_vol;
}

// This function is designed to be called in a 1 s periodic function
void solar_chg_mode_proc(void)
{
	if (flag_only_solar_check == true)
	{
		if (only_solar_power() == true)
		{
			flag_only_solar_check = false;
			solar_chg_mode_select(FUNC_JUMP_POINT_2);
		}
	}
	
	if (flag_s_v_check == true)
	{
		s_v_check_debounce--;
		if (!s_v_check_debounce)
		{
			s_v_check_debounce = s_v_check_debounce_limit;
			flag_s_v_check = false;
			solar_chg_mode_select(FUNC_JUMP_POINT_1);
		}
	}
}

void solar_chg_mode_select(uint8_t jump_point)
{
	uint16_t sol_vol = 0;
	uint32_t input_vol = 0;
	double input_cur = 0.0;
	double input_p = 0.0;
	int32_t input_vol_limt_default = 8500;

	// Parameters initialization
	flag_only_solar_check = false;
	flag_s_v_check = false;
	s_v_check_debounce = s_v_check_debounce_limit;
	timer_solar_chg_mode_stop();
	flag_low_solar_vol = false;
	
	NRF_LOG_RAW_INFO("solar_chg_mode_select(), jump point %u\r", jump_point);
	
	switch (jump_point)
	{
		case FUNC_JUMP_POINT_0:
			goto func_jump_point_0;
//			break;	// Comment this line to avoid warning
		case FUNC_JUMP_POINT_1:
			goto func_jump_point_1;
//			break;	// Comment this line to avoid warning
		case FUNC_JUMP_POINT_2:
			goto func_jump_point_2;
//			break;	// Comment this line to avoid warning
		case FUNC_JUMP_POINT_3:
			goto func_jump_point_3;
//			break;	// Comment this line to avoid warning
		default:
			break;
	}
	
func_jump_point_0:
	setChargerOff();
func_jump_point_1:
	solar_chg_mode_set(SOLAR_CHG_MODE1);
	adc_conv_force();
	if (only_solar_power() == true)
	{
func_jump_point_2:
		setChargerOff();
		adc_conv_force();
		sol_vol = adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR);
		NRF_LOG_RAW_INFO("solar_chg_mode_select(), sol V %u mV\r", sol_vol);
		if (sol_vol > VOL_LIMIT_S_CHG_MODE_SEL)
		{
			setChargerOn();
			mp2762a_input_vol_limit_set(MP2762A_INPUT_VOL_LIMIT_SOL);
			nrf_delay_ms(200);	// Leave time for Charger chip to bring new configuration into effect
			input_vol = mp2762a_input_vol_get();
			input_cur = mp2762a_input_cur_get();
			input_p = input_vol * input_cur;
			NRF_LOG_RAW_INFO("solar_chg_mode_select(), V %u mV, I %u mA, P %u uW\r", input_vol, (uint32_t)input_cur, (uint32_t)input_p);
			if (input_p > SOLAR_POWER_LIMIT)
			{
				mp2762a_input_vol_limit_set(input_vol_limt_default); // Recover Input Voltage Limit setting
				timer_solar_chg_mode_stop();	// Stop the periodic timer for checking mode switch
				solar_chg_mode_set(SOLAR_CHG_MODE1);
				NRF_LOG_RAW_INFO("solar_chg_mode_select(), set to Mode 1\r");
			}
			else
			{
func_jump_point_3:
				mp2762a_input_vol_limit_set(input_vol_limt_default);
				setChargerOff();
				solar_chg_mode_set(SOLAR_CHG_MODE2);
				timer_solar_chg_mode_restart();
				NRF_LOG_RAW_INFO("solar_chg_mode_select(), set to Mode 2\r");
			}
		}	// if (sol_vol > VOL_LIMIT_S_CHG_MODE_SEL)
		else
		{
			NRF_LOG_RAW_INFO("solar_chg_mode_select(), solar vol <= 11 V\r");
			s_v_check_debounce = s_v_check_debounce_limit;
			flag_s_v_check = true;
			flag_low_solar_vol = true;
		}
	}	// if (only_solar_power() == true)
	else
	{
		NRF_LOG_RAW_INFO("solar_chg_mode_select(), !only_solar_power()\r");
		setChargerOn();
		solar_chg_mode_set(SOLAR_CHG_MODE1);
		flag_only_solar_check = true;
	}
	NRF_LOG_FLUSH();
}

void mode_selection_proc(void)
{
	uint16_t sol_vol = 0;
	
	setChargerOff();
	solar_chg_mode_set(SOLAR_CHG_MODE1);	// Set to Mode 1. Solar voltage read is not correct when in Mode 2, because Solar power is directly connected to battery.
	pf_gpio_write(GPIO_BAT_ADC_TMP_EN, 1);	// Enable ADC power switch. This pin would be set low after next second ADC conversion finished, so setting low here is not needed
	nrf_delay_ms(20);	// Give some time for voltage of pin VBAT_ADC to reach real level after BAT_ADC_TMP_EN is set high.
	nrf_delay_ms(30);	// Add extra time to measure solar voltage
	atel_adc_converion();
	sol_vol = adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR);
	NRF_LOG_RAW_INFO("mode_selection_proc(), sol V %u mV\r", sol_vol);
	if (sol_vol > VOL_LIMIT_S_CHG_MODE_SEL)
	{
		mode_selection_subproc();
	}
	else
	{
		NRF_LOG_RAW_INFO("mode_selection_proc(), set to Mode2\r");
		solar_chg_mode_set(SOLAR_CHG_MODE2);
		timer_solar_chg_mode_restart();
	}
	NRF_LOG_FLUSH();
}

void InitApp(uint8_t resetfromDFU)
{
    memset(&monet_data, 0, sizeof(monet_data));	// Init monet_data to 0
	memset(&config_data, 0, sizeof(config_data));
	
    monet_data.firstPowerUp = 1;
    monet_data.OneWireDisable = 1; // Wait for App to enable

    monet_data.resetfromDFU = resetfromDFU;

    gAdcBatCounter          = 0;

    /* Set GPIOs first  */ // LM-34
    init_config();
    gpio_init();
    /* Setup analog functionality and port direction */
    memset(&monet_data.txQueueU1, 0, sizeof(atel_ring_buff_t));
#ifdef USE_TILT
    ARS_QUEUE_INIT(ars_accQ, ARS_ACC_Q_SIZE);
#endif
    model = 0x4D50;
	gresetCause = 0;
	gFlashLED = 0;
    monet_data.AdcMain = 0;
    monet_data.AdcAux = 0;
    monet_data.AdcSolar = 0;
    monet_data.AdcBackup = 0;
    monet_data.MainState = MAIN_UNKNOWN;
    monet_data.AdcBatC = BUB_CRITICAL_THRESHOLD;
	accPollingEnableTimer = 0;

    //Init the state machines
    monet_data.iorxframe.state = IO_WAIT_FOR_DOLLAR;
	monet_data.phoneLive = 0;
	monet_data.V3PowerOn = 0;
	monet_data.wakeBBMode = INT_MASK_WK;
	monet_data.bNeedReport = 0;
	monet_data.ResetBaseBandDelay = 0;
	monet_data.ResetBaseBandCounter = 0;

	monet_data.bBattMarkedCritical = false; // SIMBAMCU-7

	monet_data.sleepmode = 0; // SIMBAMCU-29

//    ble_connection_channel_init();

//    monet_data.glass_break.debounce = CAMERA_GLASS_BREAK_DEBOUNCE_INVALID;
//    monet_data.glass_break.pic_num = 0;
//    monet_data.glass_break.level = 0;
//    monet_data.glass_break_detected = 0;
//    monet_data.glass_break_pin_index = 0xff;


    /* Initialize peripherals */
	init_config2();
    nrf_delay_ms(20);	// Give some time for voltage of pin VBAT_ADC to reach real level after BAT_ADC_TMP_EN is set high.
	setChargerOff();	// Charger should be disable before ADC, or the solar ADC value would not be correct
    pf_adc_init();
    pf_adc_start();
    nrf_delay_ms(10);
	pf_adc_poll_finish();
	
//	pf_uart_peri_init(9, 0);	// Baudrate 115200, stop-bit 1
	pf_uart_peri_init(5, 0);	// Baudrate 9600, stop-bit 1

//	atel_ImuDataInit();
	pf_i2c_init();
//	pf_imu_reset();
	nrf_delay_ms(50);
	pf_imu_init();
//	setChargerOn();
//	mp2762a_init();
//	if (only_solar_power() == true)
//		mode_selection_proc();
	solar_chg_mode_select(FUNC_JUMP_POINT_0);
	ion_accRegInit2(config_data.ar[1], config_data.at[1], config_data.adur[1], 0, 0); //motion detection

    // Initialize the 1w bridge
    OWInit();

    pic_turnOnBaseband();

    monet_data.resetfromDFU = 0;
    monet_gpio.WDtimer = gBWD;

	IncreaseCount(&pu);
	monet_data.sysTickUnit = TIME_UNIT;
	pf_systick_start(monet_data.sysTickUnit);

	pf_wdt_init();

	monet_gpio.Intstatus |= MASK_FOR_BIT(INT_POWERUP); // Added power up flag to watch for Power up resets
	if (wdfired) {
		wdfired = 0;
		monet_gpio.Intstatus |= MASK_FOR_BIT(INT_WDFIRED); // Added WD flag to watch for WD resets
	}
	
	monet_data.AccMode = 1;
    monet_data.AccDataAvailable = 0;
	monet_data.AccData.motionMode  = MOTION_STATE_NONE;
    monet_data.AccData.motionState = MOTION_STATE_NONE;
	monet_data.AccTestMode = ACC_TEST_DEFAULT;
	monet_data.bActivated = 1;
	
#ifdef USE_TILT // PUMAMCU-136
	ion_accStart(2, 2);
#endif
//   watchdog_init();
    monet_Vcommand();
}


void init_config(void)
{
//    setdefaultConfig(pf_gpio_read(GPIO_P016_BAT_EN));
    setdefaultConfig();
	
    // If not first start up, keep the value of monet_gpio.Intstatus.
    memset(&monet_gpio.WDtimer, 0, sizeof(monet_gpio) - ((uint32_t)(&monet_gpio.WDtimer) - (uint32_t)(&monet_gpio.Intstatus)));
}

//void setdefaultConfig(uint8_t bat_en)
void setdefaultConfig(void)
{
    uint32_t i;
    uint8_t status = PIN_STATUS(1,0,1,0);

    for(i=0; i< NUM_OF_GPIO_PINS; i++)
    {
        monet_conf.gConf[i].status = status;
        monet_conf.gConf[i].Reload = 0;
    }

    monet_conf.gConf[GPIO_BLE_GPIO1].status             	= (uint8_t)PIN_STATUS(1, 1, 1, 0);
    monet_conf.gConf[GPIO_BLE_GPIO2].status             	= (uint8_t)PIN_STATUS(1, 0, 1, 0);
    monet_conf.gConf[GPIO_BLE_GPIO3].status             	= (uint8_t)PIN_STATUS(1, 1, 1, 0);
    monet_conf.gConf[GPIO_BLE_Tamper].status             	= (uint8_t)PIN_STATUS(1, 0, 1, 0);
//	monet_conf.gConf[GPIO_BLE_RELAY].status             	= (uint8_t)PIN_STATUS(0, 1, 1, 1);
    monet_conf.gConf[GPIO_BLE_RELAY].status             	= (uint8_t)PIN_STATUS(1, 0, 1, 0);
    monet_conf.gConf[GPIO_RS232_EN].status                  = (uint8_t)PIN_STATUS(0, 1, 1, 1);
    monet_conf.gConf[GPIO_MDM_PWR_KEY].status             	= (uint8_t)PIN_STATUS(0, 1, 1, 0);
    monet_conf.gConf[GPIO_BLE_CAN_PWR_EN].status            = (uint8_t)PIN_STATUS(0, 1, 1, 1);
//	monet_conf.gConf[GPIO_CS_12V_EN].status                 = (uint8_t)PIN_STATUS(0, 1, 1, 0);
//	monet_conf.gConf[GPIO_CS_12V_EN].status                 = (uint8_t)PIN_STATUS(0, 1, 1, 1);
	monet_conf.gConf[GPIO_CS_12V_EN].status                 = (uint8_t)PIN_STATUS(0, 1, 1, 0);	// Set it low by default
    monet_conf.gConf[GPIO_Hall_Tamper].status               = (uint8_t)PIN_STATUS(1, 0, 1, 0);
//	monet_conf.gConf[GPIO_VBAT_Heating_Power_EN].status     = (uint8_t)PIN_STATUS(0, 1, 1, 0);
//	monet_conf.gConf[GPIO_CAN_INT].status                   = (uint8_t)PIN_STATUS(1, 0, 1, 0);
//	monet_conf.gConf[GPIO_CHRG_PROCHOT].status              = (uint8_t)PIN_STATUS(1, 0, 1, 0);
    monet_conf.gConf[GPIO_CHRG_SLEEP_EN].status             = (uint8_t)PIN_STATUS(0, 1, 1, 0);
    monet_conf.gConf[GPIO_CHRG_INT].status                  = (uint8_t)PIN_STATUS(1, 0, 1, 0);
#if (HW_VER >= HW_VER_P3)
	monet_conf.gConf[GPIO_SOLAR_CHARGE_SWITCH].status       = (uint8_t)PIN_STATUS(0, 1, 1, 0);
#else
    monet_conf.gConf[GPIO_CHRG_ACOK].status                 = (uint8_t)PIN_STATUS(1, 0, 1, 0);
#endif
    monet_conf.gConf[GPIO_ST_MCU_PWR_EN].status             = (uint8_t)PIN_STATUS(0, 1, 1, 1);
    monet_conf.gConf[GPIO_CS_3V3_EN].status                 = (uint8_t)PIN_STATUS(0, 1, 1, 0);
    monet_conf.gConf[GPIO_VDD_MDM_EN].status                = (uint8_t)PIN_STATUS(0, 1, 1, 1);
    monet_conf.gConf[GPIO_DC_DC_9V5_EN].status              = (uint8_t)PIN_STATUS(0, 1, 1, 0);
//	monet_conf.gConf[GPIO_CS_MERCREBOOT].status             = (uint8_t)PIN_STATUS(0, 1, 1, 1);
    monet_conf.gConf[GPIO_CS_nRST].status                   = (uint8_t)PIN_STATUS(0, 1, 1, 1);
//	monet_conf.gConf[GPIO_TMP_INT].status                   = (uint8_t)PIN_STATUS(1, 0, 1, 0);
//	monet_conf.gConf[GPIO_G_SENSOR_INT].status              = (uint8_t)PIN_STATUS(1, 0, 1, 0);
    monet_conf.gConf[GPIO_G_SENSOR_INT].status              = (uint8_t)PIN_STATUS(1, 1, 1, 0);	// Enable pullup resistor
    monet_conf.gConf[GPIO_LED_ORANGE].status                = (uint8_t)PIN_STATUS(0, 1, 1, 1);	// Set low to turn on LED, set high to turn off LED
    monet_conf.gConf[GPIO_LED_GREEN].status                 = (uint8_t)PIN_STATUS(0, 1, 1, 1);	// Set low to turn on LED, set high to turn off LED
    monet_conf.gConf[GPIO_LED_RED].status                   = (uint8_t)PIN_STATUS(0, 1, 1, 1);	// Set low to turn on LED, set high to turn off LED
    monet_conf.gConf[GPIO_ONE_BUS_SLPZ].status              = (uint8_t)PIN_STATUS(0, 1, 1, 1);
//	monet_conf.gConf[GPIO_ST_UART1_TO_UART2_EN].status      = (uint8_t)PIN_STATUS(0, 1, 1, 0);
//	monet_conf.gConf[GPIO_ST_UART1_TO_UART3_EN].status      = (uint8_t)PIN_STATUS(0, 1, 1, 0);
//	monet_conf.gConf[GPIO_ST_UART2_TO_UART3_EN].status      = (uint8_t)PIN_STATUS(0, 1, 1, 0);
    monet_conf.gConf[GPIO_ST_UART1_TO_UART2_EN].status      = (uint8_t)PIN_STATUS(1, 0, 1, 0);	// These 3 pins connected with STM32 are now set to input mode
    monet_conf.gConf[GPIO_ST_UART1_TO_UART3_EN].status      = (uint8_t)PIN_STATUS(1, 0, 1, 0);
    monet_conf.gConf[GPIO_ST_UART2_TO_UART3_EN].status      = (uint8_t)PIN_STATUS(1, 0, 1, 0);
    monet_conf.gConf[GPIO_BAT_ADC_TMP_EN].status            = (uint8_t)PIN_STATUS(0, 1, 1, 1);
    monet_conf.gConf[GPIO_CHRGIN_PWR_EN].status             = (uint8_t)PIN_STATUS(0, 1, 1, 0);
    monet_conf.gConf[GPIO_ONE_BUS_SLPZ].status              = (uint8_t)PIN_STATUS(0, 1, 1, 1); // NALAMCU-29

    monet_conf.IntPin = GPIO_TO_INDEX(GPIO_NONE);
    monet_conf.WD.Pin = GPIO_TO_INDEX(GPIO_NONE);

    monet_conf.WD.Reload = gBWD; //in second
    monet_gpio.WDtimer = monet_conf.WD.Reload;
    monet_gpio.WDflag = 0;
}

void gpio_init_pin(uint8_t pin_index)
{
	if ((pin_index != GPIO_VDD_MDM_EN) &&
		(pin_index != GPIO_MDM_PWR_KEY))
	{
		configGPIO(pin_index, monet_conf.gConf[pin_index].status);
		if (pin_index == GPIO_ST_MCU_PWR_EN)
		{
			configGPIO(GPIO_ST_UART1_TO_UART2_EN, PIN_STATUS(0, 1, 1, 0));	// Configure the 3 pins as output low to ensure MUX MCU bootup state. 20201018
			configGPIO(GPIO_ST_UART1_TO_UART3_EN, PIN_STATUS(0, 1, 1, 0));	// They would quickly be configured as input when loop ends.
			configGPIO(GPIO_ST_UART2_TO_UART3_EN, PIN_STATUS(0, 1, 1, 0));
			nrf_gpio_pin_clear(ST_UART1_TO_UART2_EN);
			nrf_gpio_pin_clear(ST_UART1_TO_UART3_EN);
			nrf_gpio_pin_clear(ST_UART2_TO_UART3_EN);
			nrf_delay_ms(5);
		}
	}
	monet_gpio.counter[pin_index] = monet_conf.gConf[pin_index].Reload;
	if((monet_conf.gConf[pin_index].status & GPIO_DIRECTION) == DIRECTION_OUT) {
		SetGPIOOutput(pin_index, (bool)((monet_conf.gConf[pin_index].status & GPIO_SET_HIGH)>0));
	} else {
		GPIO_SET(pin_index, pf_gpio_read(pin_index));
	}
}

void gpio_init(void)
{
    uint8_t i;

    for(i = 0; i< NUM_OF_GPIO; i++)
    {
//        NRF_LOG_RAW_INFO("%s: Index:%d, NUM_OF_GPIO:%d\r\n", __func__, i, NUM_OF_GPIO);
//        NRF_LOG_FLUSH();

//        if ((i != GPIO_VDD_MDM_EN) &&
//            (i != GPIO_MDM_PWR_KEY))
//        {
//            configGPIO(i, monet_conf.gConf[i].status);
//        }
//        monet_gpio.counter[i] = monet_conf.gConf[i].Reload;
//        if((monet_conf.gConf[i].status & GPIO_DIRECTION) == DIRECTION_OUT) {
//            SetGPIOOutput(i, (bool)((monet_conf.gConf[i].status & GPIO_SET_HIGH)>0));
//        } else {
//            GPIO_SET(i, pf_gpio_read(i));
//        }
		
		gpio_init_pin(i);
	}
}

void configGPIO(int index, uint8_t status)
{
    atel_gpio_cfg_t gpio_cfg;

//    NRF_LOG_RAW_INFO("%s: Index:%d, Status:0x%02X\r\n", __func__, index, status);
//    NRF_LOG_FLUSH();

    if (index >= GPIO_TO_INDEX(GPIO_NONE))
    {
        NRF_LOG_RAW_INFO("configGPIO Index Out of Range.\r\n");
        NRF_LOG_FLUSH();
        return;
    }

    if((status & GPIO_DIRECTION) == DIRECTION_OUT)
    {
        if(status & GPIO_MODE)
        {
            gpio_cfg.func = ATEL_GPIO_FUNC_OUT;
            gpio_cfg.pull = ATEL_GPIO_NOPULL;
        } else {
            gpio_cfg.func = ATEL_GPIO_FUNC_OD;
            gpio_cfg.pull = ATEL_GPIO_PULLUP;
        }
        pf_gpio_cfg(index, gpio_cfg);
    }
    else
    {
        if(status & GPIO_MODE)
        {
            gpio_cfg.func = ATEL_GPIO_FUNC_INT;
            gpio_cfg.sense = ATEL_GPIO_SENSE_TOGGLE;
//            gpio_cfg.pull = ATEL_GPIO_NOPULL;
            gpio_cfg.pull = ATEL_GPIO_PULLUP;
            pf_gpio_cfg(index, gpio_cfg);
        }
        else
        {
            gpio_cfg.func = ATEL_GPIO_FUNC_IN;
            gpio_cfg.pull = ATEL_GPIO_NOPULL;
            pf_gpio_cfg(index, gpio_cfg);
        }
    }
}

void SetGPIOOutput(uint8_t index, bool Active)
{
    if (index >= GPIO_TO_INDEX(GPIO_NONE))
    {
        NRF_LOG_RAW_INFO("SetGPIOOutput Index Out of Range.\r");
        NRF_LOG_FLUSH();
        return;
    }

    // IRQ_OFF
    if (((monet_conf.gConf[index].status & GPIO_OUTPUT_HIGH) && Active) ||
        (!(monet_conf.gConf[index].status & GPIO_OUTPUT_HIGH) && !Active)) {
        pf_gpio_write(index, 1);
    } else {
        pf_gpio_write(index, 0);
    }

    GPIO_SET(index, pf_gpio_read(index));

    if(monet_conf.gConf[index].Reload) {
        if(Active) {
            monet_gpio.counter[index] = monet_conf.gConf[index].Reload;
        } else {
            monet_gpio.counter[index] = 0;
        }
    }
    // IRQ_ON
}

int8_t atel_io_queue_process(void)
{
    int8_t ret = 0;
    static uint8_t txq_resend = 0;
    static uint8_t txq_resend_value = 0;
    uint32_t again_count = 0;
    uint8_t tmp = 0;

RECV_AGAIN:
    if (pf_uart_mdm_rx_one(&tmp) == 0)
    {
        GetRxCommandEsp(tmp);
        again_count++;
        if (again_count <= (MAX_COMMAND / 32))
        {
            goto RECV_AGAIN;
        }
    }

    again_count = 0;

    if (monet_data.uartMdmTXDEnabled == 0)
    {
        return 0;
    }

SEND_AGAIN:
    if (monet_data.txQueueU1.Size && monet_data.phonePowerOn && monet_data.appActive)
    {
        // We have something to send
        uint8_t txdata = 0;
        uint32_t force_send_count = 0;

        FORCE_SEND_AGAIN:
        if (txq_resend)
        {
            if (pf_uart_mdm_tx_one(txq_resend_value) == 0)
            {
                txq_resend = 0;
                txq_resend_value = 0;

                io_tx_count++;
                again_count++;
                if (again_count <= (MAX_COMMAND / 64))
                {
                    goto SEND_AGAIN;
                }
            }
        }
        else
        {
            txdata = AtelReadRingBuff(&monet_data.txQueueU1);
            if (pf_uart_mdm_tx_one(txdata) == 0)
            {
                io_tx_count++;
                again_count++;
                if (again_count <= (MAX_COMMAND / 64))
                {
                    goto SEND_AGAIN;
                }
            }
            else
            {
                txq_resend = 1;
                txq_resend_value = txdata;
            }
        }

        if (txq_resend)
        {
            force_send_count++;
            if (force_send_count < 3)
            {
                nrf_delay_ms(1);
                goto FORCE_SEND_AGAIN;
            }
        }

        ret = 1;

        if (txq_resend)
        {
            NRF_LOG_RAW_INFO("atel_io_queue_process uart_deinit(%d)\r", txq_resend);
            NRF_LOG_FLUSH();
            pf_uart_mdm_deinit();
            nrf_delay_ms(5);
            pf_uart_mdm_init(255, 0); // Use default baud rate
        }
    }
    else
    {
        if (monet_data.uartToBeDeinit == 1)
        {
            NRF_LOG_RAW_INFO("uartToBeDeinit: %d\r", monet_data.uartToBeDeinit);
            NRF_LOG_FLUSH();
            monet_data.uartToBeDeinit = 0;
            if (monet_data.uartMdmTXDEnabled != 0)
            {
                pf_uart_mdm_deinit();
            }
            MCU_Sleep_APP();
            MCU_Sleep_MDM();
            pf_cfg_before_sleep();
            monet_data.SleepState = SLEEP_NORMAL;
            monet_data.SleepStateChange = 1;
            monet_data.bbSleepNormalDelay = 2;
//            ble_send_timer_stop();
        }
    }

POP_AGAIN:
    if (monet_data.phonePowerOn && monet_data.appActive)
    {
        if ((ATEL_RING_BUFF_SIZE_BYTE - monet_data.txQueueU1.Size) > (2 * (BLE_DATA_RECV_BUFFER_CELL_LEN + 8)))
        {
            if (is_ble_recv_queue_empty() == 0)
            {
                uint16_t recv_len = 0;
                uint8_t *p_data = NULL;
                monet_data.ble_recv_st = 0xff;
                monet_data.ble_recv_id = 0xff;

                ble_recv_data_pop(&p_data, &recv_len, &monet_data.ble_recv_st, &monet_data.ble_recv_id);

                if (monet_data.ble_recv_st == 0xff)
                {
                    NRF_LOG_RAW_INFO(">>>aus_data_handler(0x%x).\r", monet_data.ble_recv_st);
                    printf_hex_and_char(p_data, recv_len);
                    NRF_LOG_FLUSH();
                }

                if ((recv_len) && 
                    ((monet_data.ble_recv_st != 0xff) && (monet_data.ble_recv_id != 0xff)))
                {
                    if (p_data[0] == 0xAA)
                    {
                        BuildFrame(p_data[0], p_data + 1, recv_len - 1);

                        // Warning: Do not handle 0xAA from BLE peers
						// if ((p_data[1] == COMP_ZAZU_BLE) 
						// 	&& is_in_pair_mode() == true 
						// 	&& ble_state_get() == BLE_STATE_WT_FOR_ZAZU_RESP)
						// {
						// 	const ble_nus_c_t *p_ble_nus_handle = ble_nus_handle_get();
						// 	//////////////////////////////////////////
						// 	// TODO: 1 update local BLE sensor info //
						// 	//////////////////////////////////////////
						// 	uint16_t conn_handle = BLE_CONN_HANDLE_INVALID;
						// 	conn_handle = p_ble_nus_handle->conn_handle;
						// 	sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
						// 	NRF_LOG_RAW_INFO("atel_io_queue_process() disconnect. Switch to mode BLE_STATE_WT_FOR_ZAZU_DISC\r");
						// 	ble_state_set(BLE_STATE_WT_FOR_ZAZU_DISC);
						// }
                    }
                    else
                    {
                        // static uint32_t recv_len_count = 0;
                        // recv_len_count += recv_len;
                        // NRF_LOG_RAW_INFO("atel_io_queue_process recv_len(%d) total(%d) QLeft(%d)\r", recv_len, recv_len_count, monet_data.txQueueU1.Size);
                        BuildFrame(IO_CMD_CHAR_BLE_RECV, p_data, recv_len);
                    }
                }

                monet_data.ble_recv_st = 0xff;
                monet_data.ble_recv_id = 0xff;

                if (p_data != NULL)
                {
                    ble_recv_data_delete_one();
                }

                goto POP_AGAIN;
            }
        }
    }

    return ret;
}

#define  BUF_UART_RETRAN_SIZE	256
#define  BUF_UART_RETRAN_LIMIT	(BUF_UART_RETRAN_SIZE-10)

uint8_t buf_uart_retran[BUF_UART_RETRAN_SIZE];	// UART data retransmission buffer
uint32_t buf_uart_retran_len = 0;				// Length of data in buffer. Unit in byte

void uart_peri_rx_process(void)
{
	uint8_t tmp = 0;
	static uint32_t tick = 0;
	
	if (pf_uart_peri_rx_one(&tmp) == NRF_SUCCESS)
	{
		buf_uart_retran[buf_uart_retran_len] = tmp;
		buf_uart_retran_len++;
		tick = pf_systick_get();
	}
	if (!buf_uart_retran_len)	// If buffer is empty, return
		return;
	if (buf_uart_retran_len > BUF_UART_RETRAN_LIMIT || pf_systick_get() != tick)
	{
		BuildFrame('#', buf_uart_retran, buf_uart_retran_len);
		buf_uart_retran_len = 0;
	}
}

//void mcu_start_capture_process(void)
//{
//    if (monet_data.glass_break_detected)
//    {
//        monet_data.glass_break_detected = 0;
//        CheckGlassBreakEvent(0);
//    }
//}

// Data format:
// Length(byte):  1  1   1   LEN       1
// Content:      '$' LEN CMD data_body 0x0D
void GetRxCommand(uint8_t RXByte)
{
    static uint8_t parmCount = 0;

    switch(monet_data.iorxframe.state) {
    case IO_WAIT_FOR_DOLLAR:
        if(RXByte == '$') {
            monet_data.appActive = 1;
            monet_data.iorxframe.state = IO_GET_FRAME_LENGTH;
        }
        break;
    case IO_GET_FRAME_LENGTH:
        #if CMD_CHECKSUM
		RXByte--;
        #endif /* CMD_CHECKSUM */
        if (RXByte <= MAX_COMMAND) {
            monet_data.iorxframe.length = RXByte;
            monet_data.iorxframe.remaining = RXByte;
            monet_data.iorxframe.state = IO_GET_COMMAND;
        } else {
            monet_data.iorxframe.state = IO_WAIT_FOR_DOLLAR;
        }
        break;
    case IO_GET_COMMAND:
        monet_data.iorxframe.cmd = RXByte;
        monet_data.iorxframe.state = IO_GET_CARRIAGE_RETURN;
        monet_data.iorxframe.checksum = RXByte;
        parmCount = 0;
        break;
    case IO_GET_CARRIAGE_RETURN:
        monet_data.iorxframe.checksum += RXByte;
        if (monet_data.iorxframe.remaining == 0) {
            #if CMD_CHECKSUM
			if (monet_data.iorxframe.checksum == 0xFF) {
            #else
			if (RXByte == 0x0D) {
            #endif /* CMD_CHECKSUM */
                HandleRxCommand();
            }
            monet_data.iorxframe.state = IO_WAIT_FOR_DOLLAR;
        } else {
            monet_data.iorxframe.remaining--;
            monet_data.iorxframe.data[parmCount++] = RXByte;
        }
        break;
    default:
        monet_data.iorxframe.state = IO_WAIT_FOR_DOLLAR;
        break;
    }
}

void GetRxCommandEsp(uint8_t RXByte)
{
#if CMD_ESCAPE
    // Ensure protocol is resynced if un-escaped $ is revc'd
    if (RXByte == '$')
    {
        // Clean up old data
        memset(&monet_data.iorxframe, 0, sizeof(monet_data.iorxframe));
        // Setup for new message
        monet_data.iorxframe.state = IO_WAIT_FOR_DOLLAR;
        monet_data.iorxframe.escape = IO_GET_NORMAL;
    }

    if (monet_data.iorxframe.state == IO_WAIT_FOR_DOLLAR) {
        GetRxCommand(RXByte);
        monet_data.iorxframe.escape = IO_GET_NORMAL;
        return;
    }

    switch (monet_data.iorxframe.escape) {
    case IO_GET_NORMAL:
        if (RXByte == '!')
        {
            monet_data.iorxframe.escape = IO_GET_ESCAPE;
        }
        else {
            GetRxCommand(RXByte);
        }
        break;
    case IO_GET_ESCAPE:
        monet_data.iorxframe.escape = IO_GET_NORMAL;
        if (RXByte == '0')
            GetRxCommand('$');
        else if (RXByte == '1')
            GetRxCommand('!');
        else {
            // Reset the state
            monet_data.iorxframe.state = IO_WAIT_FOR_DOLLAR;
        }
        break;
    }
#else
    GetRxCommand(RXByte);
#endif /* CMD_ESCAPE */
}

void acc_tilt_check(void)
{
#ifdef USE_TILT // PUMAMCU-136
    // PUMAMCU-146, save power when tilt timer is zero
    if (monet_data.TiltEventTimer && (monet_data.bShouldPollAcc && monet_data.bI2CisEnabled)) { // SIMBAMCU-30 MNT-1494 If Timer is not set don't process tilt
        // PUMAMCU-146
        ars_PollAcc();
        uint32_t nSamples = (uint32_t)ARS_QUEUE_COUNT(ars_accQ);
        volatile double differentialTilt = 0;
        uint32_t i = 0;
        //if (nSamples) {
        //    if (monet_data.AccTestMode) {
        //        flashOrange(1, 0);
        //    }
        //}
        for (i = 0; i < nSamples; i++) {
            accValues_t data = { 0 };
            volatile accValues_t slowTiltDataPoint = { 0 };
            volatile accValues_t veryslowTiltDataPoint = { 0 };
            (void)ARS_QUEUE_POP_SAFE(ars_accQ, &data);
            ars_addToSlowIIR(data);
            //MNT-1494if (!monet_data.InMotion && ars_getTiltState() == TILT_STATE_NONE) {
                ars_addToVerySlowIIR(data);
            //MNT-1494}
            slowTiltDataPoint = ars_getAvgSlowIIRData();
            veryslowTiltDataPoint = ars_getAvgVerySlowIIRData();
            differentialTilt = ars_calcCosTiltVector(slowTiltDataPoint, veryslowTiltDataPoint);
            // Check for tilt tamper
            ars_checkTiltState(differentialTilt);
            if (ARS_QUEUE_IS_EMPTY(ars_accQ)) {
                ARS_QUEUE_RESET(ars_accQ);
            }
        }
    }
#endif
}

static bool charger_restart_is_set = true;

void charger_restart_check(void)
{
	// Check if we need to restart the charger
	uint16_t AdcBackup_mv;
	
	//if (monet_data.V3PowerOn == 0)
	{
		AdcBackup_mv = adc_to_vol_conv(monet_data.AdcBackup, VOL_BAT_FACTOR);
		if ((no_external_power() == true)	// This function should be called after ADC conversion
//			||(only_solar_power() == true && solar_charge_invalid() == true ))  //NALAMCU-46
			|| (only_solar_power() == true && solar_chg_mode_get() == SOLAR_CHG_MODE2)
			|| (only_solar_power() == true && is_solar_vol_low() == true))
		{
			NRF_LOG_RAW_INFO("no external::AdcBackup_mv=%d.\r\n", AdcBackup_mv);
			NRF_LOG_FLUSH();
			count_recharge_check = 0;  //set to init value
			count_not_recharge_check = 0;  //set to init value
			recharge_start = false;  //set to init value
//			recharge_start = true;  //set to init value
			flashOrange(0, 0);
//#if (HW_VER < HW_VER_P3)
//			if (only_solar_power() == true && solar_charge_invalid() == true)
//			{
//				if(is_charger_power_on() != true)
//				{
//					setChargerOn();  //turn on to read charger power ok and then can judge whether solar power is invalid for charge.
//				}
//			}
//			else
//#else	
//			if (mppt_is_running() != true)
//#endif
			{
				if (no_external_power() == true)
				{
					solar_chg_mode_set(SOLAR_CHG_MODE1);
					timer_solar_chg_mode_stop();
				}
				charger_state_record();	// Should run before setChargerOff()
				setChargerOff();
			}
		}
		else
		{
			NRF_LOG_RAW_INFO("have external::ChargerRestartValue=%d.AdcBackup_mv=%d\r",monet_data.ChargerRestartValue,AdcBackup_mv);
			NRF_LOG_FLUSH();
			if(monet_data.ChargerRestartValue == 0)  //voltage for battery re-charging isn't set, charger state should be recovered.
			{
				count_recharge_check = 0;  //set to init value
				count_not_recharge_check = 0;  //set to init value
				recharge_start = false;  //set to init value
//				recharge_start = true;  //set to init value
				charger_state_recover();	// Should run before flashChargingStatus()
				flashChargingStatus();	// Note: this function should be called after ADC conversion
			}
			else  //voltage for battery re-charging is set
			{
				if (charger_restart_is_set == true)
				{
					charger_restart_is_set = false;
					recharge_start = true;
				}
				//NALAMCU-59
				if(AdcBackup_mv <= monet_data.ChargerRestartValue) //battery voltage is lower than voltage for battery re-charging.
				{
					count_recharge_check++;
					count_not_recharge_check = 0;
					if(count_recharge_check >= CHARGE_RESTART_CHECK_COUNT)
					{
						count_recharge_check = CHARGE_RESTART_CHECK_COUNT;
					}
				}
				else  //battery voltage is higher than voltage for battery re-charging.
				{
					count_not_recharge_check++;
					if(count_not_recharge_check >= CHARGE_RESTART_CHECK_COUNT)  //higher than for CHARGE_RESTART_CHECK_COUNT times continuously,
					{
						count_not_recharge_check = CHARGE_RESTART_CHECK_COUNT;
						count_recharge_check = 0;
					}
				}
				
				NRF_LOG_RAW_INFO("count_recharge_check=%d,count_not_recharge_check=%d,recharge_start=%d\r\n",count_recharge_check,count_not_recharge_check,recharge_start);
				NRF_LOG_FLUSH();
				if((count_recharge_check >= CHARGE_RESTART_CHECK_COUNT) //battery voltage is lower than voltage for battery re-charging for CHARGE_RESTART_CHECK_COUNT times continuously, charger state should be recovered.
					||(recharge_start == true)) //In order to ensure the battery is fully charged, set charger off only when recharge hasnot started. 
				{
					recharge_start = true;  //recharge has started.
					charger_state_recover();	// Should run before flashChargingStatus()
					flashChargingStatus();	// Note: this function should be called after ADC conversion
				}
				else
				{
					charger_state_record();	// Should run before setChargerOff()
					setChargerOff();
				}
			}
		}
	}
}

void shipping_mode_check(void)
{
	uint16_t vol_main = 0, vol_aux = 0;
	
	vol_main = adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR);
	vol_aux = adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR);

	if (monet_data.BUBX &&
        !CheckMainPower(MAX(vol_main, vol_aux)) &&
		!monet_data.phonePowerOn)
	{
//		monet_data.bEnterEM4 = 1;
		monet_data.bEnterSysOff = 1;
    }
}

void handleAccInterrupt(void) {
    BYTE src = 0;

    if (!monet_data.bI2CisEnabled) {
        return;
    }

    LIS_ReadReg(LIS_INT1_SOURCE, &src, 1); // PUMAMCU-121
    accInterruptFlag = src;

    // Ignore MEMS if we are in BUBX waiting for the tracker to go down
	// Ignore MEMS if we don't have real power (such as running on solar power)
	if (monet_data.BUBX == 0) {
		accInterruptHandle(accInterruptFlag); // PUMAMCU-136
		//if (monet_data.AccTestMode) {
		//	flashOrange(50, 0);
		//}
	}
}

void atel_timerTickHandler(uint32_t tickUnit_ms)
{
//    uint8_t i, bDoReport=0;
    uint8_t i;
//    BitStatus  s;
    uint8_t count = 0;
	static uint32_t count2 = 0;

    if (gTimer)
    {
        gTimer--;
    }
    else
    {
        return;
    }

    pf_wdt_kick();
	
    for(i=0; i < NUM_OF_GPIO; i++) {
        if ((monet_conf.gConf[i].status & GPIO_DIRECTION) == DIRECTION_OUT) { //including watchdog timer
            if (monet_gpio.counter[i]) {
                monet_gpio.counter[i]--;
                if (monet_gpio.counter[i]==0) {
                    // TODO: this may cause GPIO pin status change
					pf_gpio_toggle(i);
                    GPIO_SET(i, pf_gpio_read(i));
                    NRF_LOG_RAW_INFO("pf_gpio_toggle(index: %d value: %d)\r", i, GPIO_GET(i));
                    NRF_LOG_FLUSH();
                }
            }
        } 
		/* else { // Input
            if ((monet_conf.gConf[i].status & GPIO_IT_MODE) &&
                (i < MAX_EXT_GPIOS)) { // No need to read the internal GPIO pins
                if (monet_gpio.counter[i] || !monet_conf.gConf[i].Reload) {
                    if (monet_gpio.counter[i]) {
                        monet_gpio.counter[i]--;
                    }
                    if (!monet_gpio.counter[i] || !monet_conf.gConf[i].Reload) {
                        s = (BitStatus)(pf_gpio_read(i) > 0);
                        if (GPIO_GET(i) != s) {
                            GPIO_SET(i,s);
                            switch(monet_conf.gConf[i].status & GPIO_IT_MODE) {
                            case GPIO_IT_LOW_TO_HIGH:
                                if(s) {
                                    bDoReport = 1;
                                }
                                break;
                            case GPIO_IT_HIGH_TO_LOW:
                                if(!s) {
                                    bDoReport = 1;
                                }
                                break;
                            case GPIO_IT_BOTH:
                                bDoReport = 1;
                                break;
                            default:
                                break;
                            }
                            if (bDoReport) {
                                monet_gpio.Intstatus |= 1<<i;
								printf("i = %d\r\n", i);
                                bDoReport = 0;
                            }
                        }
                    }
                } else {
                    s = (BitStatus)(pf_gpio_read(i) > 0);
                    if (GPIO_GET(i) != s) {
                        monet_gpio.counter[i] = monet_conf.gConf[i].Reload;
                    }
                }
            } else {
                GPIO_SET(i, pf_gpio_read(i));
            }
        } //End of Input        */
    } //End of for loop

	// process LED tasks:
	for ( i=0; i<NUM_OF_LED; i++ )
	{
		if( monet_data.ledConf[i].status & LED_RUNNING )
		{
			monet_data.ledConf[i].tick++;
			if( monet_data.ledConf[i].status & LED_ON )
			{
				if( monet_data.ledConf[i].tick >= monet_data.ledConf[i].t_on )
				{
					monet_data.ledConf[i].tick = 0;
					monet_data.ledConf[i].status &= ~LED_ON;
					pic_turnOffLed(i);  // LED_ON timed out, turn it off
					if( (monet_data.ledConf[i].status & LED_REPEAT) == 0 )
					{
						monet_data.ledConf[i].status &= ~LED_RUNNING;;
					}
				}
			}
			else
			{
				if( monet_data.ledConf[i].tick >= monet_data.ledConf[i].t_off )
				{
					monet_data.ledConf[i].tick = 0;
					monet_data.ledConf[i].status |= LED_ON;
					pic_turnOnLed(i);   // LED_OFF timed out, turn it on
				}
			}
		}
	}
	
	// Check if we need to check the sensor input
	if (monet_data.SensorDelayedTest) {
		monet_data.SensorDelayedTest--;
		if (monet_data.SensorDelayedTest == 0) {
			uint8_t pParam[2];

			pParam[0] = MONET_V3_ON;
			pParam[1] = 1; //GPIO_PinInGet(COM_PORT, PERI_RX_PIN);
			BuildFrame('p', pParam, 2);
		}
	}
	
//    #if SUPPORT_ACC_STREAMING
//    pf_imu_value_stream();
//    #endif /* SUPPORT_ACC_STREAMING */
//	accInterruptHandle();
	
	// PUMAMCU-115
    if (accInterruptFlag) { // SIMBAMCU-30
        handleAccInterrupt();
//		printf("BUBX %u\r\n", monet_data.BUBX);;//////////////////
//		accInterruptFlag = 0;//////////////////
    }
	acc_tilt_check();

    gcount += tickUnit_ms;
    while (gcount >= 1000) {
        monet_data.sysRealTimeSeconds++;
        atel_timer1s();
        if (gcount >= 1000)
        {
            gcount -= 1000;
        }
        if (count == 0)
        {
//			uint16_t vol_main = 0, vol_aux = 0;
//            atel_adc_converion();
//			charger_restart_check();	// Note: this function should be called after ADC conversion
//			shipping_mode_check();
//			vol_main = adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR);
//			vol_aux = adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR);
//			CheckMainPower(MAX(vol_main, vol_aux));
        }
        count++;
    }
	
	if ((count1sec % 30) == 0) { // SIMBAMCU-30
        if (count2 != count1sec) {
            count2 = count1sec;
            monet_timer30s();
        }
    }

//    atel_uart_restore();
	monet_gpioEventGenerator();
	
//    isMdmNeedTobeWakeup(tickUnit_ms);

//	ble_conn_param_updated_check();

    device_ble_status_report(tickUnit_ms);

	device_bootloader_enter_dealy(tickUnit_ms);
	
//	mux_mcu_proc();
	
	mux_mcu_update_proc2();
}

uint8_t monet_isHallTamperActive(void) {
    // _SIMBA1 only
    return pf_gpio_read(GPIO_Hall_Tamper);
}

void monet_gpioEventGenerator(void)
{
    uint8_t i;
    uint8_t nValue;
    uint8_t ReportData[2];
    uint8_t bDoReport;

    for(i=0; i < NUM_OF_EXT_GPIO; i++) {
        if(monet_data.gpioEv[i].report == GPIO_REPORT_NONE) {
            continue;
        }
        nValue = pf_gpio_read(egpio_config[i]);
//#ifndef _SIMBA2
//        if (i == INT_TAMPER_WAKEUP) {
//            nValue |= monet_isHallTamperActive();
//        }
//#endif
        if (monet_data.gpioEv[i].gpioCurrent != nValue) {
            if (monet_data.gpioEv[i].debouncing) {
				if (monet_data.SleepState == SLEEP_HIBERNATE)	// Systick freq. is significantly reduced when in Hibernatioin/Sleep mode. Refer to timer_systick_handler() for more info.
				{
					uint32_t debounce_step = 0;
					
					if (TIME_UNIT_IN_SLEEP_NORMAL == 0)
						debounce_step = 1;
					else
						debounce_step = TIME_UNIT_IN_SLEEP_HIBERNATION/TIME_UNIT_IN_SLEEP_NORMAL;
					if (debounce_step == 0) debounce_step = 1;
					if (monet_data.gpioEv[i].debounceCount < debounce_step)
						monet_data.gpioEv[i].debounceCount = 0;
					else
						monet_data.gpioEv[i].debounceCount -= debounce_step;	// Increase step in Hibernation mode
				}
				else
					monet_data.gpioEv[i].debounceCount--;
            } else {
                monet_data.gpioEv[i].debouncing = 1;
                monet_data.gpioEv[i].debounceCount = monet_data.gpioEv[i].debounce;
            }
            if (monet_data.gpioEv[i].debounceCount == 0) {
                monet_data.gpioEv[i].debouncing = 0;
                monet_data.gpioEv[i].gpioCurrent = nValue;
                ReportData[0] = i;
                ReportData[1] = nValue;
                bDoReport = 0;
                switch(monet_data.gpioEv[i].report) {
                case GPIO_REPORT_LOW_TO_HIGH:
                    if(nValue) {
                        bDoReport = 1;
                    }
                    break;
                case GPIO_REPORT_HIGH_TO_LOW:
                    if(!nValue) {
                        bDoReport = 1;
                    }
                    break;
                case GPIO_REPORT_BOTH:
                        bDoReport = 1;
                    break;
                default:
                    break;
                }
                if (bDoReport) {
                    if(i != 4)  // index 4 is used for the GPIO_ONE_BUS_SLPZ. NALAMCU-54
                    {
               		monet_gpio.Intstatus |= (1<<i);
	                // If the phone is on, we send a frame
               		if (monet_data.phonePowerOn) 
               		{
					BuildFrame('e', &ReportData[0], 2);
               		} 
               		else 
               		{
					monet_data.sleepmode = 0; // SIMBAMCU-29
               		}
                    }
                }
            }
        } else {
            monet_data.gpioEv[i].debouncing = 0;
        }
    }
}

/*
 *
 * mnt_accHeartbeat
 *
 * Purpose: one second check of the accelerometer interrupt pin
 *
 */
//void mnt_accHeartbeat(void)
//{
//    ars_PollAcc();
//    if ((SLEEP_OFF == monet_data.SleepState) && mp_isEventIReadyFlag()) 
//    { // WCMCU-139 & WCMCU-141 Block streaming message until App UART is ready
//        uint32_t nStreamSamples = ((ACC_MODE_COLLISION_REPORT != ars_accdata.AccWorkingMode) ? 10 : 32); // WCMCU-143
//        uint32_t nSamples = (uint32_t)ARS_QUEUE_COUNT(ars_accQ);
//        uint32_t i = 0;
//#if defined(SUPPORT_ACC_STREAMING) // WCMCU-9
//        uint8_t acc_stream[255] = {0};
//        uint16_t index = 0;
//        uint16_t streamcount = 0;
//        
//        if (nStreamSamples < nSamples) { // Only send stream msg if there are enough samples to send
//            acc_stream[index++] = 'S';
//#endif // End SUPPORT_ACC_STREAMING
//            for (i = 0; i < nSamples; i++) {
//                accValues_t data = { 0 };
//                (void)ARS_QUEUE_POP_SAFE(ars_accQ, &data);
//#if defined(SUPPORT_ACC_STREAMING) // WCMCU-9
//                acc_stream[index++] = (uint8_t)((data.x & 0x00FF) >> 0);
//                acc_stream[index++] = (uint8_t)((data.x & 0xFF00) >> 8);
//                acc_stream[index++] = (uint8_t)((data.y & 0x00FF) >> 0);
//                acc_stream[index++] = (uint8_t)((data.y & 0xFF00) >> 8);
//                acc_stream[index++] = (uint8_t)((data.z & 0x00FF) >> 0);
//                acc_stream[index++] = (uint8_t)((data.z & 0xFF00) >> 8);
//                streamcount++;
//                if ((nStreamSamples < streamcount) && eventIReadyFlag) {
//                    BuildFrame('x', acc_stream, index);
//                    break; // Leave the loop once message has been built
//                }
//#endif // End SUPPORT_ACC_STREAMING
//            }                
//        }
//#if defined(SUPPORT_ACC_STREAMING) // WCMCU-9
//    }
//#endif // End SUPPORT_ACC_STREAMING
//    if (ARS_QUEUE_IS_EMPTY(ars_accQ)) {
//        ARS_QUEUE_RESET(ars_accQ);
//    }
//}

//void accInterruptHandle(void)
//{
//    uint8_t src = 0;
//    uint8_t i = 0;

//	if (!monet_data.bI2CisEnabled) {
//        return;
//    }
//	
//    for (i = ACC_INTERRUPT_NUM1; i <= ACC_INTERRUPT_NUM2; i++)
//    {   src = pf_imu_int_src_get(i);
//        if (src & MOTION_INT_MASK())
//        { // Check all relevant bits
//            // NRF_LOG_RAW_INFO("accInterruptHandle src: 0x%x)\r", src);
//            // NRF_LOG_FLUSH();
//            monet_data.IntSrc = src;
//            if (monet_data.InMotion < 0xff)
//            { // This counter is cleared each second and serves only as an movement indicator
//                monet_data.InMotion++;
//            }
//        }
//    }
//}

APP_TIMER_DEF(pf_adc_conv_prp_timer);	// Timer for ADC preparation
#define ADC_CONV_PREP_TIME	20			// Unit in ms. Time 15 ms is tested long enough.
static bool adc_conv_prp_timeout = false;

static void timer_adc_conv_prp_handler(void * p_context)
{
	if (p_context == (void *)pf_adc_conv_prp_timer)
	{
		adc_conv_prp_timeout = true;
	}
}

// Prepare for ADC conversion
// Capacitances (C506 and C507) need time to reach target voltage
void adc_conv_prepare(void)
{
	pf_gpio_write(GPIO_BAT_ADC_TMP_EN, 1);	// Enable ADC power switch
	app_timer_stop(pf_adc_conv_prp_timer);
	app_timer_create(&pf_adc_conv_prp_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_adc_conv_prp_handler);
	app_timer_start(pf_adc_conv_prp_timer, APP_TIMER_TICKS(ADC_CONV_PREP_TIME), (void *)pf_adc_conv_prp_timer);
	adc_conv_prp_timeout = false;
}

void atel_adc_converion(void)
{
    pf_adc_init();
    pf_adc_start(); // Trigger ADC sample
    pf_adc_poll_finish();
    pf_adc_deinit();
}

void adc_conv_proc(void)
{
	if (adc_conv_prp_timeout == true)
	{
		adc_conv_prp_timeout = false;
		atel_adc_converion();
		pf_gpio_write(GPIO_BAT_ADC_TMP_EN, 0);	// Disable ADC power switch
	}
}

// Stop ADC conversion
// Call this function when need to manually stop ADC conversion
void adc_conv_stop(void)
{
	app_timer_stop(pf_adc_conv_prp_timer);
	adc_conv_prp_timeout = false;
	pf_gpio_write(GPIO_BAT_ADC_TMP_EN, 0);
}

// Flag to mark main/aux power voltage low or not
// If main/aux power voltage < 7.5 V AND no solar power, return true; else return false.
static bool main_aux_vol_low = false;

bool is_main_aux_vol_low(void)
{
	return main_aux_vol_low;
}

// Check main and aux power supply state.
// If main and aux voltage < 7.5 V, the charger LED should be off. NALAMCU-103
// This function is designed to be called every 1 second
void main_aux_power_check(void)
{
	uint16_t main_vol = 0;
	uint16_t aux_vol = 0;
	uint16_t solar_vol = 0;
	
	main_vol = adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR);
	aux_vol = adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR);
	solar_vol = adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR);
	if (main_vol < ADC_MAIN_AUX_TH && aux_vol < ADC_MAIN_AUX_TH && solar_vol < ADC_MAIN_TH)	// Main/aux voltage < 7.5 V and no solar power
	{
		if (main_aux_vol_low == false)	// Reach here for the first time
		{
			flashOrange(0, 0);
			NRF_LOG_RAW_INFO("\r\nmain_aux_power_check(), Main/aux < 7.5 V AND no solar power\r\n");
		}
		main_aux_vol_low = true;
	}
	else
	{
		main_aux_vol_low = false;
	}
}

//refresh the monitor uart num
static void device_uart_alive_refresh(void)
{
	monet_data.uartAliveDebounce = 0;
	monet_data.uartAliveCount = 0;
}
/*
**@date:2021.09.26
**@brief: for monitor uart communition 
**@note: when watchdog timeout will reset device
*/

static void check_uart_alive_handle(void)
{
	if ((monet_data.phonePowerOn) && (SLEEP_OFF == monet_data.SleepState))
	{
		monet_data.uartAliveDebounce++;
		if (monet_data.uartAliveDebounce >= DEVICE_UART_ALIVE_DEBOUNCE)
		{
			NRF_LOG_RAW_INFO("device_uart_alive_handle (D:%d  C:%d) \r",
			                 monet_data.uartAliveDebounce,
							 monet_data.uartAliveCount);

			monet_data.uartAliveDebounce = 0;
			monet_data.uartAliveCount++;
			
			pf_uart_mdm_deinit();
            nrf_delay_ms(5);
            pf_uart_mdm_init(255, 0);
			NRF_LOG_RAW_INFO("device uart try open again \r");
			if (monet_data.uartAliveCount > DEVICE_UART_ALIVE_COUNT_LIMIT)
			{
				monet_data.uartAliveCount = 0;
				NRF_LOG_RAW_INFO("Uart Error! Reset System.\r");
       			//WDTimer == 0 will results system reset
      			monet_gpio.WDtimer = 0;
			}
            NRF_LOG_FLUSH();
		 }
    }
	else
	{
		device_uart_alive_refresh();
	}
}

void atel_timer1s(void)
{
    data_time_table_t time_table = {0};
    count1sec++;

#ifdef USE_TILT // PUMAMCU-136
    if (!ARS_QUEUE_IS_EMPTY(ars_accQ)) {
        ion_accHeartbeat(); // Process accelerometer data
    }
#endif
	
	// NALAMCU-29 Process the one wire interface
	if ((monet_data.OneWireDisable == 0) && 
        (config_data.bShippingMode == 0)) { // PUMAMCU-97
		oneWire_oneSecond();
        // PUMAMCU-108 Allow wake up after any temp transision
//        if ((monet_data.interrupt & (1 << INT_TEMP_ONEWIRE)) &&	// Ignored. CheckInterrupt() in main() would check monet_gpio.Intstatus (monet_data.interrupt) to wake up Modem
//            !monet_data.phonePowerOn && !gResetCounter) {
//            gResetCounter = 1;
//        }
    }

    NRF_LOG_RAW_INFO("timer_s: %d InterruptFlags(0x%x) Sleep(%d) WD(%d) SAlarm(%d)\r", count1sec, monet_gpio.Intstatus, monet_data.SleepState, monet_gpio.WDtimer, monet_data.SleepAlarm);
    time_table = SecondsToTimeTable(monet_data.sysRealTimeSeconds);
    NRF_LOG_RAW_INFO("%d/%d/%d %02d:%02d:%02d \r", time_table.year, time_table.month, time_table.day, time_table.hour, time_table.minute, time_table.second);
    NRF_LOG_RAW_INFO("Cargo Sensor(%u), BLE conn(%u), Zazu conn method(%u)\r",
					monet_data.V3PowerOn, (ble_aus_ready_state_get_c() == true)? 1: 0, (uint32_t)com_method_zazu_get());
    //NRF_LOG_RAW_INFO("BLEC_Num(%d)\r", ble_connected_channel_num_get());
    NRF_LOG_RAW_INFO("Solar CHG Mode (%u), MPPT state(%u), Charger (%u)\r",
					(uint32_t)solar_chg_mode_get(), mppt_is_running(), is_charger_power_on());
	NRF_LOG_RAW_INFO("Pair mode(%u), ble state(%u)\r", is_in_pair_mode(), ble_state_get());
    NRF_LOG_FLUSH();
	
    if(monet_gpio.WDtimer==0) {
        NRF_LOG_RAW_INFO("Watchdog Timeout.\r");
        NRF_LOG_FLUSH();
        // if (monet_conf.WD.Pin != GPIO_TO_INDEX(GPIO_NONE)) {
        //     // TODO: whether to change the level of this pin to control modem
        //     SetGPIOOutput(monet_conf.WD.Pin, 0);
        // }
        IncreaseCount(&rst);
        MCU_TurnOff_MDM();
        systemreset(monet_gpio.WDflag);	// This is endless loop waiting for watchdog timeout reset. Following code would not be executed.
        // Don't allow timer to be reset until modem is powered off
        monet_gpio.WDtimer = gBWD;
        monet_gpio.WDflag = 0;
    } else if (monet_data.phonePowerOn				// Only update watchdog timer when Modem is on
			   && monet_gpio.WDtimer != 0xffff) {	// Special mode. When monet_gpio.WDtimer is set to 0xffff ('B' command), disable WD update
			monet_gpio.WDtimer--;
    }

	if (monet_data.basebandWakeDelay) {
		monet_data.basebandWakeDelay--;
	}
	
	// Process the forced baseband reset
	if (monet_data.ResetBaseBandDelay) {
		monet_data.ResetBaseBandDelay--;
		if (monet_data.ResetBaseBandDelay == 0) {
//			resetBaseband(false);			// PUMAMCU-163
			if (monet_data.phoneLive != 0)	// Avoid repeatedly turning off Modem
				pic_turnOffBaseband();
			monet_data.ResetBaseBandCounter = 5 + 2;	// pic_turnOffBaseband() needs 2 s to turn off Modem
			NRF_LOG_RAW_INFO("Resetting Modem: turn off mdm. monet_data.phoneLive 0x%x\r\n", monet_data.phoneLive);
		}
	}
	
	// Process the forced baseband reset
	if (monet_data.ResetBaseBandCounter) {
		monet_data.ResetBaseBandCounter--;
		if (monet_data.ResetBaseBandCounter == 0) {
			if (monet_data.phonePowerOn != 1)	// Avoid repeat turning on Modem
				pic_turnOnBaseband();
			NRF_LOG_RAW_INFO("Resetting Modem: turn on mdm. monet_data.phonePowerOn %u\r\n", monet_data.phonePowerOn);
		}
	}
	
	monet_data.rctime++;

	switch (monet_data.AdcPhase) {
	    case 0:
	        // Read Main
	        
	        break;
	    case 1:
		{
	        // Read Battery
	        uint16_t AdcBat;
	        AdcBat = (uint16_t)PF_ADC_RAW_TO_BATT_MV(monet_data.AdcBackup);

	        NRF_LOG_RAW_INFO("atel_timer1s, AdcBat(%d) gAdcBatCounter(%d) BubCriticalThreshold(%d).\r", AdcBat, gAdcBatCounter, monet_data.BubCriticalThreshold);
	        NRF_LOG_FLUSH();
	        if ((AdcBat < ADC_BAT_TH || AdcBat < monet_data.BubCriticalThreshold) &&
	            !isOtherPowerSource())
	        { // SIMBAMCU-7
	            if (gAdcBatCounter != BUB_CRITICAL_DEBOUNCE) {
	                gAdcBatCounter++;
	            } else {
	                monet_data.bBattMarkedCritical = true;
	            }
	        }
	        else
	        {
	            gAdcBatCounter = 0;
	        }
	        // Check if it is time to turn the tracker on
	        if (!config_data.bShippingMode &&
	            !monet_data.BUBX &&
	            /*!gResetCounter && */
	            !monet_data.basebandWakeDelay) {
	            if (CheckMainPower(MAX(adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR), adc_to_vol_conv(monet_data.AdcAux, VOL_MAIN_FACTOR))) || 
	                (AdcBat >= ADC_BAT_OK)) 
	            { // SIMBAMCU-7 & SIMBAMCU-31
	                if (monet_data.bBattMarkedCritical) { // Leaving battery critical state so ok to set gpios back to normal
	                    config_data.bAllowPowerKey = 0;
	                    //gpio_init();
	                    //pf_uart_peri_init(9, 0);
	                    //pf_uart_mdm_init(UART_BAUDRATE_SELECT, 0);
	                    //pf_i2c_init();
	                    NRF_LOG_RAW_INFO("exit battery power save mode.\r");
	                    NRF_LOG_FLUSH();
						
	                    monet_data.bBattMarkedCritical = false;
	                    pic_turnOnBaseband();
	                }
	                monet_data.bBattMarkedCritical = false;
	                monet_data.bEnterEM3 = false;
	            }
	            if (monet_data.bBattMarkedCritical && 
	                !isOtherPowerSource() && 
	                (gAdcBatCounter==BUB_CRITICAL_DEBOUNCE)) 
	            { // SIMBAMCU-7 Battery Critical state
	                
	                if(!monet_data.bEnterEM3)
	                {
	                monet_data.bEnterEM3 = 1;
	                monet_data.sleepmode = 0; // SIMBAMCU-29 Clear sleep indicator
	                if (monet_data.phonePowerOn) {
	                    pic_turnOffBaseband();
	                }
	                setChargerOn(); // SIMBAMCU-7 make sure the charger is enabled any time the battery is low

	                monet_data.ledConf[0].status = 0;
	                monet_data.ledConf[1].status = 0;					
	                pf_gpio_write(GPIO_LED_GREEN, 1);
	                pf_gpio_write(GPIO_LED_RED, 1);
	                }
	            }
	        }
	     }
	     break;
	}
	// Increment the round robin counter
	monet_data.AdcPhase++;
	if (monet_data.AdcPhase >= NUM_OF_ADC) {
        monet_data.AdcPhase = 0;
	}

	
   // Check if wakeup call was ordered
	if (monet_data.phonePowerOn == 0)
	{
		if (monet_data.SleepAlarm != 0) {
			monet_data.SleepAlarm--;
			if (monet_data.SleepAlarm == 0) {
				monet_gpio.Intstatus |= MASK_FOR_BIT(INT_WAKEUP_TIMER); // This will force modem on in CheckInterrupt()
				NRF_LOG_RAW_INFO("SleepAlarm reached.\r\n");
				NRF_LOG_FLUSH();
			}
		}
	}

	if (monet_data.bbPowerOffDelay) {
		monet_data.bbPowerOffDelay--;
		if (monet_data.bbPowerOffDelay == 0) {
			NRF_LOG_RAW_INFO("MCU_TurnOff_MDM(power off delay == 0).\r");
			NRF_LOG_FLUSH();
			MCU_TurnOff_MDM();
//			ble_send_timer_stop();
		}
	}

//	if (monet_data.phoneLive & monet_data.interrupt)
	if (monet_data.phoneLive & monet_gpio.Intstatus)
    {
		monet_Icommand();
	}
	
//	atel_adc_converion();
	adc_conv_prepare();
	charger_restart_check();	// Note: this function should be called after ADC conversion
	shipping_mode_check();		// Note: this function should be called after ADC conversion
	if(gFlashLED) flashLED();
	mppt_process_nml();			// Note: this function should be called after ADC conversion
	mp2762a_proc();
	solar_chg_mode_proc();
	main_aux_power_check();
	
    if (accPollingEnableTimer > 0) {
        accPollingEnableTimer--;
        if (accPollingEnableTimer == 0) {
            setShouldPollAcc(false);
        }
    }
	
    CheckForMotion();

    CheckPowerState();

	mux_mcu_update_proc();

    //it will cause monet_gpio.WDtimer = 0, so should avoid operation monet_gpio.WDtimer in other plase.
	check_uart_alive_handle();
	
    CheckVirtualIgnition();

//    CheckGlassBreakEvent(1);
}

static void ResetPowerStates(void) {
    // Clear any current bit set
    monet_gpio.Intstatus &= ~(
        MASK_FOR_BIT(INT_POWER_LOW) |
        MASK_FOR_BIT(INT_POWER_HIGH) |
        MASK_FOR_BIT(INT_BAT_OFF) |
        MASK_FOR_BIT(INT_BAT_ON));
    // Reset Debounce times
    monet_data.debounceMain = monet_data.debounceMainPower;
    monet_data.debounceBatt = monet_data.debounceBattPower;
    monet_data.debounceLow  = monet_data.debounceLowPower;
    monet_data.debounceHigh = monet_data.debounceHighPower;
}

static void ConfigPowerMasks(uint8_t* pParam, uint8_t Length) {
    monet_data.powerStateMask = pParam[0];

    // MONET_POWER_SWMAIN means battery power voltage is out range will wake device
    // MONET_POWER_SWBATT means main power voltage is out range will wake device
    if (monet_data.powerStateMask & MONET_POWER_LOW_THRES) {
        monet_data.powerLowThreshold = (uint16_t)pParam[1] + (uint16_t)(pParam[2] * 256);
        NRF_LOG_RAW_INFO("powerLowThreshold(%d mv).\r", monet_data.powerLowThreshold);
        NRF_LOG_FLUSH();
    }

    if (monet_data.powerStateMask & MONET_POWER_HIGH_THRES) {
        monet_data.powerHighThreshold = (uint16_t)pParam[3] + (uint16_t)(pParam[4] * 256);
        NRF_LOG_RAW_INFO("powerHighThreshold(%d mv).\r", monet_data.powerHighThreshold);
        NRF_LOG_FLUSH();
    }

    // Store Debounce times
    monet_data.debounceMainPower = (uint16_t)pParam[5] + ((uint16_t)pParam[6] * 256);
    monet_data.debounceBattPower = monet_data.debounceMainPower;
    NRF_LOG_RAW_INFO("debounceBattPower(%d s).\r", monet_data.debounceBattPower);
    NRF_LOG_FLUSH();

    monet_data.debounceLowPower  = (uint16_t)pParam[7] + ((uint16_t)pParam[8] * 256);
    monet_data.debounceHighPower = monet_data.debounceLowPower;
    NRF_LOG_RAW_INFO("debounceHighPower(%d s).\r", monet_data.debounceHighPower);
    NRF_LOG_FLUSH();

    ResetPowerStates();
}

static int8_t mx_isBatteryValid(uint16_t AdcBackup_mV) {
    static uint16_t batteryDebounce = BUB_CRITICAL_DEBOUNCE;
    int8_t result = (batteryDebounce) ? 1 : 0;
    if (AdcBackup_mV >= monet_data.AdcBatC) {
        batteryDebounce = BUB_CRITICAL_DEBOUNCE;
    } else {
        if (batteryDebounce > 0) {
            batteryDebounce--;
            if (batteryDebounce == 0) {
                result = 0;
            }
        }
    }
    return result;
}

uint8_t CheckMainPower(uint16_t volts) {
    uint8_t onMainPower = 0;

    if (volts >= MAIN_ADC_MAIN_VALID) {
        // Was on battery show new state
        monet_data.debounceMainFlag = 1;
        monet_data.debounceBattFlag = 0;
        monet_data.debounceBatt = monet_data.debounceBattPower;
        onMainPower = 1;
    } else  {
        // Was on Main show new state
        monet_data.debounceMainFlag = 0;
        monet_data.debounceBattFlag = 1;
        monet_data.debounceMain = monet_data.debounceMainPower;
//		flashOrange(0, 0);	// NALAMCU-64, fix bug of "The Flashing of the Orange LED is too short and therefore not visible"
    }
    return onMainPower;
}

void resetADCcounter()
{
	gAdcBatCounter=0;
	//gAdcMainCounter=0;
	//gAdcTempCounter=0;
}

uint16_t max3(uint16_t a,uint16_t b,uint16_t c)
{
    uint16_t max;

    max = a>b?a:b;
	
    if(max > c)
    	return max;
    else
    	return c;
}

// @Power_event_selection
#define PWR_EVENT_MAIN_PWR_PLUG_IN		1
#define PWR_EVENT_MAIN_PWR_GONE			2

// Power event handle
// Param. pwr_event: power event. See @Power_event_selection for value selection
void power_event_handle(uint32_t pwr_event)
{
	if (pwr_event == PWR_EVENT_MAIN_PWR_PLUG_IN)
	{
#if (HW_VER >= HW_VER_P3)
//		if (solar_chg_mode_get() == SOLAR_CHG_MODE2)
//			solar_chg_mode_set(SOLAR_CHG_MODE1);
//		setChargerOn();
		solar_chg_mode_select(FUNC_JUMP_POINT_0);
#endif
	}
	else if (pwr_event == PWR_EVENT_MAIN_PWR_GONE)
	{
		/* Reserved */;
	}
}

void CheckPowerState(void) {
    uint16_t vCheck = 0; //, mainPower, lightPower;
    uint16_t value_raw = 0;
    uint16_t MaxAdc = max3(monet_data.AdcMain, monet_data.AdcAux, 0/*monet_data.AdcSolar*/);  //should not wake when Solar power is applied
    uint16_t AdcMain_mV = (uint16_t)PF_ADC_RAW_TO_MAIN_MV(MaxAdc);  //monet_data.AdcMain
    uint16_t AdcBackup_mV = (uint16_t)PF_ADC_RAW_TO_BATT_MV(monet_data.AdcBackup);
		
    NRF_LOG_RAW_INFO("CheckPowerState(main %u mV, ", adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR));
    NRF_LOG_RAW_INFO("aux %u mV, ", adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR));
    NRF_LOG_RAW_INFO("solar %u mV)\r", adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR));
    NRF_LOG_RAW_INFO("Main(0x%x:%d mv) Bat(0x%x:%d mv)\r", MaxAdc, AdcMain_mV, monet_data.AdcBackup, AdcBackup_mV);
    //NRF_LOG_RAW_INFO("Baten(%d) IGNI(%d:%x) ", pf_gpio_read(GPIO_P016_BAT_EN), GPIO_GET(GPIO_BLE_P019_IGNI), monet_conf.gConf[GPIO_BLE_P019_IGNI].status);
    NRF_LOG_FLUSH();

    if ((monet_data.MainState == MAIN_GONE) &&
        (0 == mx_isBatteryValid(AdcBackup_mV)) &&
        (monet_data.phonePowerOn == 0)) {
        NRF_LOG_RAW_INFO("Disable BUB\r");
        NRF_LOG_FLUSH();
        // Disable BUB (suicide)
        BatteryPowerHoldEn(RELEASE);
    }

    // if ((monet_data.MainState == MAIN_GONE) &&
    //     (0 == mx_isBatteryValid(AdcBackup_mV)))
    // {
    //     NRF_LOG_RAW_INFO("BUB Low Turn Off BB\r");
    //     NRF_LOG_FLUSH();
    //     pic_turnOffBaseband();
    // }

    // We check if we have valid power
    if (AdcMain_mV >= MAIN_ADC_MAIN_VALID) {
        if (monet_data.MainState == MAIN_GONE) {
            NRF_LOG_RAW_INFO("main pwr restored.\r");
            NRF_LOG_FLUSH();
            
            if (monet_data.powerStateMask & MONET_POWER_SWBATT)
            {
                monet_data.MainStateChanged = 0;
            }
            if (monet_data.powerStateMask & MONET_POWER_SWMAIN)
            {
                monet_data.MainStateChanged = 1;
            }
			
			power_event_handle(PWR_EVENT_MAIN_PWR_PLUG_IN);
        }
        monet_data.MainState = MAIN_NORMAL;
    }
    else if ((AdcMain_mV <= MAIN_ADC_MAIN_OFF) &&
             (monet_data.MainState != MAIN_GONE)) {
        NRF_LOG_RAW_INFO("main pwr gone.\r");
        NRF_LOG_FLUSH();
        if (monet_data.powerStateMask & MONET_POWER_SWMAIN)
        {
            monet_data.MainStateChanged = 0;
        }
        if (monet_data.powerStateMask & MONET_POWER_SWBATT)
        {
            monet_data.MainStateChanged = 1;
        }
        monet_data.MainState = MAIN_GONE;
		power_event_handle(PWR_EVENT_MAIN_PWR_GONE);
    }

    CheckMainPower(AdcMain_mV);

    if (monet_data.powerStateMask & MONET_POWER_SWMAIN) {
        if (monet_data.debounceMainFlag) {
            if (monet_data.debounceMain > 0) {
                monet_data.debounceMain--;
                if (monet_data.debounceMain == 0) {
                    // Power was restored, we send an interrupt
                    monet_gpio.Intstatus |= MASK_FOR_BIT(INT_BAT_OFF);
                    NRF_LOG_RAW_INFO("INT_BAT_OFF\r");
                    NRF_LOG_FLUSH();
                    monet_data.MainStateChanged = 0;
                }
            }
        } else {
            // Reset the debounce time
            monet_data.debounceMain = monet_data.debounceMainPower;
        }
    }

    if (monet_data.powerStateMask & MONET_POWER_SWBATT) {
        if (monet_data.debounceBattFlag) {
            if (monet_data.debounceBatt > 0) {
                monet_data.debounceBatt--;
                if (monet_data.debounceBatt == 0) {
                    monet_gpio.Intstatus |= MASK_FOR_BIT(INT_BAT_ON);
                    NRF_LOG_RAW_INFO("INT_BAT_ON\r");
                    NRF_LOG_FLUSH();
                    monet_data.MainStateChanged = 0;
                }
            }
        } else {
            // Reset the debounce time
            monet_data.debounceBatt = monet_data.debounceBattPower;
        }
    }
    NRF_LOG_RAW_INFO("INT_BAT(0x%02x) MC(%d) MD(%d:%d) BD(%d:%d)\r", 
                      monet_data.powerStateMask, monet_data.MainStateChanged,
                      monet_data.debounceMain, monet_data.debounceMainPower,
                      monet_data.debounceBatt, monet_data.debounceBattPower);
    NRF_LOG_FLUSH();

    // MONET_POWER_SWMAIN means battery power voltage is out range will wake device
    // MONET_POWER_SWBATT means main power voltage is out range will wake device
    if (monet_data.powerStateMask & MONET_POWER_SWMAIN) {
        value_raw = monet_data.AdcBackup;
        vCheck = (uint16_t)PF_ADC_RAW_TO_BATT_MV(value_raw);
    }
    else if (monet_data.powerStateMask & MONET_POWER_SWBATT) {
        value_raw = MaxAdc;
        vCheck = (uint16_t)PF_ADC_RAW_TO_MAIN_MV(value_raw);
    }

    NRF_LOG_RAW_INFO("vCheck(%d:%d mv) Low(%d) High(%d).\r", value_raw, vCheck, monet_data.powerLowThreshold, monet_data.powerHighThreshold);
    NRF_LOG_FLUSH();

    if (monet_data.powerStateMask & MONET_POWER_LOW_THRES) {
        if ((vCheck < monet_data.powerLowThreshold) &&
            (monet_data.MainStateChanged == 0)) {
            if (monet_data.debounceLow > 0) {
                monet_data.debounceLow--;
                if (monet_data.debounceLow == 0) {
                    monet_gpio.Intstatus |= MASK_FOR_BIT(INT_POWER_LOW); // Latch the Power Low Threshold bit
                    NRF_LOG_RAW_INFO("INT_POWER_LOW.\r");
                    NRF_LOG_FLUSH();
                }
            }
        } else {
            monet_data.debounceLow = monet_data.debounceLowPower;
        }
    }

    if (monet_data.powerStateMask & MONET_POWER_HIGH_THRES) {
        if ((vCheck > monet_data.powerHighThreshold) &&
            (monet_data.MainStateChanged == 0)) {
            if (monet_data.debounceHigh > 0) {
                monet_data.debounceHigh--;
                if (monet_data.debounceHigh == 0) {
                    monet_gpio.Intstatus |= MASK_FOR_BIT(INT_POWER_HIGH); // Latch the Power High Threshold bit
                    NRF_LOG_RAW_INFO("INT_POWER_HIGH.\r");
                    NRF_LOG_FLUSH();
                }
            }
        } else {
            monet_data.debounceHigh = monet_data.debounceHighPower;
        }
    }
}

void BatteryPowerHoldEn(FunCtrl Status)
{
//    // Active high
//    if (Status == HOLD) {
//        pf_gpio_write(GPIO_P016_BAT_EN, 1);
//    } else if (Status == RELEASE) {
//        pf_gpio_write(GPIO_P016_BAT_EN, 0);
//    }
}

void monet_configVirtualIgnition(uint8_t* pParam, uint8_t length) {  //NALAMCU-82
   monet_data.virtualIgnitionTrigger       = pParam[0];
   monet_data.virtualIgnitionVoltage       = (pParam[2] << 8) + pParam[1];
   monet_data.virtualIgnitionHysteresis    = (pParam[4] << 8) + pParam[3];
   monet_data.virtualIgnitiondDebounce     = pParam[5];
   
   monet_data.virtualIgnitionDebounceTime = monet_data.virtualIgnitiondDebounce;

   NRF_LOG_RAW_INFO("VirtualSet: T(%d) S(%d) V(%dmv) H(%dmv) D(%dS).\r",
                        monet_data.virtualIgnitionTrigger,
                        monet_data.virtualIgnitionState,
                        monet_data.virtualIgnitionVoltage,
                        monet_data.virtualIgnitionHysteresis,
                        monet_data.virtualIgnitionDebounceTime);
   NRF_LOG_FLUSH();

   // SIMBAMCU-64    BuildFrame('a', pParam, length); // Send echo back
}

static bool DebounceVirtualIgnition(void)
{
    if (monet_data.virtualIgnitionDebounceTime > 0)
    {
        monet_data.virtualIgnitionDebounceTime--;
        if (monet_data.virtualIgnitionDebounceTime == 0)
        {
            return true;
        }
    }
    return false;
}

void CheckVirtualIgnition(void)   //NALAMCU-82
{
    uint16_t mainPower = (uint16_t)PF_ADC_RAW_TO_MAIN_MV(monet_data.AdcMain);

    if (monet_data.virtualIgnitionTrigger)
    { // If not disabled
        NRF_LOG_RAW_INFO("Virtual: T(%d) S(%d) V(%dmv) H(%dmv) D(%dS).\r",
                         monet_data.virtualIgnitionTrigger,
                         monet_data.virtualIgnitionState,
                         monet_data.virtualIgnitionVoltage,
                         monet_data.virtualIgnitionHysteresis,
                         monet_data.virtualIgnitionDebounceTime);
        NRF_LOG_FLUSH();
        if ((monet_data.virtualIgnitionState == (monet_data.virtualIgnitionTrigger & VIGN_STATE_OFF)) ||
            (monet_data.virtualIgnitionState == VIGN_STATE_NONE))
        { // Check if going Low -> High
            if (mainPower > (monet_data.virtualIgnitionVoltage + monet_data.virtualIgnitionHysteresis))
            {
                if (DebounceVirtualIgnition())
                {
                    // Trigger VIGN interrupt
                    monet_data.virtualIgnitionState = VIGN_STATE_ON;
                    monet_gpio.Intstatus |= MASK_FOR_BIT(INT_VIRTUAL_IGNITION);
                    monet_data.virtualIgnitionDebounceTime = monet_data.virtualIgnitiondDebounce;
                    NRF_LOG_RAW_INFO("INT_VIRTUAL_IGNITION Low -> High.\r");
                    NRF_LOG_FLUSH();
                }
            }
            else
            {
                if (monet_data.virtualIgnitionState != VIGN_STATE_NONE)
                {
                    monet_data.virtualIgnitionDebounceTime = monet_data.virtualIgnitiondDebounce;
                }
            }
        }
        if ((monet_data.virtualIgnitionState == (monet_data.virtualIgnitionTrigger & VIGN_STATE_ON)) ||
            (monet_data.virtualIgnitionState == VIGN_STATE_NONE))
        { // Check if going High -> Low
            if (mainPower < (monet_data.virtualIgnitionVoltage - monet_data.virtualIgnitionHysteresis))
            {
                if (DebounceVirtualIgnition())
                {
                    // Trigger VIGN interrupt
                    monet_data.virtualIgnitionState = VIGN_STATE_OFF;
                    monet_gpio.Intstatus |= MASK_FOR_BIT(INT_VIRTUAL_IGNITION);
                    monet_data.virtualIgnitionDebounceTime = monet_data.virtualIgnitiondDebounce;
                    NRF_LOG_RAW_INFO("INT_VIRTUAL_IGNITION High -> Low.\r");
                    NRF_LOG_FLUSH();
                }
            }
            else
            {
                if (monet_data.virtualIgnitionState != VIGN_STATE_NONE)
                {
                    monet_data.virtualIgnitionDebounceTime = monet_data.virtualIgnitiondDebounce;
                }
            }
        }
    }
}

void SendGlassBreakAlert(void)
{
    uint8_t buf[16] = {0};
    data_time_table_t time_table = {0};
    
    time_table = SecondsToTimeTable(monet_data.glass_break_date_time);

    buf[0] = 'b';
    buf[1] = monet_data.glass_break_mcu_start_capture;
    buf[2] = (uint8_t)(time_table.year & 0xff);
    buf[3] = (uint8_t)((time_table.year >> 8) & 0xff);
    buf[4] = (uint8_t)time_table.month;
    buf[5] = (uint8_t)time_table.day;
    buf[6] = (uint8_t)time_table.hour;
    buf[7] = (uint8_t)time_table.minute;
    buf[8] = (uint8_t)time_table.second;
    BuildFrame(IO_CMD_CHAR_BLE_CTRL, buf, 9);
    monet_data.glass_break_waitMT = 1;
}

uint8_t glass_break_pin_valid(void)
{
    if (monet_data.glass_break_pin_index >= GPIO_LAST)
    {
        return 0;
    }

    if (pf_gpio_read(monet_data.glass_break_pin_index) == GLASS_BREAK_EVENT_PIN_VALID_VALUE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//void ble_glass_break_start_capture(void)
//{
//    uint8_t param[32] = {0};
//    ble_aus_conn_param_t ble_aus_conn_param = {0};
//    data_time_table_t time_table = {0};
//    
//    time_table = SecondsToTimeTable(monet_data.glass_break_date_time);

//    // WARNING: Multi-channel required
//    monet_data.ble_conn_param[0].min_100us = 75;
//    monet_data.ble_conn_param[0].max_100us = 75;
//    monet_data.ble_conn_param[0].latency = 0;
//    monet_data.ble_conn_param[0].timeout_100us = 20000;

//    ble_aus_conn_param.min_ms = monet_data.ble_conn_param[0].min_100us / 10.0;
//    ble_aus_conn_param.max_ms = monet_data.ble_conn_param[0].max_100us / 10.0;
//    ble_aus_conn_param.latency = monet_data.ble_conn_param[0].latency;
//    ble_aus_conn_param.timeout_ms = monet_data.ble_conn_param[0].timeout_100us / 10;

//    NRF_LOG_RAW_INFO("glass_break_start Min(%d) Max(%d) Latency(%d) Timeout(%d)\r",
//                     ble_aus_conn_param.min_ms,
//                     ble_aus_conn_param.max_ms,
//                     ble_aus_conn_param.latency,
//                     ble_aus_conn_param.timeout_ms);
//    NRF_LOG_FLUSH();

//    monet_data.ble_conn_param_update_ok[0] = ble_aus_change_change_conn_params(0, ble_aus_conn_param);

//    param[0] = 0x64;
//    param[1] = 0x00;
//    param[2] = monet_data.glass_break.pic_num;
//    param[3] = monet_data.glass_break.level;
//    param[4] = 4;
//    param[5] = monet_data.glass_break.wb_mode;
//    param[6] = monet_data.ble_mac_addr[0];
//    param[7] = monet_data.ble_mac_addr[1];
//    param[8] = monet_data.ble_mac_addr[2];
//    param[9] = monet_data.ble_mac_addr[3];
//    param[10] = monet_data.ble_mac_addr[4];
//    param[11]= monet_data.ble_mac_addr[5];
//    param[12] = (uint8_t)(time_table.year & 0xff);
//    param[13] = (uint8_t)((time_table.year >> 8) & 0xff);
//    param[14] = (uint8_t)time_table.month;
//    param[15] = (uint8_t)time_table.day;
//    param[16] = (uint8_t)time_table.hour;
//    param[17] = (uint8_t)time_table.minute;
//    param[18] = (uint8_t)time_table.second;

//    monet_bleScommand(param, 19, 0);
//}

//void CheckGlassBreakEvent(uint8_t debounce)
//{
//    if (monet_data.glass_break_waitMT && debounce)
//    {
//        monet_data.glass_break_waitMT++;
//        if (monet_data.glass_break_waitMT >= 3)
//        {
//            SendGlassBreakAlert();
//        }
//    }

//    if (monet_data.glass_break.debounce != CAMERA_GLASS_BREAK_DEBOUNCE_INVALID)
//    {
//        if (glass_break_pin_valid() == 0)
//        {
//            monet_data.glass_break_time_debounce = monet_data.glass_break.debounce;
//            monet_data.glass_break.waitValid = 1;
//        }
//        else if (monet_data.glass_break.waitValid == 1)
//        {
//            if ((monet_data.glass_break_time_debounce > 0) && debounce)
//            {
//                monet_data.glass_break_time_debounce--;
//            }

//            if (monet_data.glass_break_time_debounce == 0)
//            {
//                // monet_gpio.Intstatus |= MASK_FOR_BIT(INT_GLASS_BREAK);
//                monet_data.glass_break_time_debounce = monet_data.glass_break.debounce;
//                monet_data.glass_break_date_time = monet_data.sysRealTimeSeconds;

//                monet_data.glass_break.waitValid = 0;

//                if (monet_data.SleepState != SLEEP_OFF)
//                {
//                    monet_data.glass_break_mcu_start_capture = 1;
//                }
//                else
//                {
//                    monet_data.glass_break_mcu_start_capture = 0;
//                }

//                ble_glass_break_start_capture();
//                
//                SendGlassBreakAlert();

//                NRF_LOG_RAW_INFO(">>>>>>>>CheckGlassBreakEvent debounce(%d).\r", monet_data.glass_break.debounce);
//                NRF_LOG_FLUSH();
//            }
//        }
//    }
//}

void atel_ImuDataInit(void)
{
//    ars_accdata.AccWorkingMode = ACC_MODE_MOTION_DETECTION;
//    ars_accdata.AccMotionStopThreshold = LIS_2GTHRESHOLD_REG_TO_MM_BY_SEC2(config_data.at[1]); // 700;   // Setup a default threshold
//    ars_accdata.AccMotionStartThreshold = LIS_2GTHRESHOLD_REG_TO_MM_BY_SEC2(config_data.at[1]); // 700;  // Setup a default threshold
//    monet_data.AccData.duration = 10; // 10 seconds
//    monet_data.AccData.durationTime = monet_data.AccData.duration;
    // TODO: threshold init value
//    monet_data.AccData.threshold = MAX(ars_accdata.AccMotionStartThreshold,ars_accdata.AccMotionStopThreshold); // Unit: mg
//    monet_data.AccData.motionMode = MOTION_STATE_BOTH;
}

void SendMotionAlert(void) {
    uint8_t pParam[4] = { 0 };
    pParam[0] = 'M';
    pParam[1] = (uint8_t)monet_data.AccData.motionState;
    pParam[2] = (uint8_t)monet_data.AccData.motionMode;
    pParam[3] = 0;
    BuildFrame('x', pParam, 4);
//    monet_data.waitingForMT = 1;
}

//void DebounceMotionMode(void) {
//    NRF_LOG_RAW_INFO("DebounceMotion(%d).\r", monet_data.AccData.durationTime);
//    NRF_LOG_FLUSH();
//    if (monet_data.AccData.durationTime > 0) {
//        monet_data.AccData.durationTime--;
//        if (monet_data.AccData.durationTime == 0) {
//            // Trigger Motion started interrrupt
//            if (monet_data.AccData.motionMode == MOTION_STATE_BOTH) {
//                monet_data.AccData.motionState = (monet_data.InMotion) ? MOTION_STATE_START : MOTION_STATE_STOP;
//            }
//            else {
//                monet_data.AccData.motionState = monet_data.AccData.motionMode;
//            }
//            monet_gpio.Intstatus |= MASK_FOR_BIT(INT_ACCELEROMETER_TRIGGER);
//            SendMotionAlert();
//            monet_data.AccData.durationTime = monet_data.AccData.duration;
//        }
//    }
//}

static void DebounceMotionMode(void) {
    if (monet_data.AccData.durationTime > 0) {
        monet_data.AccData.durationTime--;
        if (monet_data.AccData.durationTime == 0) {
            // Trigger Motion started interrrupt
            if (monet_data.AccData.motionMode == MOTION_STATE_BOTH) {
                monet_data.AccData.motionState = (monet_data.InMotion) ? MOTION_STATE_START: MOTION_STATE_STOP;
            } else {
                monet_data.AccData.motionState = monet_data.AccData.motionMode;
            }
			monet_gpio.Intstatus |= 1 << INT_ACC_SK;
            monet_data.sleepmode = 0; // SIMBAMCU-29
            SendMotionAlert();
            monet_data.AccData.durationTime = monet_data.AccData.duration;
        }
    }
}

//void CheckForMotion(void) {
////    if (monet_data.waitingForMT)
////    {
////        monet_data.waitingForMT++;
////        if (monet_data.waitingForMT >= 3)
////        {
////            SendMotionAlert();
////        }
////    }

//    NRF_LOG_RAW_INFO("Motion(T%d M%d S%d D%d I(%d:0x%x))\r", monet_data.AccData.threshold, monet_data.AccData.motionMode, monet_data.AccData.motionState, monet_data.AccData.duration, monet_data.InMotion, monet_data.IntSrc);
//    NRF_LOG_FLUSH();
//    if (monet_data.InMotion) { // Atleast one motion threshold breech occurred in one second
//		setShouldPollAcc(true);
//        if (monet_data.AccData.threshold &&
//            (((monet_data.AccData.motionMode == MOTION_STATE_START) && (monet_data.AccData.motionState != MOTION_STATE_START)) ||
//            ((monet_data.AccData.motionMode == MOTION_STATE_BOTH) && (monet_data.AccData.motionState != MOTION_STATE_START)))) {
//            DebounceMotionMode();
//        }
//        if (monet_data.AccData.motionMode == MOTION_STATE_STOP) {
//            // Reset motion debounce time
//            monet_data.AccData.durationTime = monet_data.AccData.duration;
//            //monet_data.AccData.motionState = MOTION_STATE_NONE;
//        }
//        if (pf_gpio_read(GPIO_G_SENSOR_INT) == ACC_GYRO_INT_INVALID) { // WCMCU-18
//            monet_data.InMotion = 0; // Clear the motion latch
//            monet_data.IntSrc = 0;
//        } else {
//            // TODO: handle IMU events
//        }
//    }
//    else { // No motion detected
//        if (monet_data.AccData.motionMode == MOTION_STATE_START) {
//            // Reset motion debounce time
//            monet_data.AccData.durationTime = monet_data.AccData.duration;
//            //monet_data.AccData.motionState = MOTION_STATE_NONE;
//        }
//        else if (((monet_data.AccData.motionMode == MOTION_STATE_STOP) && (monet_data.AccData.motionState != MOTION_STATE_STOP)) ||
//            ((monet_data.AccData.motionMode == MOTION_STATE_BOTH) && (monet_data.AccData.motionState != MOTION_STATE_STOP))) {
//            DebounceMotionMode();
//        }
//    }
//}

void CheckForMotion(void) {
    if (monet_data.InMotion) { // At least one motion threshold breech occurred in one second
        setShouldPollAcc(TRUE);
        if (monet_data.AccData.threshold &&
            (((monet_data.AccData.motionMode == MOTION_STATE_START) && (monet_data.AccData.motionState != MOTION_STATE_START)) ||
             ((monet_data.AccData.motionMode == MOTION_STATE_BOTH ) && (monet_data.AccData.motionState != MOTION_STATE_START)))) {
            DebounceMotionMode();
        }
        if (monet_data.AccData.motionMode == MOTION_STATE_STOP) {
            // Reset motion debounce time
            monet_data.AccData.durationTime = monet_data.AccData.duration;
            //monet_data.AccData.motionState = MOTION_STATE_NONE; // PUMAMCU-105
        }
        monet_data.InMotion = 0; // Clear the motion latch
    }
    else { // No motion detected
        if (monet_data.AccData.motionMode == MOTION_STATE_START) {
            // Reset motion debounce time
            monet_data.AccData.durationTime = monet_data.AccData.duration;
            //monet_data.AccData.motionState = MOTION_STATE_NONE; // PUMAMCU-105
        }
        else if (((monet_data.AccData.motionMode == MOTION_STATE_STOP) && (monet_data.AccData.motionState != MOTION_STATE_STOP)) ||
                 ((monet_data.AccData.motionMode == MOTION_STATE_BOTH) && (monet_data.AccData.motionState != MOTION_STATE_STOP))) {
            DebounceMotionMode();
        }
    }
}

//void atel_uart_restore(void)
//{
//    if ((monet_data.uartToBeInit == 1) && (monet_data.uartTXDEnabled == 0)) {
//        monet_data.uartTickCount++;
//        if (monet_data.uartTickCount <= 0) {
//            pf_gpio_write(GPIO_BLE_SLEEP_APP, 1);
//            // NRF_LOG_RAW_INFO("GPIO_BLE_SLEEP_APP High.\r");
//        }
//        else if (monet_data.uartTickCount <= 0) {
//            pf_gpio_write(GPIO_BLE_SLEEP_APP, 0);
//            // NRF_LOG_RAW_INFO("GPIO_BLE_SLEEP_APP Low.\r");
//        }
//        else {
//            // TODO: disbale uart tx rx gpio function
//            pf_uart_mdm_init(UART_BAUDRATE_SELECT, 0);
//            monet_data.uartToBeInit = 0;
//            monet_data.uartTickCount = 0;
//            monet_data.SleepState = SLEEP_OFF;
//            monet_data.SleepStateChange = 1;
//        }
//    }
//}

//void isMdmNeedTobeWakeup(uint32_t tick_ms)
//{
//    static uint8_t pre_mdm_wake_mcu = 0;
//    static uint8_t mdm_wake_mcu = 0;
//    static uint32_t tick_count = 0;

//    tick_count += tick_ms;
//    if (monet_data.SleepState == SLEEP_NORMAL)
//    {
//        if (pre_mdm_wake_mcu != isMDMWakingMCU())
//        {
//            if (0 == pre_mdm_wake_mcu)
//            {
//                mdm_wake_mcu = 1;
//            }
//        }
//    }
//    pre_mdm_wake_mcu = isMDMWakingMCU();

//AGAIN:
//    if (tick_count >= 1000)
//    {
//        tick_count -= 1000;
//    }
//    else
//    {
//        return;
//    }

//    if (monet_data.bbSleepNormalDelay > 0)
//    {
//        monet_data.bbSleepNormalDelay--;
//    }

//    if (isMDMWakingMCU() && (monet_data.SleepState == SLEEP_NORMAL)) {
//        NRF_LOG_RAW_INFO("MDMWakMCU: (%d:%d) SDelay: %d\r", isMDMWakingMCU(), mdm_wake_mcu, monet_data.bbSleepNormalDelay);
//        NRF_LOG_FLUSH();
//        if ((0 == monet_data.bbSleepNormalDelay) && (mdm_wake_mcu))
//        {
//            mdm_wake_mcu = 0;
//            monet_data.SleepAlarm = 0; // Disable the sleep timer
//            // WARNING: This will be called a lot time when modem is not start
//            // NRF_LOG_RAW_INFO("MCU_Wakeup_MDM\r");
//            // NRF_LOG_FLUSH();
//            monet_gpio.Intstatus |= MASK_FOR_BIT(INT_APP_WAKEUP);
//            MCU_Wakeup_MDM();
//            MCU_Wakeup_APP();
//        }
//    }
//    else if ((monet_data.appActive == 0) && (monet_data.uartMdmTXDEnabled == 1)) {
//        NRF_LOG_RAW_INFO("tx 0x7e\r");
//        NRF_LOG_FLUSH();
//        pf_uart_mdm_tx_one(0x7e);
//    }

//    goto AGAIN;
//}

void ble_conn_param_updated_check(void)
{
//    ble_aus_conn_param_t ble_aus_conn_param;
//    uint16_t channel = 0xffff;

//    for (channel = 0; channel < BLE_CHANNEL_NUM_MAX; channel++)
//    {
//        if ((monet_data.ble_conn_param_update_ok[channel] != 0) &&
//            (monet_data.ble_info[channel].connect_status == BLE_CONNECTION_STATUS_CONNECTED))
//        {
//            ble_aus_conn_param.min_ms = monet_data.ble_conn_param[channel].min_100us / 10.0;
//            ble_aus_conn_param.max_ms = monet_data.ble_conn_param[channel].max_100us / 10.0;
//            ble_aus_conn_param.latency = monet_data.ble_conn_param[channel].latency;
//            ble_aus_conn_param.timeout_ms = monet_data.ble_conn_param[channel].timeout_100us / 10;
//            NRF_LOG_RAW_INFO("ble_conn_param_updated_check Min(%d) Max(%d) Latency(%d) Timeout(%d)\r",
//                             ble_aus_conn_param.min_ms,
//                             ble_aus_conn_param.max_ms,
//                             ble_aus_conn_param.latency,
//                             ble_aus_conn_param.timeout_ms);
//            NRF_LOG_FLUSH();
//            monet_data.ble_conn_param_update_ok[channel] = ble_aus_change_change_conn_params(channel, ble_aus_conn_param);

//            if (monet_data.ble_conn_param_update_ok[channel] == 0)
//            {
//                #define BLE_AUS_CONN_PARAM_LEN (11)
//                uint8_t buf[16] = {0};

//                monet_data.ble_conn_param_update_retry[channel] = 0;

//                buf[0] ='p';
//                buf[1] = channel;
//                buf[2] = monet_data.ble_conn_param[channel].min_100us & 0xff;
//                buf[3] = (monet_data.ble_conn_param[channel].min_100us >> 8) & 0xff;
//                buf[4] = monet_data.ble_conn_param[channel].max_100us & 0xff;
//                buf[5] = (monet_data.ble_conn_param[channel].max_100us >> 8) & 0xff;
//                buf[6] = monet_data.ble_conn_param[channel].latency & 0xff;
//                buf[7] = (monet_data.ble_conn_param[channel].latency >> 8) & 0xff;;
//                buf[8] = monet_data.ble_conn_param[channel].timeout_100us & 0xff;
//                buf[9] = (monet_data.ble_conn_param[channel].timeout_100us >> 8) & 0xff;
//                buf[10] = (monet_data.ble_conn_param[channel].timeout_100us >> 16) & 0xff;
//                buf[11] = (monet_data.ble_conn_param[channel].timeout_100us >> 24) & 0xff;

//                BuildFrame(IO_CMD_CHAR_BLE_CTRL, buf, BLE_AUS_CONN_PARAM_LEN + 1);
//            }
//            else
//            {
//                monet_data.ble_conn_param_update_retry[channel] += 1;

//                if (monet_data.ble_conn_param_update_retry[channel] >= BLE_CONN_PARAM_UPDATE_RETRY_MAX)
//                {
//                    NRF_LOG_RAW_INFO("ble_conn_param_updated_check retry fail.\r");
//                    NRF_LOG_FLUSH();
//                    monet_data.ble_conn_param_update_retry[channel] = 0;
//                    ble_aus_change_change_conn_params_disconnect(channel);
//                }
//            }
//        }
//    }
}

void device_ble_status_report(uint32_t delta)
{
    if (monet_data.ble_pairok_notreported == 1)
    {
        uint32_t sensor_mask = scan_list_mask_get();
        const scan_list_t *p_element = NULL;
        uint8_t sensor_type = 0;

        monet_data.ble_pairok_notreported = 0;

        // The event message here does not contain address info, so it is needed to be given manually.
        p_element = scan_list_elem_get(pairing_device_index_get());
        sensor_type = p_element->scan_response.data[ST_POS_IN_SCN_RESP];

        ////////////////////////////////////////////////
        // TODO:
        // 1 need to update the paired device list
        // 2 need to update instance ID algorithm
        ////////////////////////////////////////////////
        memcpy(paired_ble_info[0].addr, p_element->adv_report.peer_addr.addr, BLE_GAP_ADDR_LEN);
        paired_ble_info[0].seq = 1;
        paired_ble_info[0].tn = 1;
        paired_ble_info[0].id = 0;
        paired_ble_info[0].st = sensor_type;

        if (sensor_mask & MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY))
        {
            sensor_mask |= MASK_FOR_BIT(sensor_type);
        }

        // Warning: Move from ble_evt_handler() BLE_GAP_EVT_AUTH_STATUS
        pair_resp_to_mdm_send(1, 0, (uint8_t *)&sensor_mask, sizeof(sensor_mask), &p_element->adv_report.peer_addr);
        dev_paired_cnt_add_one();
        leds_blink(LEDS_BLINK_INTERVAL_SLOW, LEDS_BLINK_DUR_10000MS);
        ble_dg_printf(BLE_DG_LOG_HEADER "device_ble_status_report() paired with sensor_type(%d) ", sensor_type);
        ble_dg_printf(BLE_DG_LOG_HEADER "address (LSB) %02x%02x%02x%02x%02x%02x\r",
                      p_element->adv_report.peer_addr.addr[0],
                      p_element->adv_report.peer_addr.addr[1],
                      p_element->adv_report.peer_addr.addr[2],
                      p_element->adv_report.peer_addr.addr[3],
                      p_element->adv_report.peer_addr.addr[4],
                      p_element->adv_report.peer_addr.addr[5]);
        ble_state_set(BLE_STATE_WT_FOR_ZAZU_CONN);
    }

    if (monet_data.bleConnectionEventDelay)
    {
        monet_data.bleConnectionEventDelay--;
    }
    else
    {
        if (monet_data.bleConnectionEvent)
        {
            ble_connection_status_inform(monet_data.bleConnectionEvent - 1, 1);
            monet_data.bleConnectionEvent = 0;
        }
    }

    if (monet_data.bleDisconnectEvent)
    {
        ble_connection_status_inform(monet_data.bleDisconnectEvent - 1, 0);
        monet_data.bleDisconnectEvent = 0;
    }

    if ((ble_link_target.noreported == 1) && (ble_link_target.report_delay == 0))
    {
        ble_link_action_result_inform(ble_link_target.info.id,
                                      ble_link_target.info.st,
                                      ble_link_target.action,
                                      ble_link_target.result);
        memset(&ble_link_target, 0, sizeof(ble_link_target));
    }
    else if (ble_link_target.report_delay == 1)
    {
        ble_link_target.report_delay_ms += delta;
        if (BLE_LINK_TARGET_DEALY_MAX_MS <= ble_link_target.report_delay_ms)
        {
            ble_link_target.report_delay = 0;
            ble_disconnect_with_peer(0, 1);
            NRF_LOG_RAW_INFO("ble_link_target.report_delay(%d) ble disconnect.\r", ble_link_target.report_delay_ms);
            NRF_LOG_FLUSH();

        }
    }
}

void device_bootloader_enter_dealy(uint32_t delta)
{
    if ((monet_data.deviceBootEnterDelay) &&
       (monet_data.phonePowerOn == 1)) {
       if (monet_data.deviceBootEnterDelay > delta) {
           monet_data.deviceBootEnterDelay -= delta;
       } else {
           monet_data.deviceBootEnterDelay = 0;
           NRF_LOG_RAW_INFO(">>>Enter device bootloader.\r");
           NRF_LOG_FLUSH();
           pf_bootloader_enter();
           NRF_LOG_RAW_INFO(">>>Enter device err.\r");
           NRF_LOG_FLUSH();
       }
   }
}

void setShouldPollAcc(bool value) {
    if (value &&
        !monet_data.phonePowerOn && 
        (monet_data.TiltEventTimer > 0)) {
        // MNT-1494 Setup how long to enable poll if baseband is off and tilt is enabled
        accPollingEnableTimer = monet_data.TiltEventTimer + 2;
    } else {
        accPollingEnableTimer = 0;
    }
    monet_data.bShouldPollAcc = value; // SIMBAMCU-30
}

void CheckInterrupt(void)
{
    if (monet_gpio.Intstatus)
    {
        if (!monet_data.phonePowerOn &&
            !monet_gpio.WDflag &&
			monet_data.BUBX != 1)	// Add condition restriction for shipping mode
        {
            // TODO: Do we need these conditions?
            // if (!monet_data.phonePowerOn &&
            //     !monet_gpio.WDflag &&
            //     ((monet_data.MainState == MAIN_NORMAL) ||
            //         ((monet_data.MainState == MAIN_GONE) &&
            //          ((GPIO_GET(GPIO_TO_INDEX(GPIO_P016_BAT_EN)) == 1) &&
            //           (monet_data.AdcBackup > monet_data.AdcBatC))))) { // LM-44 LM-40 LM-36 LM-22 block critical shutdown wakeups
            // Interrupt was detected so make sure modem wakes up
			pic_turnOnBaseband();
        }
//        else if (!eventIReadyFlag && monet_data.phonePowerOn)
//        {   // Must be sleeping not hibernating
//            monet_data.SleepAlarm = 0; // Disable the sleep timer
//            // WARNING: This will be called a lot time when modem is not start
//            // NRF_LOG_RAW_INFO("MCU_Wakeup_MDM\r");
//            // NRF_LOG_FLUSH();
////            MCU_Wakeup_MDM();
////            MCU_Wakeup_APP();
//        }

//        if (eventIReadyFlag)
//        { // LM-21 Flag is set once the app signals it is ready to receive
//            monet_Icommand();
//        }
    }
}

/* Frame Structure: <0x7E><0x7E>$<L><C><P><CKSM><CR><LF>
 * <0x7E><0x7E> - Preamble
 * $ - 0x24
 * L - Length of <P>, 1 byte
 * C - Command, 1 byte
 * P - Parameters, <L> bytes
 * <CKSM> - Checksum, 1 byte
 * <CR> - 0x0D
 * <LF> - 0x0A
 *
 * nSize - the size of parameters
 */
void BuildFrame(uint8_t cmd, uint8_t * pParameters, uint8_t nSize)
{
    uint8_t  i;
    uint8_t sum;
    atel_ring_buff_t *pQueue;

    pQueue =  &monet_data.txQueueU1;

    #if BLE_DATA_CHANNEL_SUPPORT
    if (IO_CMD_CHAR_BLE_RECV == cmd)
    {
        nSize += 2;
    }
    #endif /* BLE_DATA_CHANNEL_SUPPORT */

    if ((ATEL_RING_BUFF_SIZE_BYTE - pQueue->Size) < (nSize + 8)) {
        // No room
        return;
    }
    // IRQ_OFF;
    AtelWriteRingBuff(pQueue, 0x7E);
    AtelWriteRingBuff(pQueue, 0x7E);
    AtelWriteRingBuff(pQueue, '$');

    pf_print_mdm_uart_tx_init();

    pf_print_mdm_uart_tx('$');

#if USE_CHECKSUM
    #if CMD_ESCAPE
    AtelWriteRingBuffEscape(pQueue, nSize + 1);
    #else
    AtelWriteRingBuff(pQueue, nSize + 1);
    #endif /* CMD_ESCAPE */

    pf_print_mdm_uart_tx(nSize + 1);
#else
    #if CMD_ESCAPE
    AtelWriteRingBuffEscape(pQueue, nSize);
    #else
    AtelWriteRingBuff(pQueue, nSize);
    #endif /* CMD_ESCAPE */

    pf_print_mdm_uart_tx(nSize);
#endif /* USE_CHECKSUM */
    #if CMD_ESCAPE
    AtelWriteRingBuffEscape(pQueue, cmd);
    #else
    AtelWriteRingBuff(pQueue, cmd);
    #endif /* CMD_ESCAPE */

    pf_print_mdm_uart_tx(cmd);

    sum = cmd;

    #if BLE_DATA_CHANNEL_SUPPORT
    if (IO_CMD_CHAR_BLE_RECV == cmd)
    {
    #if CMD_ESCAPE
        AtelWriteRingBuffEscape(pQueue, (uint8_t)monet_data.ble_recv_st);
        AtelWriteRingBuffEscape(pQueue, (uint8_t)monet_data.ble_recv_id);
    #else
        AtelWriteRingBuff(pQueue, (uint8_t)monet_data.ble_recv_st);
        AtelWriteRingBuff(pQueue, (uint8_t)monet_data.ble_recv_id);
    #endif /* CMD_ESCAPE */

        pf_print_mdm_uart_tx((uint8_t)monet_data.ble_recv_st);
        pf_print_mdm_uart_tx((uint8_t)monet_data.ble_recv_id);

    #if USE_CHECKSUM
        sum += (uint8_t)monet_data.ble_recv_st;
        sum += (uint8_t)monet_data.ble_recv_id;
    #endif /* USE_CHECKSUM */
        nSize -= 2;
    }
    #endif /* BLE_DATA_CHANNEL_SUPPORT */

    for(i=0; i < nSize; i++) {
        #if CMD_ESCAPE
        AtelWriteRingBuffEscape(pQueue, pParameters[i]);
        #else
        AtelWriteRingBuff(pQueue, pParameters[i]);
        #endif /* CMD_ESCAPE */

        pf_print_mdm_uart_tx(pParameters[i]);

        sum += pParameters[i];
    }

#if USE_CHECKSUM
    #if CMD_ESCAPE
    AtelWriteRingBuffEscape(pQueue, sum ^ 0xFF);
    #else
    AtelWriteRingBuff(pQueue, sum ^ 0xFF);
    #endif /* CMD_ESCAPE */
#endif /* USE_CHECKSUM */

    pf_print_mdm_uart_tx(sum ^ 0xFF);

    AtelWriteRingBuff(pQueue, 0x0D);
    AtelWriteRingBuff(pQueue, 0x0A); // For easier readability when capturing wire logs

    pf_print_mdm_uart_tx(0x0D);
	
	pf_print_mdm_uart_tx_flush();
    // IRQ_ON;
}

// Check solar charging mode
solar_chg_mode_t solar_chg_mode_get(void)
{
	return (!pf_gpio_read(GPIO_SOLAR_CHARGE_SWITCH)) ? SOLAR_CHG_MODE1 : SOLAR_CHG_MODE2;
}

// Check solar charging mode
void solar_chg_mode_set(solar_chg_mode_t mode)
{
	pf_gpio_write(GPIO_SOLAR_CHARGE_SWITCH, (mode == SOLAR_CHG_MODE1) ? 0 : 1);
}


extern uint32_t pf_uart_mdm_tx_one(uint8_t byte);

// Send data to modem
void send_to_mdm(uint8_t * p_data, uint8_t len)
{
	int i = 0;
	
	for (i = 0; i < len; i++)
		pf_uart_mdm_tx_one(p_data[i]);
}

static volatile com_method_zazu_t com_method_zazu = COM_METHOD_ZAZU_BLE;

// Set communication method with Zazu
void com_method_zazu_set(com_method_zazu_t method)
{
	com_method_zazu = method;
}

// Get communication method with Zazu
com_method_zazu_t com_method_zazu_get(void)
{
	return com_method_zazu;
}

#if (BLE_FUNCTION_ONOFF == BLE_FUNCTION_ON)
uint16_t ble_channel_get_from_mac(uint8_t *p_addr)
{
    uint16_t channel = 0xffff;
    uint16_t i = 0;

    if (p_addr == NULL)
    {
        return 0xffff;
    }

    for (i = 0; i < BLE_CHANNEL_NUM_MAX; i++)
    {
        if ((monet_data.ble_info[i].connect_status <= BLE_CONNECTION_STATUS_MAC_SET) &&
            (monet_data.ble_info[i].connect_status != BLE_CONNECTION_STATUS_NOTVALID))
        {
            uint8_t *p_tmp = monet_data.ble_info[i].mac_addr;
            if ((p_tmp[0] == p_addr[0]) &&
                (p_tmp[1] == p_addr[1]) &&
                (p_tmp[2] == p_addr[2]) &&
                (p_tmp[3] == p_addr[3]) &&
                (p_tmp[4] == p_addr[4]) &&
                (p_tmp[5] == p_addr[5]))
            {
                channel = i;
                break;
            }
        }
    }

    return channel;
}

uint16_t ble_channel_get_from_handler(uint16_t handler)
{
    uint16_t channel = 0xffff;
    uint16_t i = 0;

    for (i = 0; i < BLE_CHANNEL_NUM_MAX; i++)
    {
        // if (monet_data.ble_info[i].connect_status == BLE_CONNECTION_STATUS_CONNECTED)
        {
            if (handler == monet_data.ble_info[i].handler)
            {
                channel = i;
                break;
            }
        }
    }

    return channel;
}

uint16_t ble_connected_handler_get_from_channel(uint16_t channel)
{
    if (monet_data.ble_info[channel].connect_status == BLE_CONNECTION_STATUS_CONNECTED)
    {
        return monet_data.ble_info[channel].handler;
    }
    else
    {
        return 0xffff;
    }
}

uint16_t ble_information_set(uint16_t handler, uint8_t status, uint8_t *p_addr)
{
    uint16_t channel = ble_channel_get_from_mac(p_addr);

    if (channel < BLE_CHANNEL_NUM_MAX)
    {
        monet_data.ble_info[channel].handler = handler;
        monet_data.ble_info[channel].connect_status = status;
    }

    return channel;
}

void ble_connection_channel_init(void)
{
    uint16_t channel = 0;

    for (channel = 0; channel < BLE_CHANNEL_NUM_MAX; channel++)
    {
        monet_data.ble_info[channel].connect_status = BLE_CONNECTION_STATUS_NOTVALID;
        monet_data.ble_info[channel].handler = 0xffff;
    }
}

void ble_connected_channel_clear(void)
{
    uint16_t channel = 0;

    for (channel = 0; channel < BLE_CHANNEL_NUM_MAX; channel++)
    {
        if (monet_data.ble_info[channel].connect_status == BLE_CONNECTION_STATUS_CONNECTED)
        {
            monet_data.ble_info[channel].connect_status = BLE_CONNECTION_STATUS_NOT_CONNECTED;
            monet_data.ble_info[channel].handler = 0xffff;
        }
    }
}

uint16_t ble_connected_channel_num_get(void)
{
    uint16_t channel = 0;
    uint16_t num = 0;

    for (channel = 0; channel < BLE_CHANNEL_NUM_MAX; channel++)
    {
        if (monet_data.ble_info[channel].connect_status == BLE_CONNECTION_STATUS_CONNECTED)
        {
            num++;
        }
    }

    return num;
}

void ble_link_action_result_inform(uint8_t inst_id, uint8_t sensor_typte, uint8_t action, uint8_t result)
{
    uint8_t Param[16] = {0};

    Param[0] = 'l';
    Param[1] = inst_id;
    Param[2] = sensor_typte;
    Param[3] = action;
    Param[4] = result;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 5);
}

void ble_connection_status_inform(uint16_t channel, uint8_t state)
{
    uint8_t buf[16] = {0};

    buf[0] = 'c';
    buf[1] = channel;
    buf[2] = state;
    buf[3] = 1;
    BuildFrame(IO_CMD_CHAR_BLE_CTRL, buf, 4);
}

void monet_bleScommand(uint8_t* pParam, uint8_t Length, uint8_t st, uint8_t id)
{
	if (BLE_DATA_SEND_BUFFER_CELL_LEN < Length)
	{
		pf_log_raw(atel_log_ctl.error_en, "monet_bleScommand Length(%d) err\r", Length);
		NRF_LOG_FLUSH();
		return;
	}

	// int i = 0;
	// NRF_LOG_RAW_INFO("monet_bleScommand() ch %02x, len %02x\r\n", channel, Length);
	// for (i = 0; i < Length; i++)
	// {
	// 	NRF_LOG_RAW_INFO("%02x ", pParam[i]);
	// }
	// NRF_LOG_RAW_INFO("\r\n");
    // NRF_LOG_FLUSH();
	
	// pf_log_raw(atel_log_ctl.io_protocol_en, ">>>>monet_bleScommand len(%d)\r", Length);
	if (is_ble_send_queue_full() == 0)
	{
		if (Length)
		{
			ble_send_data_push(pParam, Length, st, id);
		}
	}
	else
	{
		pf_log_raw(atel_log_ctl.error_en, "monet_bleScommand queue full drop\r");
	}
}

// Read ble address table
void monet_bleCcommand_m(uint8_t** param, uint8_t *p_len)
{
    uint8_t Param[64] = {0};
    uint8_t i = 0, j = 0;

    *param += 1;
    *p_len -= 1;

    Param[0] = 'M';
    Param[1] = 0xff;
    i += 2;
    
    memcpy(Param + 2, monet_data.ble_mac_addr, BLE_MAC_ADDRESS_LEN);
    i += BLE_MAC_ADDRESS_LEN;

    for (j = 0; j < BLE_CHANNEL_NUM_MAX; j++)
    {
        Param[i] = j;
        i++;
        memcpy(Param + i, monet_data.ble_info[j].mac_addr, BLE_MAC_ADDRESS_LEN);
        i += BLE_MAC_ADDRESS_LEN;
    }

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, i);
}

// Set ble address table
void monet_bleCcommand_M(uint8_t** param, uint8_t *p_len)
{
    #define BLE_CONTRIL_M_COMMAND_LEN (7)
    uint16_t channel = 0xffff;
    uint8_t Param[32] = {0};

    *param += 1;
    *p_len -= 1;

    channel = (*param)[0];

    if (channel >= BLE_CHANNEL_NUM_MAX)
    {
        NRF_LOG_RAW_INFO("monet_bleCcommand_M channel(%d) err\r", channel);
        NRF_LOG_FLUSH();
        return;
    }

    if (*p_len < BLE_CONTRIL_M_COMMAND_LEN)
    {
        NRF_LOG_RAW_INFO("monet_bleCcommand_M len(%d) err\r", *p_len);
        NRF_LOG_FLUSH();
        return;
    }

    Param[0] = 'm';
    Param[1] = channel;
    memcpy(Param + 2, *param + 1, BLE_MAC_ADDRESS_LEN);
    Param[8] = 0xff;
    memcpy(Param + 9, monet_data.ble_mac_addr, BLE_MAC_ADDRESS_LEN);

    memcpy(monet_data.ble_info[channel].mac_addr, *param + 1, BLE_MAC_ADDRESS_LEN);
    if (((*param)[1] | (*param)[2] | (*param)[3] | (*param)[4] | (*param)[5] | (*param)[6]) != 0)
    {
        monet_data.ble_info[channel].connect_status = BLE_CONNECTION_STATUS_MAC_SET;
    }
    else
    {
        monet_data.ble_info[channel].connect_status = BLE_CONNECTION_STATUS_NOTVALID;
    }
    // monet_data.ble_info[channel].handler = 0xffff;

    *param += BLE_CONTRIL_M_COMMAND_LEN;
    *p_len -= BLE_CONTRIL_M_COMMAND_LEN;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, BLE_CONTRIL_M_COMMAND_LEN + 1 + BLE_MAC_ADDRESS_LEN + 1);
}

// Enable White list
void monet_bleCcommand_W(uint8_t** param, uint8_t *p_len)
{
    uint8_t Param[16] = {0};

    *param += 1;
    *p_len -= 1;

//    ble_aus_white_list_set();
	ble_aus_set_scan_filter(monet_data.ble_info[0].mac_addr);

    Param[0] = 'w';
    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 1);
}

// Start/stop scan
void monet_bleCcommand_A(uint8_t** param, uint8_t *p_len)
{
    uint8_t Param[16] = {0};
    uint8_t err_code = 0;

    *param += 1;
    *p_len -= 1;

// Comment the below code to avoid unexpected result
//    if ((*param)[0] == 0)			// Stop scan
//    {
//		ble_send_timer_stop_c();
//		scan_stop();
//    }
//    else if ((*param)[0] == 1)		// Start scan
//    {
////		ble_send_recv_init_c();
//		ble_send_timer_start_c();
//		err_code = scan_start();
//    }
//    else							// Re-start scan
//    {
//		ble_send_timer_stop_c();	
//		scan_stop();
//		
//		nrf_delay_ms(100);
//		
////		ble_send_recv_init_c();
//		ble_send_timer_start_c();
//		err_code = scan_start();
//    }

    Param[0] = 'a';
    Param[1] = (*param)[0];
    Param[2] = err_code;

    *param += 1;
    *p_len -= 1;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 3);
}
#endif /* BLE_FUNCTION_ONOFF */

#if BLE_DTM_ENABLE

// BLE DTM test
void monet_bleCcommand_D(uint8_t** param, uint8_t *p_len)
{
    uint8_t Param[16] = {0};
    uint8_t parm_len = 0;

    *param += 1;
    *p_len -= 1;

    Param[0] = 'd';
    Param[1] = (*param)[0];
    parm_len += 2;

    NRF_LOG_RAW_INFO("monet_bleCcommand_D cmd(0x%x)\r", (*param)[0]);

    switch ((*param)[0])
    {
        case 'E':
//			pf_dtm_enter();
			pf_dtm_enter_nala();
			break;

        case 'C':
			pf_dtm_cmd((*param)[1], (*param)[2], (*param)[3], (*param)[4]);
			memcpy(Param + 2, *param + 1, 4);
			*param += 4;
			*p_len -= 4;
			parm_len += 4;
			break;

        case 'X':
			{
				uint8_t resp[] = {0x24, 0x03, 0x33, 0x64, 0x58, 0x10};
				// System would be reset after pf_dtm_exit().But BuildFrame() uses buffer 
				// and cannot send the response to modem even with delay.
				// So new function here is created to send that response.
				send_to_mdm(resp, sizeof(resp));
				nrf_delay_ms(100);
			}
			pf_dtm_exit();
			break;

        default:
			NRF_LOG_RAW_INFO("monet_bleCcommand_D unknown cmd\r");
			*p_len = 0;
			break;
    }

    NRF_LOG_FLUSH();

    *param += 1;
    *p_len -= 1;

    BuildFrame('3', Param, parm_len);
}
#endif /* BLE_DTM_ENABLE */

#if ((BLE_FUNCTION_ONOFF == BLE_FUNCTION_ON) && (BLE_PROFILE_SELECTION == BLE_PROFILE_CAMERA))
// Modify the BLE calender
void monet_bleCcommand_T(uint8_t** param, uint8_t *p_len)
{
    #define DATA_TIME_PARAM_LEN (7)
    data_time_table_t timetable = {0};
    uint8_t Param[16] = {0};

    *param += 1;
    *p_len -= 1;

    if (*p_len < DATA_TIME_PARAM_LEN)
    {
        NRF_LOG_RAW_INFO("monet_bleCcommand_T len(%d) err\r", *p_len);
        NRF_LOG_FLUSH();
        return;
    }

    Param[0] = 't';
    memcpy(Param + 1, *param, DATA_TIME_PARAM_LEN);

    timetable.year = (*param)[0] + (((*param)[1] << 8) & 0xff00);
    timetable.month = (*param)[2];
    timetable.day = (*param)[3];
    timetable.hour = (*param)[4];
    timetable.minute = (*param)[5];
    timetable.second = (*param)[6];

    monet_data.sysRealTimeSeconds = TimeTableToSeconds(timetable);

    *param += DATA_TIME_PARAM_LEN;
    *p_len -= DATA_TIME_PARAM_LEN;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, DATA_TIME_PARAM_LEN + 1);
}
#endif /* BLE_FUNCTION_ONOFF && BLE_PROFILE_SELECTION */

#if GLASS_BREAK_EVENT_EN
void monet_bleCcommand_G(uint8_t** param, uint8_t *p_len)
{
    #define GLASS_BREAK_PARAM_LEN (7)
    uint8_t Param[16] = {0};
    uint8_t param_len = GLASS_BREAK_PARAM_LEN;

    *param += 1;
    *p_len -= 1;

    if (*p_len < (GLASS_BREAK_PARAM_LEN - 1))
    {
        NRF_LOG_RAW_INFO("monet_bleCcommand_G len(%d) err\r", *p_len);
        NRF_LOG_FLUSH();
        return;
    }

    Param[0] = 'g';
    memcpy(Param + 1, *param, GLASS_BREAK_PARAM_LEN);

    monet_data.glass_break.debounce = (*param)[0] | ((*param)[1] << 8);
    monet_data.glass_break.pic_num = (*param)[2] | ((*param)[3] << 8);
    monet_data.glass_break.level = (*param)[4];
    monet_data.glass_break.pin_index = (*param)[5];
    if (*p_len == GLASS_BREAK_PARAM_LEN)
    {
        monet_data.glass_break.wb_mode = (*param)[6];
    }
    else
    {
        param_len = GLASS_BREAK_PARAM_LEN - 1;
        monet_data.glass_break.wb_mode = 0xff;
    }
    monet_data.glass_break.waitValid = 1;
    monet_data.glass_break_time_debounce = monet_data.glass_break.debounce;
    monet_data.glass_break_pin_index = monet_data.glass_break.pin_index;

    *param += param_len;
    *p_len -= param_len;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, param_len + 1);
}

void monet_bleCcommand_B(uint8_t** param, uint8_t *p_len)
{
    *param += 1;
    *p_len -= 1;

    monet_data.glass_break_waitMT = 0;
}
#endif /* GLASS_BREAK_EVENT_EN */

#if (BLE_FUNCTION_ONOFF == BLE_FUNCTION_ON)
void monet_bleCcommand_P(uint8_t** param, uint8_t *p_len)
{
//    #define BLE_AUS_CONN_PARAM_LEN (11)
//    ble_aus_conn_param_t ble_aus_conn_param;
//    uint16_t channel = 0xffff;
//    uint8_t Param[16] = {0};

//    *param += 1;
//    *p_len -= 1;

//    if (*p_len < BLE_AUS_CONN_PARAM_LEN)
//    {
//        NRF_LOG_RAW_INFO("monet_bleCcommand_P len(%d) err\r", *p_len);
//        NRF_LOG_FLUSH();
//        return;
//    }

//    Param[0] = 'p';
//    memcpy(Param + 1, *param, BLE_AUS_CONN_PARAM_LEN);

//    channel = (*param)[0];
//    monet_data.ble_conn_param[channel].max_100us = ((*param)[3] | ((*param)[4] << 8));
//    monet_data.ble_conn_param[channel].min_100us = ((*param)[1] | ((*param)[2] << 8));
//    monet_data.ble_conn_param[channel].latency = ((*param)[5] | ((*param)[6] << 8));
//    monet_data.ble_conn_param[channel].timeout_100us = ((*param)[7] | ((*param)[8] << 8) | ((*param)[9] << 16) | ((*param)[10] << 24));

//    ble_aus_conn_param.min_ms = monet_data.ble_conn_param[channel].min_100us / 10.0;
//    ble_aus_conn_param.max_ms = monet_data.ble_conn_param[channel].max_100us / 10.0;
//    ble_aus_conn_param.latency = monet_data.ble_conn_param[channel].latency;
//    ble_aus_conn_param.timeout_ms = monet_data.ble_conn_param[channel].timeout_100us / 10;

//    NRF_LOG_RAW_INFO("monet_bleCcommand_P Min(%d) Max(%d) Latency(%d) Timeout(%d)\r",
//                     ble_aus_conn_param.min_ms,
//                     ble_aus_conn_param.max_ms,
//                     ble_aus_conn_param.latency,
//                     ble_aus_conn_param.timeout_ms);
//    NRF_LOG_FLUSH();

//    monet_data.ble_conn_param_update_ok[channel] = ble_aus_change_change_conn_params(channel, ble_aus_conn_param);

//    if (monet_data.ble_conn_param_update_ok[channel] != 0)
//    {
//        memset(Param + 2, 0, BLE_AUS_CONN_PARAM_LEN - 1);
//    }

//    *param += BLE_AUS_CONN_PARAM_LEN;
//    *p_len -= BLE_AUS_CONN_PARAM_LEN;

//    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, BLE_AUS_CONN_PARAM_LEN + 1);
}

#if 0	// Old version of '3C' handle
// Check connection state
void monet_bleCcommand_C(uint8_t** param, uint8_t *p_len)
{
    uint16_t channel = 0xffff;
    uint8_t Param[16] = {0};

    *param += 1;
    *p_len -= 1;

    channel = (*param)[0];

    Param[0] = 'c';
    Param[1] = channel;
    Param[2] = 0;
//	if (monet_data.ble_info[channel].connect_status == BLE_CONNECTION_STATUS_CONNECTED)
//	{
//		Param[2] = 1;
//	}
	if (com_method_zazu_get() == COM_METHOD_ZAZU_BLE)
		Param[2] = (ble_aus_ready_state_get_c()== true)? 1: 0;
	else if (com_method_zazu_get() == COM_METHOD_ZAZU_CAN)
		Param[2] = 1;	// Set connection always ready when using CAN. 20210205
	else
		Param[2] = 0;

    *param += 1;
    *p_len -= 1;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 3);
}

#else	// New version of '3C' handle

// Test response for test purpose
void monet_bleCcommand_C_resp_lite(void)
{
	uint8_t Param[8] = {0};
	
	Param[0] = 'c';
	Param[1] = 0;	// channel
	Param[2] = 1;	// conn_state
	Param[3] = 1;	// sensor_type;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 4);
}

// Check connection state
void monet_bleCcommand_C(uint8_t** param, uint8_t *p_len)
{
	const uint8_t PARAM_LEN = 2;	// Pramameter bytes. E.g. command '3L' 0x01 0x02 0x03, PARAM_LEN is 3
    uint8_t Param[8] = {0};
	uint8_t	*p_data = NULL;
	uint8_t	len = 0;
    uint8_t channel = 0xff;		// New name is "Instance ID"
    uint8_t sensor_type = 0;
    uint8_t conn_state = 0;		// Connection state

    *param += 1;
    *p_len -= 1;
	
	p_data = *param;
	len = *p_len;

	if (len < PARAM_LEN)
	{
		*param += len;
		*p_len -= len;
		NRF_LOG_RAW_INFO("monet_bleCcommand_C() parameters count(%u) invalid\r", len);
		return;
	}
	
    channel = p_data[0];
	sensor_type = p_data[1];

	if (com_method_zazu_get() == COM_METHOD_ZAZU_BLE)
		conn_state = (ble_aus_ready_state_get_c()== true)? 1: 0;
	else if (com_method_zazu_get() == COM_METHOD_ZAZU_CAN)
		conn_state = 1;		// Set connection always ready when using CAN. 20210205
	else
		conn_state = 0;

	Param[0] = 'c';
	Param[1] = channel;
	Param[2] = conn_state;
	Param[3] = sensor_type;		// Moving "sensor_type" forward for 1 byte is for old code compatibility
	
    *param += PARAM_LEN;
    *p_len -= PARAM_LEN;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 1+PARAM_LEN+1);
}
#endif	// #if 0	// Old version of '3C' handle

#endif /* BLE_FUNCTION_ONOFF */

// Select data transmission method, BLE or CAN bus.
void monet_bleCcommand_S(uint8_t** param, uint8_t *p_len)
{
    uint8_t Param[16] = {0};
	uint8_t *p_data = (*param)+1;
	uint8_t len = (*p_len)-1;

    if (len == 0 || len > 1)
    {
        NRF_LOG_RAW_INFO("monet_bleCcommand_S() len(%d) err\r", *p_len);
        NRF_LOG_FLUSH();
        return;
    }
    *p_len -= 2;
	
//	if (p_data[0] == 'E')		// Select BLE
	if (p_data[0] == 1)			// Select BLE
	{
		com_method_zazu_set(COM_METHOD_ZAZU_BLE);
		pf_gpio_write(GPIO_CS_12V_EN, 0);
	}
//	else if (p_data[0] == 'N')	// Select CAN bus
	else if (p_data[0] == 2)	// Select CAN bus
	{
		com_method_zazu_set(COM_METHOD_ZAZU_CAN);
		pf_gpio_write(GPIO_CS_12V_EN, 1);
	}
	else
	{
		NRF_LOG_RAW_INFO("monet_bleCcommand_S() param err %x\r", p_data[0]);
        NRF_LOG_FLUSH();
        return;
	}
	
    Param[0] = 's';
    Param[1] = p_data[0];
    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 2);
}

static bool leds_ctrl_by_mcu = false;

void leds_ctrl_take(void)
{
	leds_ctrl_by_mcu = true;
	
	// Turn off all LEDs
	monet_data.ledConf[0].status = 0;
	monet_data.ledConf[1].status = 0;
	pf_gpio_write(GPIO_LED_GREEN, 1);
	pf_gpio_write(GPIO_LED_RED, 1);
	pf_gpio_write(GPIO_LED_ORANGE, 1);
}

bool leds_ctrlled_by_mcu(void)
{
	return leds_ctrl_by_mcu;
}

void leds_ctrl_release(void)
{
	leds_ctrl_by_mcu = false;
}

APP_TIMER_DEF(timer_leds_blink);		// Timer for LEDs blink
#define ADC_CONV_PREP_TIME	20			// Unit in ms. Time 15 ms is tested long enough.
static uint32_t leds_blink_period = 0;
static uint32_t leds_blink_dur = 0;
static uint32_t leds_blink_time_sum = 0;
static bool leds_alway_blink = false;
static leds_blink_callback_t leds_blink_callback = NULL;	// Callback function. Called after LEDs blink duration reached.
static bool leds_are_blinking = false;

static void timer_handler_leds_blink(void * p_context)
{
	if (p_context == (void *)timer_leds_blink)
	{
		if (pic_IsLedOn(0) == true)		// The patterns of 3 LEDs are same
			{pic_turnOffLed(0); pic_turnOffLed(1); pic_turnOffLed(2);}
		else
			{pic_turnOnLed(0); pic_turnOnLed(1); pic_turnOnLed(2);}
		leds_blink_time_sum += leds_blink_period;
		if (leds_alway_blink == false && leds_blink_time_sum >= leds_blink_dur)
		{
			pic_turnOffLed(0);
			pic_turnOffLed(1);
			pic_turnOffLed(2);
			app_timer_stop(timer_leds_blink);
			leds_are_blinking = false;
			if (leds_blink_callback != NULL)
			{
				leds_blink_callback();
				leds_blink_callback = NULL;
			}
		}
	}
}

void leds_blink_callback_enable(leds_blink_callback_t func)
{
	leds_blink_callback = func;
}

// LEDs blink function
// Param. interval: blink interval. Unit in ms
// Param. duration: blink duration. Unit in ms
// Note: If interval < 2 ms, duration is 0, or (interval > duration), LEDs would be turned off
// Example: leds_blink(200, 5000), LEDs on for 100 ms, off for 100 ms, persist for 5000 ms
//          leds_blink(0, 0), turn off all 3 LEDs
void leds_blink(uint32_t interval, uint32_t duration)
{
	app_timer_stop(timer_leds_blink);
	
	// Initialize parameters
	leds_blink_period = 0;
	leds_blink_dur = 0;
	leds_blink_time_sum = 0;
	leds_alway_blink = false;
	pic_turnOffLed(0);
	pic_turnOffLed(1);
	pic_turnOffLed(2);
	leds_are_blinking = false;
	
	if (interval < 2	// At least 2 ms interval
		|| duration == 0 || interval > duration)
		return;
	leds_are_blinking = true;
	leds_blink_period = interval / 2;	// The period of the timer used is half of the interval
	leds_blink_dur = duration;
	if (duration == LEDS_BLINK_DUR_UNLIMITED)
		leds_alway_blink = true;
	app_timer_create(&timer_leds_blink, APP_TIMER_MODE_REPEATED, timer_handler_leds_blink);
	app_timer_start(timer_leds_blink, APP_TIMER_TICKS(leds_blink_period), (void *)timer_leds_blink);
}

// Check LEDs are blinking or not.
// Return value: if true, blinking, else not.
bool are_leds_blinking(void)
{
	return leds_are_blinking;
}

// BLE pair process
void ble_pair_proc(uint32_t sensor_mask)
{
	scan_stop();
	ble_scan_mode_exit();
	scan_list_clear();
//	scan_init_with_param(false, true, NULL);
//	scan_init_with_param(false, false, NULL);
	scan_init_with_param(false, true, NULL);
	scan_list_mask_set(sensor_mask);
	scan_start();
	ble_scan_mode_enter();
	ble_state_set(BLE_STATE_SCAN_STARTED);
	pair_mode_set(true);
	leds_ctrl_take();
	leds_blink(LEDS_BLINK_INTERVAL_FAST, LEDS_BLINK_DUR_UNLIMITED);
	ble_dg_printf(BLE_DG_LOG_HEADER "scaning for pair, sensor mask 0x%x\r\n", sensor_mask);
	NRF_LOG_RAW_INFO("ble_pair_proc() scaning for pair, sensor mask 0x%x\r\n", sensor_mask);
}

// Send pair command responce to Modem
// Param. result: 0: failed, 1: success
// Param. inst_id: instance ID, starts from 0
// Param. p_sensor_mask: sensor mask. 
// Param. sensor_mask_len: sensor mask length. Default 4 bytes
// Param. p_addr: BLE address
void pair_resp_to_mdm_send(uint8_t result, uint8_t inst_id, const uint8_t *p_sensor_mask, uint8_t sensor_mask_len, const ble_gap_addr_t *p_addr)
{
#define BUF_SIZE_PAIR_RESP_TO_MDM	16
	uint8_t Param[BUF_SIZE_PAIR_RESP_TO_MDM] = {0};
	uint8_t resp_len = 3 + sensor_mask_len + BLE_GAP_ADDR_LEN;

	if (resp_len > BUF_SIZE_PAIR_RESP_TO_MDM)	// To avoid array overflow
	{
		NRF_LOG_RAW_INFO("pair_resp_to_mdm_send() resp size (%u) error\r", resp_len);
		return;
	}
	Param[0] = 'r';
    Param[1] = result;		// Result (0: failed, 1: success)
	Param[2] = inst_id;		// Instance ID
	memcpy(&Param[3], p_sensor_mask, sensor_mask_len);
	if (result == 0)
		memset(&Param[3+sensor_mask_len], 0, BLE_GAP_ADDR_LEN);
	else
		memcpy(&Param[3+sensor_mask_len], p_addr->addr, BLE_GAP_ADDR_LEN);
	BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, resp_len);
}

// BLE Pairing
// This command passes the Sensor Mask to IO for BLE pairing
static void monet_bleCcommand_R(uint8_t** param, uint8_t *p_len)
{
	const uint8_t PARAM_LEN = 4;		// Pramameter bytes. E.g. command '3L' 0x01 0x02 0x03, PARAM_LEN is 3
	uint8_t	*p_data = NULL;
	uint8_t	len = 0;
	uint32_t sensor_mask = 0;

    *param += 1;
    *p_len -= 1;

	p_data = *param;
	len = *p_len;
	
	if (len < PARAM_LEN)
	{
		*param += len;
		*p_len -= len;
		NRF_LOG_RAW_INFO("monet_bleCcommand_R() parameters count (%u) invalid\r", len);
		return;
	}
	sensor_mask = p_data[0] + (p_data[1] << 8) + (p_data[2] << 16) + (p_data[3] << 24);
	NRF_LOG_RAW_INFO("monet_bleCcommand_R() mask %x\r", sensor_mask);
	if ((sensor_mask & MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY)) == 0			// Not pair any
		&& (sensor_mask & (MASK_FOR_BIT(SENSOR_MASK_BPOS_CMR) | MASK_FOR_BIT(SENSOR_MASK_BPOS_TMP) | MASK_FOR_BIT(SENSOR_MASK_BPOS_DOOR)) ) == 0 )	// Invalid pairing type
	{
		NRF_LOG_RAW_INFO("monet_bleCcommand_R() invalid sensor mask\r");
		ble_dg_printf(BLE_DG_LOG_HEADER "invalid sensor mask, mask %x\r\n", sensor_mask);
		goto monet_bleCcommand_R_reply;
	}
	ble_pair_proc(sensor_mask);
	goto monet_bleCcommand_R_index_proc;	// There will be response in other place when scan is done.

monet_bleCcommand_R_reply:
	pair_resp_to_mdm_send(0, 0, p_data, PARAM_LEN, NULL);
	
monet_bleCcommand_R_index_proc:
    *param += PARAM_LEN;
    *p_len -= PARAM_LEN;
}

static bool is_mac_addr_null(uint8_t *p_addr, uint8_t len)
{
	int i = 0;
	
	if (p_addr == NULL || len == 0)
		return false;
	for (i = 0; i < len; i++)
	{
		if (p_addr[i] != 0)
			return false;
	}
	return true;
}

// BLE Unpairing
// This command passes the Sensor Mask and MAC Address to IO for BLE unpairing
static void monet_bleCcommand_U(uint8_t** param, uint8_t *p_len)
{
    uint8_t Param[16] = {0};
	uint8_t	*p_data = NULL;
	uint8_t	len = 0;
	uint32_t sensor_mask = 0;
	uint8_t	mac_addr[BLE_GAP_ADDR_LEN] = {0};
	uint8_t result = BLE_RESULT_FAILED;
	pm_peer_id_t current_peer_id = PM_PEER_ID_INVALID;
	uint16_t conn_handle = BLE_CONN_HANDLE_INVALID;
	ret_code_t ret_code = NRF_SUCCESS;
	
    *param += 1;
    *p_len -= 1;

	p_data = *param;
	len = *p_len;
	
	if (len < 4)	// Parameters invalid. There should be 10 bytes parameters
	{
		*param += len;
		*p_len -= len;
		NRF_LOG_RAW_INFO("monet_bleCcommand_U() parameters count(%u) invalid\r", len);
		return;
	}
	sensor_mask = p_data[0] + (p_data[1] << 8) + (p_data[2] << 16) + (p_data[3] << 24);
	NRF_LOG_RAW_INFO("monet_bleCcommand_U() mask %x\r", sensor_mask);
	
	memcpy(&mac_addr[0], &p_data[4], BLE_GAP_ADDR_LEN);
	NRF_LOG_RAW_INFO("monet_bleCcommand_U() mac addr (LSB): ");
	ble_dg_printf(BLE_DG_LOG_HEADER "unpair devices, address (LSB): ");
	for (int i = 0; i < 6; i++)
	{
		NRF_LOG_RAW_INFO("%02X", mac_addr[i]);
		ble_dg_printf("%02X", mac_addr[i]);
	}
	NRF_LOG_RAW_INFO("\r");
	ble_dg_printf("\r\n");

 	NRF_LOG_INFO("monet_bleCcommand_U() disconnect with devices");
	ble_disconnect_with_peer(0, 1);
	memset(&paired_ble_info[0], 0, PAIRED_BLE_INFO_ITEM_SIZE);
 	NRF_LOG_INFO("monet_bleCcommand_U() erase bonds!");
    if (pm_peers_delete() == NRF_SUCCESS)
		result = BLE_RESULT_SUCCESS;
	else
		result = BLE_RESULT_FAILED;
   pf_delay_ms(50);
	
//	if (is_mac_addr_null(mac_addr, sizeof(mac_addr)) == true)
//	{
//		NRF_LOG_RAW_INFO("monet_bleCcommand_U() mac addr is null\r");
//		ble_dg_printf(BLE_DG_LOG_HEADER "unpair device, mac addr is null");
//		result = BLE_RESULT_FAILED;
//		goto monet_bleCcommand_U_reply;
//	}
//	
//	current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
//	while (current_peer_id != PM_PEER_ID_INVALID)
//	{
//		pm_peer_data_bonding_t bonding_data;
//		pm_peer_data_bonding_load(current_peer_id, &bonding_data);
//		NRF_LOG_HEXDUMP_INFO(bonding_data.peer_ble_id.id_addr_info.addr, BLE_GAP_ADDR_LEN);
//		if (memcmp(mac_addr, bonding_data.peer_ble_id.id_addr_info.addr, BLE_GAP_ADDR_LEN) == 0)	// Address matches
//		{
//			ret_code = pm_conn_handle_get(current_peer_id, &conn_handle);
//			if (ret_code == NRF_SUCCESS && conn_handle != BLE_CONN_HANDLE_INVALID)
//			{
////				ret_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION);
//				ret_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//				NRF_LOG_RAW_INFO("monet_bleCcommand_U() ret code %u, conn hdl %x\r", ret_code, conn_handle);
//				NRF_LOG_RAW_INFO("monet_bleCcommand_U() disconnect from the device\r");
//				ble_dg_printf(BLE_DG_LOG_HEADER "disconnect from the device\r\n");
//			}
//			pm_peer_delete(current_peer_id);
//			result = BLE_RESULT_SUCCESS;
//			goto monet_bleCcommand_U_reply;
//		}
//		current_peer_id = pm_next_peer_id_get(current_peer_id);
//	}
	
monet_bleCcommand_U_reply:
	
    Param[0] = 'u';
	Param[1] = result;
	memcpy(&Param[2], p_data, 10);
	NRF_LOG_RAW_INFO("monet_bleCcommand_U() unpair device %s\r\n", (result == BLE_RESULT_SUCCESS) ? "success" : "device not found");
	ble_dg_printf(BLE_DG_LOG_HEADER "unpair device, %s\r\n", (result == BLE_RESULT_SUCCESS) ? "success" : "device not found");
	
    *param += 10;
    *p_len -= 10;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 2 + len);
}

static void monet_bleCcommand_I_resp_send(const ble_gap_addr_t *p_addr, uint8_t sum, uint8_t cur_cnt)
{
	uint8_t Param[16] = {0};
	int i = 0;
	
	Param[0] = 'i';
//    Param[1] = SENSOR_MASK_BPOS_CMR;//0;	// Sensor Type
//    Param[2] = 0;						// Sensor Instance ID
//	memcpy(&Param[3], p_addr->addr, BLE_GAP_ADDR_LEN);	// Paired device BLE mac Address
//	Param[9] = sum;				 		// Total number of paired sensors
//	Param[10] = cur_cnt;				// Sequence number of the sensor
	
	if (is_mac_addr_null(paired_ble_info[0].addr, BLE_MAC_ADDRESS_LEN) == true)
	{
		pm_peer_id_t current_peer_id = PM_PEER_ID_INVALID;
		current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
		if (current_peer_id != PM_PEER_ID_INVALID)
		{
			pm_peer_data_bonding_t bonding_data;
			pm_peer_data_bonding_load(current_peer_id, &bonding_data);
			NRF_LOG_RAW_INFO("monet_bleCcommand_I_resp_send() mac addr\r");
			NRF_LOG_HEXDUMP_INFO(bonding_data.peer_ble_id.id_addr_info.addr, BLE_GAP_ADDR_LEN);
			memcpy(paired_ble_info[0].addr, bonding_data.peer_ble_id.id_addr_info.addr, 6);
			paired_ble_info[0].id = 0;
			paired_ble_info[0].seq = 1;
			paired_ble_info[0].st = 1;
			paired_ble_info[0].tn = 1;
		}
	}
	memcpy(&Param[1], &paired_ble_info[0], PAIRED_BLE_INFO_ITEM_SIZE);

	NRF_LOG_RAW_INFO("monet_bleCcommand_I_resp_send paired device (%u/%u) address (LSB): ", cur_cnt, sum);
	ble_dg_printf(BLE_DG_LOG_HEADER "paired device (%u/%u) address: ", cur_cnt, sum);
//	for (i = 0; i < BLE_GAP_ADDR_LEN; i++)
//	{
//		NRF_LOG_RAW_INFO("%02X", p_addr->addr[i]);
//		ble_dg_printf("%02X", p_addr->addr[i]);
//	}
	for (i = 0; i < BLE_GAP_ADDR_LEN; i++)
	{
		NRF_LOG_RAW_INFO("%02X", paired_ble_info[0].addr[i]);
		ble_dg_printf("%02X", paired_ble_info[0].addr[i]);
	}
	NRF_LOG_RAW_INFO("\r\n");
	ble_dg_printf("\r\n");

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 11);
}

// Query the information of Paired Sensors
static void monet_bleCcommand_I(uint8_t** param, uint8_t *p_len)
{
//	uint8_t sum = 0;
//	uint8_t cnt = 0;
//	pm_peer_id_t current_peer_id = PM_PEER_ID_INVALID;
//	pm_peer_data_bonding_t bonding_data;
	
    *param += 1;
    *p_len -= 1;
	
//	sum = pm_peer_count();
//	NRF_LOG_RAW_INFO("Nala BLE paired device count %u\r\n", sum);
//	ble_dg_printf(BLE_DG_LOG_HEADER "paired device count %u\r\n", sum);
//	current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
//	while (current_peer_id != PM_PEER_ID_INVALID)
//	{
//		pm_peer_data_bonding_load(current_peer_id, &bonding_data);
//		NRF_LOG_HEXDUMP_INFO(bonding_data.peer_ble_id.id_addr_info.addr, BLE_GAP_ADDR_LEN);
//		cnt++;
//		monet_bleCcommand_I_resp_send(&bonding_data.peer_ble_id.id_addr_info, sum, cnt);
	monet_bleCcommand_I_resp_send(NULL, 1, 1);
//		current_peer_id = pm_next_peer_id_get(current_peer_id);
//	}
}

// Send request to Modem to query Zazu sensor info which is stored in Modem
// Note: Modem would reply with '3i' command
void monet_bleCcommand_QI(void)
{
    uint8_t Param[16] = {0};
    Param[0] = 'I';

    NRF_LOG_RAW_INFO("monet_bleCcommand_QI\r");

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 1);
}

// Synchronize the information of Paired Sensors
static void monet_bleCcommand_i(uint8_t** param, uint8_t *p_len)
{
    *param += 1;
    *p_len -= 1;

    memcpy(&paired_ble_info[0], *param, PAIRED_BLE_INFO_ITEM_SIZE);
    NRF_LOG_RAW_INFO("monet_bleCcommand_i(Len:%d st:%d id:%d tn:%d seq:%d):\r", 
                     *p_len,
                     paired_ble_info[0].st,
                     paired_ble_info[0].id,
                     paired_ble_info[0].tn,
                     paired_ble_info[0].seq);
    printf_hex_and_char(*param, *p_len);

    // Warning: All information will be deleted
    if (paired_ble_info[0].tn == 0)
    {
        if (pm_peers_delete() == NRF_SUCCESS)
        {
            pf_delay_ms(10);
        }
        else
        {
            pf_delay_ms(10);
            pm_peers_delete();
        }

        NRF_LOG_RAW_INFO("monet_bleCcommand_i() pm_peers_delete.\r");
    }

    *param += PAIRED_BLE_INFO_ITEM_SIZE;
    *p_len -= PAIRED_BLE_INFO_ITEM_SIZE;
}

// BLE Scan Now
// Start scanning for all compatible sensors within the signal level range (both paired and new)
static void monet_bleCcommand_N(uint8_t** param, uint8_t *p_len)
{
	*param += 1;
	*p_len -= 1;
	
	scan_stop();
	ble_scan_mode_exit();
	scan_list_clear();
	scan_init_with_param(false, true, NULL);
	scan_list_mask_set(MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY));
	scan_start();
	ble_scan_mode_enter();
	ble_state_set(BLE_STATE_SCAN_STARTED);
	scan_now_set(true);
	ble_dg_printf(BLE_DG_LOG_HEADER "scaning...\r\n");
	NRF_LOG_RAW_INFO("monet_bleCcommand_N() scaning...\r\n");
	
//	scan_stop();
//	ble_scan_mode_exit();
//	scan_list_clear();
//	scan_init_with_param(false, true, NULL);
//	scan_list_mask_set(MASK_FOR_BIT(SENSOR_MASK_BPOS_ANY));
//	scan_start();
//	ble_scan_mode_enter();
//	ble_dg_printf("Nala BLE scaning...\r\n");
	
//    uint8_t Param[16] = {0};
//    uint8_t	mac_addr[BLE_GAP_ADDR_LEN] = {0x12, 0x34, 0x56, 0x78, 0x90, 0x12};	// Mac address for test
//    int16_t sig_level = 0x1234;		// Signed value

//    *param += 1;
//    *p_len -= 1;
//	
//    Param[0] = 'n';
//    Param[1] = SENSOR_MASK_BPOS_CMR;	// Sensor Type
//	memcpy(&Param[2], mac_addr, BLE_GAP_ADDR_LEN);	// BLE mac Address
//	Param[8] = sig_level & 0xff;				 	// signal strength value coded on 2 bytes in little endian format
//	Param[9] = (sig_level >> 8) & 0xff;

//    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 10);
}

// Inform Modem to restore LEDs State
// Note: when doing BLE scanning, MCU gets fully control to the 3 LEDs.
//   when the scanning is finished, MCU needs to release the LEDs control to Modem.
void inform_mdm_to_restore_leds_state(void)
{
	uint8_t Param[4] = {0};
	Param[0] = 'e';
	BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 1);
}

// Offer Scan Parameter
// Set BLE Scan Window, Scan Interval and Scan Duration to IO.
static void monet_bleCcommand_O(uint8_t** param, uint8_t *p_len)
{
    uint8_t Param[16] = {0};
	uint8_t	*p_data = NULL;
	uint8_t	len = 0;
	uint32_t scan_window = 0;	// Unit in ms
	uint32_t scan_interval = 0;	// Unit in ms
	uint32_t scan_duration = 0;	// Unit in ms

    *param += 1;
    *p_len -= 1;

	p_data = *param;
	len = *p_len;
	
	if (len < 8)	// Parameters count invalid. There should be 8 bytes parameters
	{
		*param += len;
		*p_len -= len;
		NRF_LOG_RAW_INFO("monet_bleCcommand_O() parameters count(%u) invalid\r", len);
		return;
	}
	scan_window = p_data[0] + (p_data[1] << 8);
	scan_interval = p_data[2] + (p_data[3] << 8);
	scan_duration = p_data[4] + (p_data[5] << 8) + (p_data[6] << 16) + (p_data[7] << 24);
	NRF_LOG_RAW_INFO("monet_bleCcommand_O() scan_window %u, ", scan_window);
	NRF_LOG_RAW_INFO("scan_interval %u, ", scan_interval);
	NRF_LOG_RAW_INFO("scan_duration %u\r", scan_duration);
	ble_dg_printf(BLE_DG_LOG_HEADER "set parameters, scan_window %u, scan_interval %u, scan_duration %u\r\n",
					scan_window, scan_interval, scan_duration);
	
	if (scan_window > 0 && scan_interval > 0 && scan_duration > 0)
		scan_param_set(scan_window, scan_interval, scan_duration);
	
    Param[0] = 'o';
	memcpy(&Param[1], p_data, 8);

    *param += 8;
    *p_len -= 8;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 9);
}

// Enable/disable BLE diagnostic info output to TPMS UART port
static void monet_bleCcommand_H(uint8_t** param, uint8_t *p_len)
{
    uint8_t Param[8] = {0};
	uint8_t	*p_data = NULL;
	uint8_t	len = 0;

    *param += 1;
    *p_len -= 1;

	p_data = *param;
	len = *p_len;
	
	if (len == 1 && p_data[0] == 1)
		ble_dg_info_to_tpms_enable();
	else if (len == 1 && p_data[0] == 0)
		ble_dg_info_to_tpms_disable();
		
    Param[0] = 'h';
    Param[1] = p_data[0];

    *param += 1;
    *p_len -= 1;

    BuildFrame(IO_CMD_CHAR_BLE_CTRL, Param, 2);
}

// Connect/disconnect with Zazu sensor (Establish/Break BLE Link)
static void monet_bleCcommand_L(uint8_t** param, uint8_t *p_len)
{
	const uint8_t PARAM_LEN = 3;		// Pramameter bytes. E.g. command '3L' 0x01 0x02 0x03, PARAM_LEN is 3
	uint8_t	*p_data = NULL;
	uint8_t	len = 0;
	uint8_t inst_id = 0;
	uint8_t sensor_type = 0;
	uint8_t action = 0;

    *param += 1;
    *p_len -= 1;

	p_data = *param;
	len = *p_len;
	
	if (len < PARAM_LEN)
	{
		*param += len;
		*p_len -= len;
		NRF_LOG_RAW_INFO("monet_bleCcommand_L() parameters count(%u) invalid\r", len);
		return;
	}
	
	inst_id = p_data[0];
	sensor_type = p_data[1];
	action = p_data[2];
	NRF_LOG_RAW_INFO("monet_bleCcommand_L() inst_id %u, sensor_type %u, action %u\r", inst_id, sensor_type, action);
	ble_dg_printf(BLE_DG_LOG_HEADER "to %s with sensor type %u inst_id %u\r\n", (action == 1) ? "connect" : "disconnect", sensor_type, inst_id);

    if ((ble_link_target.action == 1) && (action == 1))
    {
        ble_link_target.attempt++;

        NRF_LOG_RAW_INFO("monet_bleCcommand_L() device is trying to build link(%d).\r", ble_link_target.attempt);
        if (ble_link_target.attempt < BLE_LINK_TARGET_ATTEMPT_COUNT)
        {
            return;
        }
        else
        {
            ble_link_target.attempt = 0;
        }
    }

    // Warning: Instance ID and Sensor Type not managed.
    memset(&ble_link_target, 0, sizeof(ble_link_target));
    memcpy(&ble_link_target.info, &paired_ble_info[0], sizeof(ble_link_target.info));
    ble_link_target.action = action;
    if (action == 1)
    {
        if (monet_data.bleConnectionStatus == 0)
        {
            ble_disconnect_with_peer(0, 1);

            nrf_delay_ms(10);

            if (is_mac_addr_null(paired_ble_info[0].addr, BLE_MAC_ADDRESS_LEN) == false)
			{
				NRF_LOG_RAW_INFO("monet_bleCcommand_L() connect with dev address: \r");
                printf_hex_and_char(paired_ble_info[0].addr, BLE_MAC_ADDRESS_LEN);
				scan_init_with_param(true, true, paired_ble_info[0].addr);
                if (scan_start() != 0)
                {
                    ble_link_action_result_inform(inst_id, sensor_type, action, 0);
                    ble_link_target.action = 0;
                }
			}
			else
			{
				// pm_peer_id_t current_peer_id = PM_PEER_ID_INVALID;
				// current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
				// if (current_peer_id != PM_PEER_ID_INVALID)
				// {
				// 	pm_peer_data_bonding_t bonding_data;
				// 	pm_peer_data_bonding_load(current_peer_id, &bonding_data);
				// 	NRF_LOG_RAW_INFO("monet_bleCcommand_L() connect with dev address (bonded): ");
				// 	NRF_LOG_HEXDUMP_INFO(bonding_data.peer_ble_id.id_addr_info.addr, BLE_GAP_ADDR_LEN);
				// 	scan_init_with_param(true, true, bonding_data.peer_ble_id.id_addr_info.addr);
				// }
				// else
				{
					NRF_LOG_RAW_INFO("monet_bleCcommand_L() Failed to connect with the device MAC:\r");
                    printf_hex_and_char(paired_ble_info[0].addr, BLE_MAC_ADDRESS_LEN);
                    monet_bleCcommand_QI();
                    monet_bleCcommand_QI();
                    ble_connection_status_inform(0, 0);		// Failed to connect with the address
                    ble_link_action_result_inform(inst_id, sensor_type, action, 0);
                    ble_link_target.action = 0;
				}
			}
        }
        else
        {
            ble_connection_status_inform(0, 1);
            ble_link_action_result_inform(inst_id, sensor_type, action, 1);
            ble_link_target.action = 0;
        }
    }
    else if (action == 0)
    {
        ble_disconnect_with_peer(0, 1);
        ble_link_action_result_inform(inst_id, sensor_type, action, 1);
        ble_link_target.action = 0;
    }

    NRF_LOG_FLUSH();
}

void monet_bleCcommand(uint8_t* pParam, uint8_t Length)
{
    uint8_t left_len = Length;
    uint8_t *p_tmp = pParam;

// BLE_C_AGAIN:
    switch (*p_tmp)
    {
        #if (BLE_FUNCTION_ONOFF == BLE_FUNCTION_ON)
        case 'm':	// Read ble address table
        monet_bleCcommand_m(&p_tmp, &left_len);
        break;

        case 'M':	// Set ble address table
        monet_bleCcommand_M(&p_tmp, &left_len);
        break;

        case 'W':	// Enable White list
        monet_bleCcommand_W(&p_tmp, &left_len);
        break;

        case 'A':	// Start/stop scan
        monet_bleCcommand_A(&p_tmp, &left_len);
        break;
        #endif /* BLE_FUNCTION_ONOFF */

        #if BLE_DTM_ENABLE
        case 'D':
        monet_bleCcommand_D(&p_tmp, &left_len);
        break;
        #endif /* BLE_DTM_ENABLE */

        #if ((BLE_FUNCTION_ONOFF == BLE_FUNCTION_ON) && (BLE_PROFILE_SELECTION == BLE_PROFILE_CAMERA))
        case 'T':	// Modify the BLE calender
        monet_bleCcommand_T(&p_tmp, &left_len);
        break;
        #endif /* BLE_FUNCTION_ONOFF && BLE_PROFILE_SELECTION */

        #if GLASS_BREAK_EVENT_EN
        case 'G':
        monet_bleCcommand_G(&p_tmp, &left_len);
        break;

        case 'B': // do not response
        monet_bleCcommand_B(&p_tmp, &left_len);
        break;
        #endif /* GLASS_BREAK_EVENT_EN */

        #if (BLE_FUNCTION_ONOFF == BLE_FUNCTION_ON)
//        case 'P':
////		monet_bleCcommand_P(&p_tmp, &left_len);
//		left_len = 0;
//        break;

        case 'C':	// Check connection state
        monet_bleCcommand_C(&p_tmp, &left_len);
        break;
        #endif /* BLE_FUNCTION_ONOFF */

		case 'S':	// Select data transmission method, BLE or CAN bus.
        monet_bleCcommand_S(&p_tmp, &left_len);
        break;
		
		case 'R':	// BLE Pairing
        monet_bleCcommand_R(&p_tmp, &left_len);
        break;
		
		case 'U':	// BLE Unpairing
        monet_bleCcommand_U(&p_tmp, &left_len);
        break;
		
		case 'I':	// Query the information of Paired Sensors
        monet_bleCcommand_I(&p_tmp, &left_len);
        break;
		
		case 'i':	// Update Zazu sensor info
        monet_bleCcommand_i(&p_tmp, &left_len);
        break;
		
		case 'N':	// BLE Scan Now
        monet_bleCcommand_N(&p_tmp, &left_len);
        break;
		
		case 'O':	// Offer Scan Parameter
        monet_bleCcommand_O(&p_tmp, &left_len);
        break;
		
		case 'H':	// Enable/disable BLE diagnostic info output to TPMS UART port
        monet_bleCcommand_H(&p_tmp, &left_len);
        break;
		
		case 'L':	// Connect/disconnect with Zazu sensor
        monet_bleCcommand_L(&p_tmp, &left_len);
        break;
		
        default:
        NRF_LOG_RAW_INFO("monet_bleCcommand unknown cmd\r");
        NRF_LOG_FLUSH();
        left_len = 0;
        return;
    }

    // if (left_len > 0)
    // {
    //     goto BLE_C_AGAIN;
    // }
}

APP_TIMER_DEF(pf_orng_led_blink_timer);	// Timer for orange LED blink
#define ORNG_LED_BLINK_TIME	100			// Unit in ms.

static uint32_t orange_led_on_time = 0;		// Unit in ms
static uint32_t orange_led_off_time = 0;	// Unit in ms
static bool orange_led_is_on = false;
static bool orange_led_blink_repeatedly = false;

static void timer_org_led_blink_handler(void * p_context)
{
	if (p_context == (void *)pf_orng_led_blink_timer)
	{
		if (orange_led_blink_repeatedly != true)
			pic_turnOffLed(2);
		else
		{
			if (orange_led_is_on == true)
			{
				pic_turnOffLed(2);
				orange_led_is_on = false;
				app_timer_create(&pf_orng_led_blink_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_org_led_blink_handler);
				app_timer_start(pf_orng_led_blink_timer, APP_TIMER_TICKS(orange_led_off_time), (void *)pf_orng_led_blink_timer);
			}
			else
			{
				pic_turnOnLed(2);
				orange_led_is_on = true;
				app_timer_create(&pf_orng_led_blink_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_org_led_blink_handler);
				app_timer_start(pf_orng_led_blink_timer, APP_TIMER_TICKS(orange_led_on_time), (void *)pf_orng_led_blink_timer);
			}
		}
	}
}

void monet_timer30s(void)
{
//	if(monet_data.chargerBlink){
	if((monet_data.chargerBlink || no_external_power() == true) &&
		monet_data.bBattMarkedCritical != true)
	{
		flashOrange(10, 0);		// Turned on for 10*10 ms, then turned off
	}
}

static bool test_mode_state = false;

void test_mode_enter(void)
{
	test_mode_state = true;
}

bool is_in_test_mode(void)
{
	return test_mode_state;
}

void test_mode_leave()
{
	test_mode_state = false;
}

void flashOrange(BYTE dur1, BYTE dur2)
{
//	BYTE pParam[4];
//	pParam[1]=dur1;
//	pParam[2]=dur2;
//	pParam[0]=dur2? 0x82 : 0x2;
//	monet_startLED(pParam,  0);
	
	// Re-write orange LED blink function
	uint32_t led_on_time = 0;	// Unit in ms
	uint32_t led_off_time = 0;	// Unit in ms
	
//	if (is_in_test_mode() == true)	// Disable old LED control method
	if (leds_ctrlled_by_mcu() == true ||	// MCU fully controlls the 3 LEDs
		is_in_test_mode() == true)			// Test mode. Disable old LED control method. Only 1 LED is turned on every time.
	{
		app_timer_stop(pf_orng_led_blink_timer);
		orange_led_blink_repeatedly = false;
		return;
	}
	
	if (dur1 == 0)
	{
		app_timer_stop(pf_orng_led_blink_timer);
		pic_turnOffLed(2);
		orange_led_blink_repeatedly = false;
	}
	else if (dur1 == 255)
	{
		app_timer_stop(pf_orng_led_blink_timer);
		pic_turnOnLed(2);
		orange_led_blink_repeatedly = false;
	}
	else if (dur2 == 0)	// It is not repeatedly blinking
	{
		app_timer_stop(pf_orng_led_blink_timer);
		led_on_time = dur1 * 10;
		pic_turnOnLed(2);
		app_timer_create(&pf_orng_led_blink_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_org_led_blink_handler);
		app_timer_start(pf_orng_led_blink_timer, APP_TIMER_TICKS(led_on_time), (void *)pf_orng_led_blink_timer);
		orange_led_blink_repeatedly = false;
	}
	else
	{
		if (orange_led_blink_repeatedly != true)
		{
			orange_led_blink_repeatedly = true;
			orange_led_on_time = dur1 * 10;
			orange_led_off_time = dur2 * 10;
			pic_turnOnLed(2);
			orange_led_is_on = true;
			app_timer_create(&pf_orng_led_blink_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_org_led_blink_handler);
			app_timer_start(pf_orng_led_blink_timer, APP_TIMER_TICKS(orange_led_on_time), (void *)pf_orng_led_blink_timer);
		}
		else
		{
			led_on_time = dur1 * 10;
			led_off_time = dur2 * 10;
			if (led_on_time != orange_led_on_time ||
				led_off_time != orange_led_off_time)
			{
				orange_led_on_time = led_on_time;
				orange_led_off_time = led_off_time;
			}
		}
	}
		
}

bool no_external_power(void)
{
	uint16_t vol_main = 0, vol_aux = 0, vol_solar = 0;
	bool result = 0;
	
	vol_main = adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR);
	vol_aux = adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR);
	vol_solar = adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR);
	result = (vol_main < MAIN_ADC_MAIN_VALID &&	
			  vol_aux < MAIN_ADC_MAIN_VALID &&
			  vol_solar < MAIN_ADC_MAIN_VALID);// There is no external power supply
	return result;
}

bool only_solar_power(void)
{
	uint16_t vol_main = 0, vol_aux = 0, vol_solar = 0;
	bool result = 0;
	
	vol_main = adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR);
	vol_aux = adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR);
	vol_solar = adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR);
	result = (vol_main < MAIN_ADC_MAIN_VALID &&	
			  vol_aux < MAIN_ADC_MAIN_VALID &&
			  vol_solar > MAIN_ADC_MAIN_VALID);		// Only solar power
	return result;
}

//Obtain the value of solar power for SOLAR_GET_POWER_COUNT times, and then calculate the average value
//return   1----solar charger is invalid
//return   0----solar charger is valid
bool solar_charge_invalid(void)
{
	bool result = 0;

	count_solar_p++;
	if(count_solar_p % (SOLAR_GET_POWER_COUNT+1) == 0)
	{
		aver_solar_p = sum_solar_p/SOLAR_GET_POWER_COUNT;
		sum_solar_p = 0.0;
		count_solar_p = 0;
	}
	else
	{
		sum_solar_p += mppt_output_p_get();
	}
	
//	result = (0 < aver_solar_p  && aver_solar_p < SOLAR_CHARGE_POWER_VALID);		//
	result = (/*0 < aver_solar_p  && */aver_solar_p < SOLAR_CHARGE_POWER_VALID);		//
	NRF_LOG_RAW_INFO("solar_charge_invalid::count=%d,sumpow=%d,averpow=%d,result=%d.\r\n",count_solar_p,(int32_t)sum_solar_p,(int32_t)aver_solar_p,result);
	NRF_LOG_FLUSH();
	return result;
}

void flashChargingStatus(void)
{
	BYTE stat;

	monet_data.chargerBlink = 0;
	// Check charger state
//	stat = GPIO_PinInGet(egpio_config[PIN_STAT1].port, egpio_config[PIN_STAT1].pin) << 1;
//	stat |= GPIO_PinInGet(egpio_config[PIN_STAT2].port, egpio_config[PIN_STAT2].pin);
	stat = getChargerState();
	
	NRF_LOG_RAW_INFO("CHG state: (%u, %u), monet_data.ChargerRestartValue: %u,%u\r", 
						stat, monet_data.ChargerStatus, monet_data.ChargerRestartValue,monet_data.ChargerDelay);
	//////////////////////////////////////////////////////////////////////////////////////////////
	//                                                                    stat1 stat2    stat
	// Not Specified														0     0		   0
	// Charge-in-progress													0     1		   1
	// Charge complete														1     0		   2
	// Charge suspend, timer fault, overvoltage, sleep mode, battery absent	1     1		   3
	//////////////////////////////////////////////////////////////////////////////////////////////
	
	switch (stat) {
	case 0:
		// Should not happen
		flashOrange(0, 255);
		break;
	case 1:
		// Charge in progress
		if (is_main_aux_vol_low() == false)
			flashOrange(50, 0);
		monet_data.ChargerDelay = 0;
		break;
	case 2:
		// Charge complete
		if (monet_data.ChargerStatus == CHARGER_ON) {
			if (is_main_aux_vol_low() == false)
				flashOrange(255, 0);
			if (monet_data.ChargerRestartValue) {
				monet_data.ChargerDelay++;
				if (monet_data.ChargerDelay >= CHARGER_DELAY) {
                    monet_data.PrevChargerStatus = CHARGER_OFF; // PUMAMCU-102 Ensure Charger remains off when power state changes
                    if(recharge_start == true)
                    {
                    	recharge_start = false;  //recharge has ended.
                    	flashOrange(0, 255);      // NALAMCU-59 and NALAMCU-73:  orange led should be on solid for 10mins and then turn off indicating that charging is disabled.
                    }
                    setChargerOff();
				}
			}
		}
		break;
	case 3:	
		if (monet_data.ChargerStatus == CHARGER_OFF) {
			flashOrange(0, 255);
		}
		else {
            if ((adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR)   > ADC_MAIN_TH) ||
                (adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR)     > ADC_MAIN_TH) ||
                (adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR) > ADC_MAIN_TH)) {
				// We should be charging or charged
					flashOrange(25, 25);
			}
			else {
				monet_data.chargerBlink = 1;
			}
		}
	}
}

void flashLED(void)
{
	BYTE pParam[4];
	static short count=0;
	short t1=60;

	pParam[1]=100;
	pParam[2]=0;

	if(count==0)
	{
		pParam[0]=1;
		monet_startLED(pParam, 0);
	}
	else if(count==1)
	{
		pParam[0]=1;
		monet_startLED(pParam, 0);
	}
	count++;
	if(count == t1)count=0;
}

#define ORG_LED_INDEX	2

void orange_led_ctrl(uint8_t *pParam, uint8_t Length)
{
	if (Length != 3 || pParam == NULL)	// Index 2 is orange LED
	{
		NRF_LOG_RAW_INFO("orange_led_ctrl(), error, param L%u P%x\r", Length, pParam);
		NRF_LOG_FLUSH();
		return;
	}
	if (pParam[0] != ORG_LED_INDEX)
		goto ORANGE_LED_CTRL_ERR_PROC;
	
	if (pParam[1] == 0 && pParam[2] == 0)
	{
		test_mode_enter();
		pic_turnOffLed(pParam[0]);
	}
	else if (pParam[1] == 255 && pParam[2] == 0)
	{
		test_mode_enter();
		pic_turnOnLed(pParam[0]);
	}
	else if ((pParam[1] == 255 && pParam[2] == 255))
	{
		pic_turnOffLed(pParam[0]);
		test_mode_leave();
	}
	else
	{
ORANGE_LED_CTRL_ERR_PROC:
		NRF_LOG_RAW_INFO("orange_led_ctrl(), error, param invalid %u %u %u\r", pParam[0], pParam[1], pParam[2]);
		NRF_LOG_FLUSH();
		return;
	}
	
}

void monet_startLED(BYTE* pParam, BYTE status)
{
    BYTE led = pParam[0] & 0x7F;
	
    if(led >= NUM_OF_LED) led = NUM_OF_LED -1;
    /*reset LED*/
    monet_data.ledConf[led].status = status;

    /*config LED*/
    if ( pParam[0] & 0x80) {
        monet_data.ledConf[led].status |= LED_REPEAT;
    }
    monet_data.ledConf[led].t_on = (pParam[1]*10+TIME_UNIT-1)/(TIME_UNIT);
    monet_data.ledConf[led].t_off = (pParam[2]*10+TIME_UNIT-1)/(TIME_UNIT);

	switch (pParam[1]) {
    case 0:
        pic_turnOffLed(led);
        break;
    case 255:
        pic_turnOnLed(led);
        break;
    default:
        monet_data.ledConf[led].status |= LED_RUNNING;
        break;
    }
}

/* @: Bootloader command in wrong mode */
void monet_AtSigncommand( void )
{
    uint8_t ReportData;

    ReportData = 0xFF;
    BuildFrame('@', &ReportData, 1);
}

// Get Nordic chip temperature. Unit in Celsius Degree.
int32_t chip_temp_get(void)
{
#if 0	// When there is no SoftDevice
	int32_t temp = 0;
	
	nrf_temp_init();
	NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */
	/* Busy wait while temperature measurement is not finished, you can skip waiting if you enable interrupt for DATARDY event and read the result in the interrupt. */
	/*lint -e{845} // A zero has been given as right argument to operator '|'" */
	while (NRF_TEMP->EVENTS_DATARDY == 0)
	{ /* Do nothing. */ };
	NRF_TEMP->EVENTS_DATARDY = 0;
	temp = (nrf_temp_read() / 4);	// nrf_temp_read() output unit is in 0.25 Celsius degree.
	NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */
	return temp;
#else	// When there is SoftDevice
	int32_t temp = 0;
	sd_temp_get(&temp);		// Temperature in 0.25 degrees Celsius
	return (temp/4);
#endif
}

// Convert ADC value to voltage
// Param. adc_value, ADC value
// Param. res_factor, resistance factor
// Return value: voltage value. Unit in mV
uint16_t adc_to_vol_conv(uint16_t adc_value, double res_factor)
{
	return (uint16_t)(1.0 * adc_value / ADC_MAX * ADC_FACTOR * res_factor);
}

// ADC report
void monet_requestAdc(uint8_t adc)
{
	uint8_t pParm[16];
	uint8_t index;
	uint16_t voltage = 0;		// Unit in mV
	uint16_t temp_m_dgree = 0;	// Unit in centi degree Celsius (0.01 degree)
	
	index = 0;
	pParm[index++] = adc;		// Report the MASK back to caller
	if (adc & MASK_ADC_LIGHT) {
#if (MDM_REQUIRE_ADC_RAW == 1)
		pParm[index++] = monet_data.AdcAux & 0xff;
		pParm[index++] = (monet_data.AdcAux >> 8) & 0xff;
#else
//		voltage = (uint16_t)(1.0 * monet_data.AdcAux / ADC_MAX * ADC_FACTOR * VOL_AUX_FACTOR);
		voltage = adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR);
		pParm[index++] = voltage & 0xff;
		pParm[index++] = (voltage >> 8) & 0xff;
#endif
	}
	if (adc & MASK_ADC_BAT) {
#if (MDM_REQUIRE_ADC_RAW == 1)
		pParm[index++] = monet_data.AdcBackup & 0xff;
		pParm[index++] = (monet_data.AdcBackup >> 8) & 0xff;
#else
//		voltage = (uint16_t)(1.0 * monet_data.AdcBackup / ADC_MAX * ADC_FACTOR * VOL_BAT_FACTOR);
		voltage = adc_to_vol_conv(monet_data.AdcBackup, VOL_BAT_FACTOR);
		pParm[index++] = voltage & 0xff;
		pParm[index++] = (voltage >> 8) & 0xff;
#endif
	}
	if (adc & MASK_ADC_MAIN) {
#if (MDM_REQUIRE_ADC_RAW == 1)
		pParm[index++] = monet_data.AdcMain & 0xff;
		pParm[index++] = (monet_data.AdcMain >> 8) & 0xff;
#else
//		voltage = (uint16_t)(1.0 * monet_data.AdcMain / ADC_MAX * ADC_FACTOR * VOL_MAIN_FACTOR);
		voltage = adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR);
		pParm[index++] = voltage & 0xff;
		pParm[index++] = (voltage >> 8) & 0xff;
#endif
	}
	if (adc & MASK_ADC_SOLAR) {
#if (MDM_REQUIRE_ADC_RAW == 1)
		pParm[index++] = monet_data.AdcSolar & 0xff;
		pParm[index++] = (monet_data.AdcSolar >> 8) & 0xff;
#else
//		voltage = (uint16_t)(1.0 * monet_data.AdcSolar / ADC_MAX * ADC_FACTOR * VOL_SOLAR_FACTOR);
		voltage = adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR);
		pParm[index++] = voltage & 0xff;
		pParm[index++] = (voltage >> 8) & 0xff;
#endif
	}
	if (adc & MASK_ADC_TEMP) {
		int16_t temp = (int16_t)chip_temp_get();
#if (MDM_REQUIRE_ADC_RAW == 1)
		pParm[index++] = temp & 0xff;
		pParm[index++] = (temp >> 8) & 0xff;
#else
		temp_m_dgree = temp * 100;
		pParm[index++] = temp_m_dgree & 0xff;
		pParm[index++] = (temp_m_dgree >> 8) & 0xff;
#endif
	}
	
	BuildFrame('a', pParm, index);
}

// WD timer and baseband watchdog setting
void monet_Bcommand(uint8_t* pParam, uint8_t Length)
{
    uint32_t value;

    value = (pParam[1]<<8) + pParam[0];
    if(value > 0)
    {
        monet_gpio.WDtimer = value;
        monet_gpio.WDflag = 0;
        monet_conf.WD.Reload = value;
        gBWD = monet_conf.WD.Reload;
        // save_config();
    } else {
        pParam[0] = (uint8_t)((monet_conf.WD.Reload) & 0xFF);
        pParam[1] = (uint8_t)(((monet_conf.WD.Reload) >> 8) & 0xFF);
    }
    BuildFrame('b', pParam, 2);
}

/* C: UART Configuration. App UART and peri UART */
void monet_Ccommand(uint8_t* pParam, uint8_t Length)
{
	volatile uint32_t count = 0;

    // For LEUART check for valid baudrate: set to the default 9600 if unsupported baudrate selected
    if (Length == 3 && pParam[2] == 1 && ((pParam[0] == 6) || (pParam[0] == 7))) {
        pParam[0] = 5;
    }
    BuildFrame('c', pParam, Length);      // Respond firstly
		
	// Sendout all data for Modem in queue
	if (monet_data.uartMdmTXDEnabled != 0)
    {
        while (atel_io_queue_process());	// Put data to buffer
    }
	while ((pf_uart_mdm_tx_queue_is_empty() != true)  && (count < 10)) {	// Send all data in buffer
		delay_10ms_wds(1);
		count++; // Dont allow loop to get stuck
    }
	
	if (Length == 2) {
		if (monet_data.uartPeriTXDEnabled)
			pf_uart_peri_deinit();
        pf_uart_peri_init(pParam[0], pParam[1]);
    } else if (Length == 3) {
        if (pParam[2] == 0) {
			if (monet_data.uartPeriTXDEnabled)
				pf_uart_peri_deinit();
            pf_uart_peri_init(pParam[0], pParam[1]);
        } else if (pParam[2] == 1) {
            // Warning: NALA MCU MDM UART Will not controled by APP
			if (monet_data.uartMdmTXDEnabled)
				pf_uart_mdm_deinit();
            pf_uart_mdm_init(pParam[0], pParam[1]);
        }
    }
}

// Enable peri UART and send data to peri UART
void monet_Dcommand(uint8_t* pParam, uint8_t Length)
{
	uint8_t i;
	
    monet_data.dataframecounter = pParam[0];
    for (i = 0; i < Length; i++)
		pf_uart_peri_tx_one(pParam[i]);
    monet_data.waitOnCSTxEmpty = 1;
    monet_data.waitOnCSTxLen = Length;

    //BuildFrame('d', pParam, 1);
}

// Send data to peri UART
void monet_dcommand(uint8_t* pParam, uint8_t Length)
{
	uint8_t i;

    for (i = 0; i < Length; i++)
		pf_uart_peri_tx_one(pParam[i]);
    BuildFrame('D', pParam, 4);
}

/* E: External GPIO Event Registration */
void monet_Ecommand(uint8_t* pParam)
{
	uint8_t gpio;
    uint8_t nValue;
    uint8_t ReportData[2];

    gpio = pParam[0];
    nValue = pf_gpio_read(egpio_config[gpio]);
    monet_data.gpioEv[gpio].gpioCurrent = nValue;           // Set the initial state

    monet_data.gpioEv[gpio].gpio     = gpio;
    monet_data.gpioEv[gpio].report   = (GpioReport)pParam[1];
    monet_data.gpioEv[gpio].debounce = pParam[2];
    monet_data.gpioEv[gpio].debouncing = 0;
	
	// To avoid repeated 'e' report sent to Modem, 
	// because there would be also 'e' command sent when GPIO event triggered.
	if (is_in_cs_prov_test_mode() == true)
		return;
	
    ReportData[0] = gpio;
    ReportData[1] = nValue;
    BuildFrame('e', &ReportData[0], 2);

//    uint8_t pin = pParam[0];
//    uint8_t status = pParam[1];
//    uint16_t timer = pParam[2]+(pParam[3]<<8);

//    NRF_LOG_RAW_INFO("monet_Ecommand pin(%d) status(0x%x) timer(%d)\r", pin, status, timer);
//    NRF_LOG_FLUSH();

//    monet_conf.gConf[pin].Reload = (timer + TIME_UNIT - 1) / TIME_UNIT;
//    monet_conf.gConf[pin].status = status;
//    monet_gpio.counter[pin] = 0;
//    if(status&GPIO_INTO_MASK) {
//        if(monet_conf.IntPin != GPIO_TO_INDEX(GPIO_NONE)) {
//            monet_conf.gConf[monet_conf.IntPin].status &= (uint8_t)(~GPIO_INTO_MASK);
//        }
//        monet_conf.IntPin = pin;
//        monet_conf.gConf[pin].status |= GPIO_INTO_MASK;
//        monet_gpio.Intstatus= 0;
//    } else {
//        if(monet_conf.IntPin == pin) {
//            monet_conf.IntPin = GPIO_TO_INDEX(GPIO_NONE);
//            monet_gpio.Intstatus= 0;
//        }
//    }
//    if(status&GPIO_WD_MASK) {
//        if(monet_conf.WD.Pin != GPIO_TO_INDEX(GPIO_NONE)) {
//            monet_conf.gConf[monet_conf.WD.Pin].status &= (uint8_t)(~GPIO_WD_MASK);
//        }
//        monet_conf.WD.Pin = pin;
//    } else {
//        if(monet_conf.WD.Pin == pin)
//        {
//            monet_conf.WD.Pin = GPIO_TO_INDEX(GPIO_NONE);
//        }
//    }

//    configGPIO(pin, status);
//    if ((status & GPIO_DIRECTION) == DIRECTION_OUT) {
//        SetGPIOOutput(pin, (bool)((status&GPIO_SET_HIGH) >0));
//    } else {
//        GPIO_SET(pin, pf_gpio_read(pin));
//    }
//    // save_config();
//    pParam[1]= monet_conf.gConf[pin].status;
//    BuildFrame('e', pParam, 4);
}

void pic_setGpioDir(uint8_t ext_gpio, uint8_t dir, uint8_t output)
{
	uint8_t pin = egpio_config[ext_gpio];
	uint8_t pin_status = 0;
	
	if (monet_conf.gConf[pin].status & GPIO_DIRECTION)	// Direction is input
	{
		gpio_deinit(pin);	// Deinit the pin
		if (dir > 0)	// Set to input
		{
			pin_status = monet_conf.gConf[pin].status;
			configGPIO(pin, pin_status);
			GPIO_SET(pin, pf_gpio_read(pin));
		}
		else			// Set to output
		{
			pin_status = (uint8_t)PIN_STATUS(0, 1, 1, (output > 0)? 1: 0);
			configGPIO(pin, pin_status);
			SetGPIOOutput(pin, (output > 0)? 1: 0);
		}
	}
	else	// Direction is output
		pf_gpio_write(pin, output);
}

/* G: External GPIO Configuration */
void monet_Gcommand(uint8_t* pParam, uint8_t Length)
{
	uint8_t gpio;
//    uint8_t pin;
//    uint8_t x, y, z, s;

    gpio = pParam[0];
    monet_data.gpioConf[gpio].direct = pParam[1];
    monet_data.gpioConf[gpio].config = pParam[2];
    if(pParam[1] == 0) {
	    //output
	    monet_data.gpioEv[gpio].report = GPIO_REPORT_NONE;
    }

    pic_setGpioDir(gpio, pParam[1], pParam[2]);
///////////////////////////////////////////////////////////////////////
//    pin = pParam[0];
//    x = (uint8_t)(pParam[1] ? 1 : 0); // 1: Input 0: Output
//    y = (uint8_t)((monet_conf.gConf[pin].status & GPIO_MODE) ? 1 : 0);
//    z = (uint8_t)((monet_conf.gConf[pin].status & GPIO_OUTPUT_HIGH) ? 1 : 0);
//    s = (uint8_t)((pParam[2]) ? 1 : 0);

//    // WARNING: "G" command will set [pin].status bit0-bit3 to 0, so interrupt type will lose efficacy
//    monet_conf.gConf[pin].status = (uint8_t)(PIN_STATUS(x, y, z, s));
//    monet_gpio.counter[pin] = 0;
//    monet_conf.gConf[pin].Reload=0;

//    configGPIO(pin, monet_conf.gConf[pin].status);
//    if ((monet_conf.gConf[pin].status & GPIO_DIRECTION) == DIRECTION_OUT)
//    {
//        SetGPIOOutput(pin, (bool)((uint8_t)pParam[2] > (uint8_t)0));
//    }

//    pParam[2] =  GPIO_GET(pin);

//    BuildFrame('g', pParam, 3);	// There is no reply in Simba code, So this line is commented
}

/* Reset the baseband */
void monet_Hcommand(uint8_t* pParamIn, uint8_t Length)
{
	monet_data.ResetBaseBandDelay = pParamIn[0];
	BuildFrame('h', &pParamIn[0], 1); // NALAMCU-84
	NRF_LOG_RAW_INFO("monet_Hcommand(), reseting Modem\r\n");
}

// Send interrupt setting and ACC state
void monet_Icommand(void)
{
//	BYTE pParam[4];

//    pParam[0] = (uint8_t)(monet_gpio.Intstatus       & 0xFF);
//    pParam[1] = (uint8_t)((monet_gpio.Intstatus>>8)  & 0xFF);
//    pParam[2] = (uint8_t)((monet_gpio.Intstatus>>16) & 0xFF);
//    pParam[3] = (uint8_t)((monet_gpio.Intstatus>>24) & 0xFF);
//    monet_gpio.Intstatus = 0;
//    BuildFrame('i', pParam, 4);
//    eventIReadyFlag = true;		// Keep this code for original code compatibility
	
	uint8_t pParam[4];
	if(monet_data.phoneLive)
        monet_gpio.Intstatus &= monet_data.phoneLive;
	else 
        monet_gpio.Intstatus &= (monet_data.wakeBBMode | INT_MASK_ON);

	pParam[0] = monet_gpio.Intstatus & 0xFF;
	pParam[1] = (monet_gpio.Intstatus>>8)&0xFF;
	pParam[2] = (monet_gpio.Intstatus>>16)&0xFF;
	pParam[3] = (monet_gpio.Intstatus>>24)&0xFF;
	BuildFrame('i', pParam, 4);
    SendMotionAlert();
	monet_gpio.Intstatus = 0;

#ifdef USE_TILT // PUMAMCU-136
    ars_clearTiltState();
#endif
}

uint8_t mux_mcu_delay_cnt = 0;		// Unit in second
uint8_t mux_mcu_delay2_cnt = 0;		// Unit in second
uint8_t mux_mcu_delay3_cnt = 0;		// Unit in TIME_UNIT ms

// MUX MCU update process
// This function is designed to be called in a 1 s period function
void mux_mcu_update_proc(void)
{	
	if (mux_mcu_delay_cnt)
	{
		mux_mcu_delay_cnt--;
		if (mux_mcu_delay_cnt == 0)
		{
			pf_gpio_write(GPIO_ST_MCU_PWR_EN, 1);	// Power on MUX MCU
			mux_mcu_delay2_cnt = 1+1;	// Delay 1 s for MUX. "mux_mcu_delay2_cnt" decreases 1 in current function
		}
	}
	
	if (mux_mcu_delay2_cnt)
	{
		mux_mcu_delay2_cnt--;
		if (mux_mcu_delay2_cnt == 0)
		{
			nrf_gpio_pin_clear(ST_UART1_TO_UART2_EN);	// Set GPIO pin mode 0
			nrf_gpio_pin_clear(ST_UART2_TO_UART3_EN);
			nrf_gpio_pin_clear(ST_UART1_TO_UART3_EN);
			
			gpio_init_pin(GPIO_ST_UART1_TO_UART2_EN);	// Recover 3 GPIO pins to original state (input)
			gpio_init_pin(GPIO_ST_UART2_TO_UART3_EN);
			gpio_init_pin(GPIO_ST_UART1_TO_UART3_EN);
		}
	}
}

// MUX MCU update process2
// This function is designed to be called in a TIME_UNIT ms period function
void mux_mcu_update_proc2(void)
{
	if (mux_mcu_delay3_cnt)
	{
		mux_mcu_delay3_cnt--;
		if (mux_mcu_delay3_cnt == 0)
		{
			uint8_t pParamIn[2] = {'M', 'R'};
			gpio_init_pin(GPIO_ST_UART1_TO_UART2_EN);	// Recover 3 GPIO pins to original state (input)
			gpio_init_pin(GPIO_ST_UART2_TO_UART3_EN);
			gpio_init_pin(GPIO_ST_UART1_TO_UART3_EN);
			mux_update_mode_clear();
			BuildFrame('j', pParamIn, 2);
		}
	}
}

static bool cs_prov_test_mode = false;

// Enter Cargo Sensor PROV-UART test mode
void cs_prov_test_mode_set(void)
{
	cs_prov_test_mode = true;
}

// Leave Cargo Sensor PROV-UART test mode
void cs_prov_test_mode_clear(void)
{
	cs_prov_test_mode = false;
}

// Check if in Cargo Sensor PROV-UART test mode
bool is_in_cs_prov_test_mode(void)
{
	return cs_prov_test_mode;
}

void monet_Jcommand(uint8_t* pParamIn, uint8_t Length)
{
	if (Length == 0)
	{
		// Added for Loader interface
		monet_data.deviceBootEnterDelay = DEVICE_BOOT_ENTER_DELAY_MS;
	}
	else if (Length == 2)
	{
		if (pParamIn[0] == 'M' && pParamIn[1] == 'U')		// Update MUX MCU firmware. See Confluence page for detail, "Mux MCU FW Update and Version Display" (https://montage-systems.atlassian.net/l/c/SVTRyuUk)
		{
			mux_update_mode_set();
			BuildFrame('j', pParamIn, 2);
			pf_gpio_write(GPIO_ST_MCU_PWR_EN, 0);	// Power off MUX MCU
			configGPIO(GPIO_ST_UART1_TO_UART2_EN, PIN_STATUS(0, 1, 1, 0));	// Set the 3 GPIO pins to output and set to mode 4 (Motherboard <-> STM32G070KBT6)
			configGPIO(GPIO_ST_UART2_TO_UART3_EN, PIN_STATUS(0, 1, 1, 0));
			configGPIO(GPIO_ST_UART1_TO_UART3_EN, PIN_STATUS(0, 1, 1, 1));
			nrf_gpio_pin_clear(ST_UART1_TO_UART2_EN);
			nrf_gpio_pin_clear(ST_UART2_TO_UART3_EN);
			nrf_gpio_pin_set(ST_UART1_TO_UART3_EN);
			mux_mcu_delay_cnt = 1;//3;//1;	// Delay mux_mcu_delay_cnt second(s) to power on MUX
		}
		else if (pParamIn[0] == 'M' && pParamIn[1] == 'R')	// Recover MUX MCU state
		{
			pf_gpio_write(GPIO_ST_MCU_PWR_EN, 0);	// Power off MUX MCU
			nrf_delay_ms(10);	// Delay to power off MUX
			configGPIO(GPIO_ST_UART1_TO_UART2_EN, PIN_STATUS(0, 1, 1, 0));	// Set the 3 GPIO pins to output and set to mode 0 (Motherboard <-> External 12 Pin connector)
			configGPIO(GPIO_ST_UART2_TO_UART3_EN, PIN_STATUS(0, 1, 1, 0));
			configGPIO(GPIO_ST_UART1_TO_UART3_EN, PIN_STATUS(0, 1, 1, 0));
			nrf_gpio_pin_clear(ST_UART1_TO_UART2_EN);
			nrf_gpio_pin_clear(ST_UART2_TO_UART3_EN);
			nrf_gpio_pin_clear(ST_UART1_TO_UART3_EN);
			pf_gpio_write(GPIO_ST_MCU_PWR_EN, 1);		// Power on MUX MCU
			mux_mcu_delay3_cnt = 2;		// Unit in TIME_UNIT ms
		}
		else if (pParamIn[0] == 'C' && pParamIn[1] == 'P')	// Enable testing Cargo Sensor through PROV_UART. See Confluence page for detail, "Cargo Sensor Provisioning Support" (https://montage-systems.atlassian.net/l/c/DuBF4G5C)
		{
			mcu_mux_set(MUX_SETTING_2);		// Set MCU MUX to [Internal Cargo Sensor <==> External 12 Pin Connector]
			nrf_gpio_pin_set(CS_3V3_EN);	// Enable internal CS power
			monet_data.V3PowerOn = 1;
			cs_prov_test_mode_set();
			BuildFrame('j', pParamIn, 2);
		}
		else if (pParamIn[0] == 'C' && pParamIn[1] == 'R')	// Recover to default state, App(Modem) <==> PROV-UART
		{
			mcu_mux_set(MUX_SETTING_0);		// Set MCU MUX to default [Motherboard <==> External 12 Pin connector]
			if (!monet_data.phonePowerOn && (monet_gpio.counter[GPIO_VDD_MDM_EN] == 0))		// If Modem is not on, power on it
				monet_gpio.counter[GPIO_VDD_MDM_EN] = (V3_MDM_PW_DELAY_MS + TIME_UNIT - 1) / TIME_UNIT;	//50;	// Value in Simba code here is 50, means 50*100 ms
			nrf_gpio_pin_clear(CS_3V3_EN);	// Disable internal CS power
			monet_data.V3PowerOn = 0;
			cs_prov_test_mode_clear();
			BuildFrame('j', pParamIn, 2);
		}
	}
}

// Send baseband watchdog and reset it to default value
void monet_Kcommand(uint8_t* pParamIn, uint8_t Length)
{
    uint8_t pParam[3];

//    pParam[0] = (uint8_t) (monet_gpio.WDtimer         & 0x000000FF);
//    pParam[1] = (uint8_t)((uint32_t)(monet_gpio.WDtimer & 0x0000FF00)>>8);
//    pParam[2] = (uint8_t)((uint32_t)(monet_gpio.WDtimer & 0x00FF0000)>>16);
//    monet_gpio.WDtimer = (uint32_t)(((uint32_t)pParamIn[0]) + ((uint32_t)pParamIn[1]<<8) + ((uint32_t)pParamIn[2]<<16));
//    BuildFrame('k', pParam, 3);
//    monet_gpio.WDflag  = 0;

    pParam[0] = (uint8_t) (monet_gpio.WDtimer         & 0x000000FF);
    pParam[1] = (uint8_t)((uint32_t)(monet_gpio.WDtimer & 0x0000FF00)>>8);
    monet_gpio.WDtimer = gBWD;
    BuildFrame('k', pParam, 2);
    monet_gpio.WDflag = 0;
}

// LED control
void monet_Lcommand(BYTE* pParam, BYTE Length)
{
	// NALAMCU-79. After Modmem sent 'P','9' command to enter Battery Save Mode,
	// there is risk that Modem would send 'L' command to turn on LEDs. 
	// This would make LED on in Battery save mode.
	if (monet_data.bBattMarkedCritical == true)
		goto REPLY_WITHOUT_OPER;
	
	if (leds_ctrlled_by_mcu() == true)
		goto REPLY_WITHOUT_OPER;
	
    gFlashLED = 0;
    switch (pParam[0] & 0x7F) {
    case 0:
    case 1:
    /*case 2:*/
	    monet_startLED(pParam, 0);
	    break;
	case 2:
	    orange_led_ctrl(pParam, Length);
	    break;
    case 3:
	    // Firefighter mode
	    pParam[0] = 0x80;  // Do LED 0
	    monet_startLED(pParam, LED_ON);
	    pParam[0] = 0x81;  // Do LED 1
	    monet_startLED(pParam, 0);
	    break;
    case 4:
	    // Synchronous mode
	    pParam[0] = 0x80;  // Do LED 0
	    monet_startLED(pParam, LED_ON);
	    pParam[0] = 0x81;  // Do LED 1
	    monet_startLED(pParam, LED_ON);
	    break;
    default:
	    break;
    }
	
REPLY_WITHOUT_OPER:
    BuildFrame('l', pParam, Length);
}

// ACC configure
void monet_Mcommand(uint8_t* pParam, uint8_t Length)
{
    // Old Format
    if (Length <= 2) {
        // Update the threshold
//        monet_data.AccData.threshold = LIS_2GTHRESHOLD_REG_TO_MM_BY_SEC2(pParam[0]);
//        if (monet_data.AccChipID) pf_imu_sensitivity_set(monet_data.AccData.threshold);
		config_data.at[1] = pParam[0];
        // Update the duration
        if (Length == 2) {
            // When duration is changed durationTime should be changed
            monet_data.AccData.duration = pParam[1];
//            monet_data.AccData.durationTime = monet_data.AccData.duration;
			config_data.adur[1] = 0;
        }
        BuildFrame('m', pParam, Length);
        return;
    }
}

// ACC configure
void monet_mCommand(uint8_t* pParam, uint8_t Length)
{
    uint8_t nIndex;
    uint8_t mode;
//    uint32_t threshold = 0;
	int retries = 0;
//    uint8_t success = monet_data.AccChipID;

    // New Format
    nIndex = 0;
    while (nIndex < Length) {
        switch (pParam[nIndex]) {
        case 'D':
            // When duration is changed durationTime should be changed
            monet_data.AccData.duration = ((uint32_t)pParam[nIndex + 1] + ((uint32_t)pParam[nIndex + 2] << 8));
            monet_data.AccData.durationTime = monet_data.AccData.duration;
			config_data.adur[1] =  0; // Application will debounce; not accelerometer
            nIndex += 3;
            break;
        case 'T':
            monet_data.AccData.threshold = pParam[nIndex + 1];
			config_data.at[1] = monet_data.AccData.threshold;
			nIndex += 2;
            break;
        case 'M':
            mode = pParam[nIndex + 1];
            if (IS_MOTION_STATE_VALID(mode)) {
                monet_data.AccData.motionMode  = (ACC_MOTION_STATES_t)mode;
                monet_data.AccData.motionState = (mode == MOTION_STATE_START) ? MOTION_STATE_STOP : 
                                                 (mode == MOTION_STATE_STOP) ? MOTION_STATE_START : 
                                                 MOTION_STATE_NONE;
            } else {
                monet_data.AccData.motionMode  = MOTION_STATE_NONE;
                monet_data.AccData.motionState = MOTION_STATE_NONE;
            }
			nIndex += 2;
            break;
        default:
            return;
        }
    }

//    // TODO: Program the MEMS with default value.
//    if (success) {
//        pf_imu_workmode_set((pf_AccWorkMode_t)ars_accdata.AccWorkingMode);
//    }
	// Program the MEMS with default value. The loop exit if the
    // configuration is successful or too many attempts
    while ((ion_accRegInit2(config_data.ar[1], config_data.at[1], config_data.adur[1], 0, 0) == MEMS_ERROR) &&
        (retries < 10)) {
        retries++;
    }
    BuildFrame('M', pParam, Length);
//    monet_data.waitingForMT = 0;
}

// Set WBtimerDefault
void monet_Ncommand(uint8_t* pParam, uint8_t Length)
{
    monet_data.SleepAlarm  = ((uint32_t)pParam[2] << 16);
    monet_data.SleepAlarm += ((uint32_t)pParam[1] << 8);
    monet_data.SleepAlarm += ((uint32_t)pParam[0] << 0);
    monet_gpio.WDtimer = monet_data.SleepAlarm + 90;	// Keep Modem watchdog timer larger than Modem wakeup alarm
//    monet_data.bbofftime = count1sec; // LM-23 take a snapshot of the runtime count for sleep
    BuildFrame('n', pParam, Length);
}

/*
 * O is a test command
 *
 * First Parameter is the test
 * Additional parameters specify values for the test
 *
 * Test 01: Turn the orange LED into an accelerometer test (0 disable, 1 enable)
 */
// "The 'O' (oh) command was being used as a test command 
//  to allow a visual indication of the Accelerometer activy which 
//  is no longer being used and can be ignored", Michael Silveus, 2020/9/9 3:01
void monet_Ocommand(BYTE* pParam)
{
	BYTE nSize = 1;

//	nSize = 0;
//	switch (pParam[0]) {
//	case 0x01:
//		monet_data.AccTestMode = pParam[1];
//		nSize = 2;
//	default:
//		break;
//	}

	if (nSize) {
		BuildFrame('o', pParam, 1);
	}
}

// Power control of Baseband, V3(Cargo Sensor) and ...
void monet_Pcommand(uint8_t* pParam, uint8_t Length)
{
    switch(pParam[0])
	{
		case MONET_BB_OFF:
			pic_turnOffBaseband();
			break;
		case MONET_BB_ON:
			pic_turnOnBaseband();	//test command
			break;
 		case MONET_V3_ON:
			pic_turnOnV3();
//			monet_data.SensorDelayedTest = (V3_TURN_ON_DELAY_MS + TIME_UNIT - 1) / TIME_UNIT; //5; // 50 ms
			monet_data.SensorDelayedTest = 5; // 50 ms
			break;
 		case MONET_V3_OFF:
			pic_turnOffV3();
			break;
 		case MONET_BUBX_ON:			// To be implemented...
			// Indicate shipping mode
//			SetConfiguration(PARAM_SHIPPING_MODE, SHIPPING_MODE_ON);	// Flash operation is to be implemented.
            config_data.bShippingMode = 1;
            monet_data.BUBX = 1;
            // PUMAMCU-78 Ensure allow power is set on whenever Shipping mode is set
//            SetConfiguration(PARAM_ALLOW_POWERKEY, ALLOW_POWER_KEY);
            config_data.bAllowPowerKey = 1;
            monet_data.isStarted       = 0;    // PUMAMCU-152. Not used in Simba code
            monet_data.sleepmode       = 0;    // SIMBAMCU-29
			break;
 		case MONET_BUBX_OFF:
			monet_data.BUBX = 0;
			break;
 		case MONET_WPIN_ON:			// No function implemented in Simba. Ignore it
			break;
 		case MONET_WPIN_OFF:		// No function implemented in Simba. Ignore it
			break;
 		case MONET_BLATCH_ON:		// It's empty here also in Simba code
			break;
 		case MONET_BLATCH_OFF:		// It's empty here also in Simba code
			if(!monet_data.bEnterEM3)
			{ // SIMBAMCU-61 Battery Critical state
	                NRF_LOG_RAW_INFO("Enter battery power save mode.\r\n");
	                NRF_LOG_FLUSH();
			
	                monet_data.bBattMarkedCritical = true;
	                monet_data.bEnterEM3 = true;
	                monet_data.sleepmode = 0; // SIMBAMCU-29 Clear sleep indicator
	                if (monet_data.phonePowerOn) {
	                    pic_turnOffBaseband();
	                }
	                gAdcBatCounter = BUB_CRITICAL_DEBOUNCE + 1; // Indicate the batt critical debounce expired
	                setChargerOn(); // SIMBAMCU-7 make sure the charger is enabled any time the battery is low

	                monet_data.ledConf[0].status = 0;
	                monet_data.ledConf[1].status = 0;					
	                pf_gpio_write(GPIO_LED_GREEN, 1);
	                pf_gpio_write(GPIO_LED_RED, 1);
			}
			break;
 		case MONET_BUB_CRITICAL:	// To be implemented...
			monet_data.BubCriticalThreshold = (pParam[2] << 8) + pParam[1];
            if (Length > 3) {
                monet_data.BubCriticalTime = (pParam[4] << 8) + pParam[3]; // SIMBAMCU-29
            }
			break;
 		case MONET_EXT_CS_OFF:
			pf_gpio_write(GPIO_CS_12V_EN, 0);
			BuildFrame('p', pParam, Length);
			break;
 		case MONET_EXT_CS_ON:
			pf_gpio_write(GPIO_CS_12V_EN, 1);
			BuildFrame('p', pParam, Length);
			break;
        default:
			// Extract the power state config data
			ConfigPowerMasks(pParam, Length);
			break;
    }
//    BuildFrame('p', pParam, Length);	// There is no reply in Simba code, so this line is commented.
}

/*
 * First Parameter Command
 * 0x01: Read GPIO all
 * 0x02: Read GPIO x
 * 0x03: Read count of 1-wire sensors, disable/enable 1-wire
 */
void monet_Qcommand(uint8_t* pParam, uint8_t Length)
{
	uint8_t command, value;
	uint8_t gpio;
    uint8_t ReportData[4] = { 0 };
	uint8_t size;
    uint8_t sensor = 0;
    uint8_t sensorcnt = 0;
    uint8_t current1wirestate = monet_data.OneWireDisable;

	command = pParam[0];
	size = 0;
	
	ReportData[0] = command;
	switch (command) {
	case 0x01:
		value = 0;
		for (gpio = 0; gpio < 8; gpio++) {
			value = value | (pf_gpio_read(egpio_config[gpio]) << gpio);	
		}
		ReportData[1] = value;
		size = 2;
		break;
	case 0x02:
		gpio = pParam[1];
		ReportData[1] = gpio;
		ReportData[2] = pf_gpio_read(egpio_config[gpio]);
		size = 3;
		break;
	case 0x03:
		ReportData[1] = pParam[1];
		size = 1;
        if (pParam[1] == 2) { // NALAMCU-29 1-wire discover now
            monet_data.OneWireDisable = 0;
            OneBusInit();
            OWDiscovery();
            for (sensor = 0; sensor < MAX_TEMP_SENSORS; sensor++) {
                if (get1WSensorRegId(sensor) != 0) {
                    sensorcnt++;
                }
            }
            ReportData[2] = sensorcnt;
            size = 3;
			monet_data.OneWireDisable = current1wirestate;
        }
        else {
            monet_data.OneWireDisable = pParam[1]; // 1-wire disable (0=not disabled, 1=disabled)
		    size = 2;
        }
        if (monet_data.OneWireDisable != 0) {
//			GPIO_PinModeSet(egpio_config[ONEWIRE_INDEX].port, egpio_config[ONEWIRE_INDEX].pin, egpio_config[ONEWIRE_INDEX].mode, egpio_config[ONEWIRE_INDEX].out);
//			drive_OW_low();
			// Disable One-Wire
			configGPIO(GPIO_ONE_BUS_SLPZ, PIN_STATUS(0, 1, 1, 0));
			pf_gpio_write(GPIO_ONE_BUS_SLPZ, 0);
			gpio_deinit(GPIO_ONE_BUS_SLPZ);
        }
        break;
	}
	if (size) {
		BuildFrame('q', &ReportData[0], size);
	}

}

// Send (RTC time - bbofftime)
void monet_Rcommand(uint8_t* pParam)
{
    uint32_t t;
    uint8_t pParamO[4];

    if (monet_data.bbofftime == 0) { // Handle the initial power up case
        t = 0;
    } else {
        t = count1sec - monet_data.bbofftime;
    }

    pParamO[0] = (uint8_t)((t>>0)  & 0xFF);
    pParamO[1] = (uint8_t)((t>>8)  & 0xFF);
    pParamO[2] = (uint8_t)((t>>16) & 0xFF);
    pParamO[3] = (uint8_t)((t>>24) & 0xFF);
    BuildFrame('r', pParamO, 4);
}

// Set rttime, basebandWakeDelay and turn off Baseband
void monet_Scommand(BYTE* pParam)
{
	monet_data.rctime = pParam[0] + (pParam[1]<<8) + (pParam[2]<<16) + (pParam[3]<<24);
//	count1sec = pParam[0] + (pParam[1]<<8) + (pParam[2]<<16) + (pParam[3]<<24);
    monet_data.basebandWakeDelay = 5;	// To be implemented...
    pic_turnOffBaseband();
	NRF_LOG_RAW_INFO("S cmd. rctime %u. Turning bb off\r\n", monet_data.rctime);
}

// Read and send MODEL
void monet_Tcommand(void)
{
    uint8_t pParm[4];
    uint32_t *pData = (uint32_t*)(pParm);
    *pData = model;
    BuildFrame('t', pParm, 4);
}

// Send reset cause
void monet_Ucommand(void)
{
	BYTE pParm[4];
	pParm[0] = ((gresetCause & 0xFF000000) >> 24) & 0xFF;
	pParm[1] = ((gresetCause & 0x00FF0000) >> 16) & 0xFF;
	pParm[2] = ((gresetCause & 0x0000FF00) >> 8) & 0xFF;
	pParm[3] = ((gresetCause & 0x000000FF) >> 0) & 0xFF;

	BuildFrame('u', pParm, 4);
}

// Send MCU software version
void monet_Vcommand(void)
{
    uint8_t pParm[4];

    pParm[0] = MNT_MAJOR;
    pParm[1] = MNT_MINOR;
    pParm[2] = MNT_REVISION;
    pParm[3] = MNT_BUILD;

    BuildFrame('v', pParm, 4);
}

// Set PARAM_MODEL in EEPROM
void monet_Wcommand(uint8_t* pParam)
{
	uint32_t *pData = (uint32_t*)pParam;

	model = *pData;
    monet_Tcommand();
}

// ACC control
void monet_Xcommand(uint8_t *pParam, uint8_t Length)
{
    switch (pParam[0])
    {
//        case 'C':
//        {
//            ars_AccWorkMode_t workmode = (ars_AccWorkMode_t)pParam[1];
//            if (IS_VALID_ACC_WORKMODE(workmode))
//            {
//                BuildFrame('x', pParam, Length);
//                ars_accdata.AccWorkingMode = workmode;
//                pf_imu_workmode_set((pf_AccWorkMode_t)ars_accdata.AccWorkingMode);
//            }
//        }
//        break;
        case 'R':
		{
			pParam[3] = md_LisReadReg(pParam[1], &(pParam[2]), 1);
			BuildFrame('x', pParam, 4);
		}
			break;
        case 'W':
        {
			pParam[3] = md_LisWriteReg(pParam[1], pParam[2]);
			BuildFrame('x', pParam, 4);
        }
			break;
		case 'B':
			monet_data.AccMode = 0;				// AccMode and AccDataAvailable are not used in Simba. Ignore them here.
			monet_data.AccDataAvailable = 0;
			ion_accStart(0, pParam[1]);
			BuildFrame('x', pParam, 2);
			break;
        case 'M':
			SendMotionAlert();
			break;
		case 'S':
			ion_accStart(0, 0);
			BuildFrame('x', pParam, 2);
			break;
		case 'U':
			monet_data.AccMode= 1;
			monet_data.AccDataAvailable = 0;
			ion_accRegInit2(pParam[1], pParam[2], 0, 0, 0);
			BuildFrame('x', pParam, 4);
			break;
        default:
//		{
//			SendMotionAlert();
//		}
			break;
    }
}

// Send data of parameters PARAM_PU and PARAM_WD in EEPROM
void monet_Ycommand()
{
    uint8_t pParm[4] = {0};

    pParm[0] = (uint8_t)((pu & 0xff00) >> 8);
    pParm[1] = (uint8_t)(pu & 0xff);
    pParm[2] = (uint8_t)((rst & 0xff00) >> 8);
    pParm[3] = (uint8_t)(rst & 0xff);
    BuildFrame('y', pParm, 4);
}

// ('C')Charger control
// ('D')last frame count read/write
// ('G')1Wsensor temp and family read
// ('P')PeriUart control
// ('S')1WSensor temp threshold set
// ('T')Tilt cosine and debounce
// ('X')get clock rates
// ('Z')Sleep mode, WBtimerDefault set
void monet_Zcommand(uint8_t* pParam, uint8_t Length)
{
	uint8_t nSize = 0;
//    uint8_t mode;
//    uint8_t z_buf[16] = {0};
//    uint8_t len = Length;
	uint8_t Response[10] = {0};
	uint8_t clocks[(4 * sizeof(uint32_t)) + 1] = {0};
    uint8_t index;
	uint32_t value;
	uint32_t solar_check_period = 0;
	
//    memcpy(z_buf, pParam, len);
	Response[0] = pParam[0];
	
    switch(pParam[0])
	{
		case 'B':
		{
			Response[1] = pParam[1];
			if (pParam[1] == 'W')
			{
				nSize = 2;
				solar_check_period = pParam[2] + (pParam[3]<<8) + (pParam[4]<<16) + (pParam[5]<<24);
				timer_solar_chg_mode_intvl_set(solar_check_period);
				timer_solar_chg_mode_restart();
			}
			else if (pParam[1] == 'R')
			{
				nSize = 2+4;
				solar_check_period = timer_solar_chg_mode_intvl_get();
				Response[2] = solar_check_period & 0xff;
				Response[3] = (solar_check_period >> 8) & 0xff;
				Response[4] = (solar_check_period >> 16) & 0xff;
				Response[5] = (solar_check_period >> 24) & 0xff;
			}
		}
			break;
		case 'C':
		{
			uint16_t oldChargeValue = monet_data.ChargerRestartValue;
			// Get the new charger threshold
			value = pParam[1] + (pParam[2] << 8);
			monet_data.ChargerRestartValue = (UINT16)(value);
			//Restart the charger
			if (oldChargeValue != monet_data.ChargerRestartValue) {
				if (monet_data.V3PowerOn == 0) {
					setChargerOn();
				}
			}
			// Prepare the response
			Response[1] = pParam[1];
			Response[2] = pParam[2];
			nSize = 3;
		}
			break;
		case 'D':
		{
			switch (pParam[1]) {
			case 0: // Query last data frame
			{
				Response[1] = pParam[1];
				Response[2] = monet_data.lastdataframecounter;
			}
			break;
			case 1: // Set last frame count
			{
				monet_data.lastdataframecounter = pParam[2];
				Response[1] = pParam[1];
				Response[2] = monet_data.lastdataframecounter;
			}
			break;
			}
			// Prepare the response
			nSize = 3;
		}
			break;
		case 'G':
	#ifdef SUPPORTS_CHAIN_PROTOCOL // NALAMCU-29
		{
			INT16 temp10 = 0;
			uint8_t index = 0;

			if (get1WSensorFamily(0) == CHAIN_FAMILY) {
				Response[1] = 0;
				for (index = 0; index < MAX_TEMP_SENSORS; index++) {
					temp10 = get1WSensorTemp(index); // Retrieve the first sensor location
					if (get1WSensorFamily(index) == CHAIN_FAMILY) {
						Response[1] = index + 1;
						Response[2] = get1WSensorFamily(0);
						Response[(index * 2) + 3] = (temp10 & 0x00FF) >> 0;
						Response[(index * 2) + 4] = (temp10 & 0xFF00) >> 8;
						// Prepare the response
						nSize = (index * 2) + 5;
					}
				}
			}
			else {
				temp10 = get1WSensorTemp(0); // Retrieve the first sensor location if any
				// Get the current temperature
				Response[1] = (get1WSensorFamily(0) != 0) ? 1 : 0;
				Response[2] = get1WSensorFamily(0);
				Response[3] = (temp10 & 0x00FF) >> 0;
				Response[4] = (temp10 & 0xFF00) >> 8;
				// Prepare the response
				nSize = 5;
			}
		}
	#else
			// Get the current temperature
			pParam[1] = monet_data.Temp10 & 0xFF;
			pParam[2] = (monet_data.Temp10 & 0xFF00) >> 8;
			// Prepare the response
			Response[1] = pParam[1];
			Response[2] = pParam[2];
			nSize = 3;
	#endif
			break;
		case 'P':  // PUMAMCU-137 Control TX line for external sensors
		{
			// Ignore 'P' command
			
//			if (pParam[1] == 1) {
//				pic_setPeriUartTxHigh();
//			} else {
//				pic_setPeriUartTxLow();
//			}
		}
			break;
		case 'S':
			// Set the threshold
			if (Length == 5) { // If no index byte set the first or only sensor temp
				// get the Low Temp threshold
				value = (pParam[2] << 8) + pParam[1];
				// Store the Low Temp threshold
				store1WSensorTempLow(0, value);
				// get the High Temp threshold
				value = (pParam[4] << 8) + pParam[3];
				// Store the High Temp threshold
				store1WSensorTempHigh(0, value);
				Response[1] = pParam[1];
				Response[2] = pParam[2];
				Response[3] = pParam[3];
				Response[4] = pParam[4];
			}
			else {
				uint8_t sensor = pParam[1];
				// get the Low Temp threshold
				value = (pParam[3] << 8) + pParam[2];
				// Store the Low Temp threshold
				store1WSensorTempLow(sensor, value);
				// get the High Temp threshold
				value = (pParam[5] << 8) + pParam[4];
				// Store the High Temp threshold
				store1WSensorTempHigh(sensor, value);
				Response[1] = pParam[1];
				Response[2] = pParam[2];
				Response[3] = pParam[3];
				Response[4] = pParam[4];
				Response[5] = pParam[5];
			}
			// Prepare the response
			nSize = Length;
			break;
#ifdef USE_TILT // PUMAMCU-136
		case 'T':  // Tilt cosine and debounce
		{
			int32_t     tilt;
			tilt = pParam[1] + (pParam[2] << 8) + (pParam[3] << 16) + (pParam[4] << 24);
			monet_data.TiltThreshold = ars_shiftCosValue(((double)tilt / 1000000)); // MNT-1494 Improvement in detection
			monet_data.TiltEventTimer = pParam[5];
			Response[0] = pParam[0]; // SIMBAMCU-40
			Response[1] = pParam[1]; // SIMBAMCU-40
			Response[2] = pParam[2]; // SIMBAMCU-40
			Response[3] = pParam[3]; // SIMBAMCU-40
			Response[4] = pParam[4]; // SIMBAMCU-40
			Response[5] = pParam[5]; // SIMBAMCU-40
			nSize = Length;          // SIMBAMCU-40
		}
			break;
#endif
		case 'X':
			// get clock rates
			index = 0;
			clocks[index++] = 'X';
//			switch (pParam[1]) {
//			case 0:
//				value = CMU_ClockFreqGet(cmuClock_CORE);
//				clocks[index++] = value & 0xff;
//				clocks[index++] = (value & 0xff00) >> 8;
//				clocks[index++] = (value & 0xff0000) >> 16;
//				clocks[index++] = (value & 0xff000000) >> 24;
//				break;
//			case 1:
//				value = CMU_ClockFreqGet(cmuClock_HFPER);
//				clocks[index++] = value & 0xff;
//				clocks[index++] = (value & 0xff00) >> 8;
//				clocks[index++] = (value & 0xff0000) >> 16;
//				clocks[index++] = (value & 0xff000000) >> 24;
//				break;
//			case 2:
//				value = CMU_ClockFreqGet(cmuClock_USART1);
//				clocks[index++] = value & 0xff;
//				clocks[index++] = (value & 0xff00) >> 8;
//				clocks[index++] = (value & 0xff0000) >> 16;
//				clocks[index++] = (value & 0xff000000) >> 24;
//				break;
//			case 3:
//				value = CMU_ClockFreqGet(cmuClock_LEUART0);
//				clocks[index++] = value & 0xff;
//				clocks[index++] = (value & 0xff00) >> 8;
//				clocks[index++] = (value & 0xff0000) >> 16;
//				clocks[index++] = (value & 0xff000000) >> 24;
//				break;
//			}
			index += 4;	///////////////////	For debuging
			BuildFrame('z', clocks, index);
			// Prepare the response
			nSize = 0;
			break;
        case 'Z':
// The sleepmode/SleepState is commented here. Change of monet_data.SleepState in SLP01 
// would influence systick frequency. See timer_systick_handler() for reference.
// And monet_data.SleepState would finally be set to SLEEP_HIBERNATE in MCU_TurnOff_MDM().
//			monet_data.sleepmode  = pParam[1]; // SIMBAMCU-29
//			monet_data.SleepState = (SleepState_e)pParam[1];
			monet_data.SleepAlarm = ((UINT32)pParam[2] << 0);
			monet_data.SleepAlarm += ((UINT32)pParam[3] << 8);
			monet_data.SleepAlarm += ((UINT32)pParam[4] << 16);
			monet_gpio.WDtimer = monet_data.SleepAlarm + 90;	// Keep Modem watchdog timer larger than Modem wakeup alarm
			Response[0] = pParam[0];
			Response[1] = pParam[1];
			Response[2] = pParam[2];
			Response[3] = pParam[3];
			Response[4] = pParam[4];
			Response[5] = pParam[5];
			nSize = Length;
			NRF_LOG_RAW_INFO("ZZ cmd, alarm %u, sleepmode %u, WD %u\r\n", monet_data.SleepAlarm, monet_data.SleepState, monet_gpio.WDtimer);
			break;
//		{
//            mode = z_buf[1];
//            len -= 2;
//            if (len-- > 0) monet_data.SleepAlarm  = ((uint32_t)z_buf[2] << 0);
//            if (len-- > 0) monet_data.SleepAlarm += ((uint32_t)z_buf[3] << 8);
//            if (len-- > 0) monet_data.SleepAlarm += ((uint32_t)z_buf[4] << 16);
//            monet_gpio.WDtimer = monet_data.SleepAlarm + 90;
//            NRF_LOG_RAW_INFO("monet_Zcommand mode: %d SleepAlarm: %d WDtimer: %d\r", mode, monet_data.SleepAlarm, monet_gpio.WDtimer);
//            NRF_LOG_FLUSH();
//            switch(mode) {
//                case 0: {
//                    monet_data.SleepState = SLEEP_OFF;
//                    monet_data.SleepStateChange = 1;
//                    monet_data.SleepAlarm = 0;
//                    monet_gpio.WDtimer = monet_conf.WD.Reload;
//                }
//                break;
//                case 1: {
//                    monet_data.uartToBeDeinit = 1;
//                    break;
//                }
//                case 2: {
//                    pic_turnOffBaseband();
//                    break;
//                }
//                case 3:
//                    // Could help to calculate bootup time for softresets
//                    monet_data.bbofftime = count1sec;
//                    break;
//                default:{
//                    // MCU_Wakeup_MDM();
//                    // MCU_Wakeup_APP();
//                }
//			}
//			// LM-37
//			z_buf[2] = (uint8_t)((monet_gpio.WDtimer & 0x000000FF)>>0);
//			z_buf[3] = (uint8_t)((monet_gpio.WDtimer & 0x0000FF00)>>8);
//			z_buf[4] = (uint8_t)((monet_gpio.WDtimer & 0x00FF0000)>>16);
//			z_buf[5] = 0;
//			BuildFrame('z', z_buf, Length);
//        }
//            break;
    }
	
	if (nSize) {
		BuildFrame('z', Response, nSize);
	}
}

void monet_AAcommand_zazu(uint8_t* p_data, uint8_t len)
{
	uint8_t param[32] = {0};
	uint8_t comp = 0;	// Component
	uint8_t ch = 0;	// Channel (instance ID)
	
	if ((len != 3) && (len != 6))
	{
		NRF_LOG_RAW_INFO("monet_AAcommand_zazu() err: len incorrect %u\r\n", len);
		NRF_LOG_FLUSH();
		return;
	}
	comp = p_data[0];
	ch = p_data[1];
	if (comp == COMP_ZAZU_BLE || comp == COMP_ZAZU_ESP32)
	{
		param[0] = 0xAA;
		memcpy(&param[1], p_data, len);
		monet_bleScommand(param, len+1, SENSOR_MASK_BPOS_CMR, ch);
		return;
	}
	else
	{
		NRF_LOG_RAW_INFO("monet_AAcommand_zazu() err: unknown comp %u\r\n", comp);
		NRF_LOG_FLUSH();
		return;
	}
}

#define CHG_DATA_BYTE_CNT	10

// Report Charger chip info.
// Data format is defined on Confluence page "Extracting information from the components" (https://montage-systems.atlassian.net/l/c/m31PLxv9)
static void charger_chip_info_report(const uint8_t* pParam, uint8_t Length)
{
	uint8_t Response[32] = {0};
	uint8_t Len = 0;
	
	uint32_t input_vol = 0;
	uint32_t input_vol_limit = 0;
	uint32_t bat_vol = 0.0;
	double input_cur = 0.0;
	uint32_t input_cur_10 = 0.0;
	double bat_chg_cur = 0.0;
	uint32_t bat_chg_cur_10 = 0.0;
	
	Response[0] = pParam[0];
	Response[1] = pParam[1];
	Response[2] = 0x03;		// Bit 3 is for Charger Chip Data
	Response[3] = CHG_DATA_BYTE_CNT;
	Len += 4;
	
	if (is_charger_power_on() == true)
	{
		input_vol = mp2762a_input_vol_get();
		input_vol_limit = (uint32_t)mp2762a_input_vol_limit_get();
		bat_vol = (uint32_t)mp2762a_bat_vol_get();
		input_cur = mp2762a_input_cur_get();
		bat_chg_cur = mp2762a_bat_chg_cur_get();
	}
	
	Response[4] = input_vol & 0xff;
	Response[5] = (input_vol >> 8) & 0xff;
	Response[6] = input_vol_limit & 0xff;
	Response[7] = (input_vol_limit >> 8) & 0xff;
	Response[8] = bat_vol & 0xff;
	Response[9] = (bat_vol >> 8) & 0xff;
	input_cur_10 = (uint32_t)(input_cur * 10);
	Response[10] = input_cur_10 & 0xff;
	Response[11] = (input_cur_10 >> 8) & 0xff;
	bat_chg_cur_10 = (uint32_t)(bat_chg_cur * 10);
	Response[12] = bat_chg_cur_10 & 0xff;
	Response[13] = (bat_chg_cur_10 >> 8) & 0xff;
	Len += CHG_DATA_BYTE_CNT;
	
	BuildFrame(0xAA, Response, Len);
}


// Check component version
/*
 * Component List
 * 0 Reserved
 * 1 Simba MCU
 * 2 Simba BLE MCU
 * 3 SLP-01 MCU
 * 4 Nala MCU
 * 5 Nala MUX MCU
 * 6 Camera (AC61/Zazu) Nordic MCU (multi-instance)
 * 7 Camera (AC61/Zazu) ESP32 MCU (multi-instance)
 * 
 * Information Type List
 * Mask  Bit  Information Type
 * 0x01  0    Version
 * 0x02  1    MAC Address
 * 0x04  2    Hardware ID and Revision
 * 
 * For detail description, please refer to the below link.
 * Extracting information from the components
 * https://montage-systems.atlassian.net/wiki/spaces/DD/pages/1302593559/Extracting+information+from+the+components
 */
void monet_AAcommand(uint8_t* pParam, uint8_t Length)
{
	uint8_t Response[32] = {0};
	uint8_t Len = 0;
	uint32_t info_type = 0; 

//	if (Length == 3)	// Newly added feature
	if (pParam[0] == COMP_ZAZU_BLE || pParam[0] == COMP_ZAZU_ESP32)	// For Zazu BLE or ESP32
	{
		monet_AAcommand_zazu(pParam, Length);
		return;
	}
	
//	if (Length != 2)
//	{
//		NRF_LOG_RAW_INFO("monet_AAcommand() err: command unknown\r\n");
//		NRF_LOG_FLUSH();
//		return;
//	}
	Response[0] = pParam[0];
	Response[1] = pParam[1];
	if (pParam[0] == COMP_NALA_MCU)	// Component number for Nala MCU
	{
		Len += 2;
		if (pParam[1] & 0x80)	// Info type is 4 byte version
		{
			Response[2] = pParam[2];
			Response[3] = pParam[3];
			Response[4] = pParam[4];
			Len += 3;
			info_type = pParam[1] + (pParam[2] << 8) + (pParam[3] << 8) + (pParam[4] << 8);
		}
		else
			info_type = pParam[1];
		
		if (info_type & MASK_FOR_BIT(INFO_TYPE_BPOS_VER))		// Information Type Version
		{
			Response[Len+0] = INFO_TYPE_BPOS_VER;
			Response[Len+1] = 0x04;
			Response[Len+2] = MNT_MAJOR;
			Response[Len+3] = MNT_MINOR;
			Response[Len+4] = MNT_REVISION;
			Response[Len+5] = MNT_BUILD;
			Len += 6;
		}
		if (info_type & MASK_FOR_BIT(INFO_TYPE_BPOS_MAC))	// Information Type MAC Address
		{
			ble_gap_addr_t ble_id_addr = {0};
			
			sd_ble_gap_addr_get(&ble_id_addr);
			Response[Len+0] = INFO_TYPE_BPOS_MAC;
			Response[Len+1] = 0x06;
			Response[Len+2] = ble_id_addr.addr[5];
			Response[Len+3] = ble_id_addr.addr[4];
			Response[Len+4] = ble_id_addr.addr[3];
			Response[Len+5] = ble_id_addr.addr[2];
			Response[Len+6] = ble_id_addr.addr[1];
			Response[Len+7] = ble_id_addr.addr[0];
			Len += 8;
		}
		if (info_type & MASK_FOR_BIT(INFO_TYPE_BPOS_PW_INFO))	// Power Information
		{
			uint16_t vol_ext = 0;
			uint16_t vol_main = adc_to_vol_conv(monet_data.AdcMain, VOL_MAIN_FACTOR);
			uint16_t vol_aux = adc_to_vol_conv(monet_data.AdcAux, VOL_AUX_FACTOR);
			uint16_t vol_bat = adc_to_vol_conv(monet_data.AdcBackup, VOL_BAT_FACTOR);
			uint16_t vol_solar = adc_to_vol_conv(monet_data.AdcSolar, VOL_SOLAR_FACTOR);
			
			vol_ext = MAX(vol_main, vol_aux);
			Response[Len+0] = INFO_TYPE_BPOS_PW_INFO;
			Response[Len+1] = 9;
			Response[Len+2] = PW_SOURCE_EXT;
			Response[Len+3] = vol_ext & 0xff;
			Response[Len+4] = (vol_ext >> 8) & 0xff;
			Response[Len+5] = PW_SOURCE_BAT;
			Response[Len+6] = vol_bat & 0xff;
			Response[Len+7] = (vol_bat >> 8) & 0xff;
			Response[Len+8] = PW_SOURCE_SOLAR;
			Response[Len+9] = vol_solar & 0xff;
			Response[Len+10] = (vol_solar >> 8) & 0xff;
			Len += 11;
		}
		
		BuildFrame(0xAA, Response, Len);
	}
	else if (pParam[0] == COMP_NALA_MUX)	// Component number for Nala MUX MCU
	{
		if ((pParam[1] & 0x01) ==  1)		// Information Type 0x01, Version
		{
			mux_version_read(pParam[1]);	// Response would be sent after calling this function
		}
	}
	else if (pParam[0] == COMP_NALA_CHARGER_CHIP)	// Component number for Nala Charger chip
	{
		if ((pParam[1] & 0x08) != 0)		// Information Type 0x08, Charger Chip Data
		{
			charger_chip_info_report(pParam, Length);
		}
	}

}

void ble_pair_test(uint8_t para)
{
	if (para == 0)
	{
		NRF_LOG_RAW_INFO("peer count %u\r", pm_peer_count());
	}
	else if (para == 1)
	{
		pm_peer_id_t current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
		while (current_peer_id != PM_PEER_ID_INVALID)
		{
			pm_peer_data_bonding_t bonding_data;
			uint8_t app_data[128];
			uint32_t app_len = 128; 
			
			pm_peer_data_bonding_load(current_peer_id, &bonding_data);
			pm_peer_data_app_data_load(current_peer_id, app_data, &app_len);
			
			NRF_LOG_HEXDUMP_INFO(bonding_data.peer_ble_id.id_addr_info.addr, 6);
			NRF_LOG_RAW_INFO("\r");
			NRF_LOG_HEXDUMP_INFO(app_data, app_len);
			
			current_peer_id = pm_next_peer_id_get(current_peer_id);
		}
	}
}

// Test command.
void monet_0command(uint8_t para)
{
//	mcu_mux_set(para);
	ble_pair_test(para);
	BuildFrame('0', &para, 1);
}

// Data format:
// Length(byte):  1  1   1   LEN       1
// Content:      '$' LEN CMD data_body 0x0D
void HandleRxCommand(void)
{
	device_uart_alive_refresh();
    pf_print_mdm_uart_rx(monet_data.iorxframe.cmd, &monet_data.iorxframe.data[0], monet_data.iorxframe.length);
	
    switch(monet_data.iorxframe.cmd) {
		
    case IO_CMD_CHAR_BLE_SEND:
    #if BLE_DATA_CHANNEL_SUPPORT
        monet_bleScommand((&monet_data.iorxframe.data[0]) + 2,
                          monet_data.iorxframe.length - 2,
                          monet_data.iorxframe.data[0],
                          monet_data.iorxframe.data[1]);
    #else
        monet_bleScommand((&monet_data.iorxframe.data[0]), monet_data.iorxframe.length, 0, 0);
    #endif /* BLE_DATA_CHANNEL_SUPPORT */
        break;
    case IO_CMD_CHAR_BLE_CTRL:
		monet_bleCcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;

    case '@':
        monet_AtSigncommand();
        break;
    case 'A':
	    if (6 == monet_data.iorxframe.length)
		{
			monet_configVirtualIgnition(&monet_data.iorxframe.data[0],monet_data.iorxframe.length);
		}
		else
		{
			monet_requestAdc(monet_data.iorxframe.data[0]);
		}
        break;
    case 'B':
        monet_Bcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'C':
        monet_Ccommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'D':
        monet_Dcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'd':
        monet_dcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'E':
        monet_Ecommand(&monet_data.iorxframe.data[0]);
        break;
//    case 'F':	 // Ignored. "F command is not needed but we need a way for the MCU to not turn off the module by toggling the PWR_KEY when the MCU resets", Andy Webster and Michael Silveus, 2020.9.9
//        monet_Fcommand(&monet_data.iorxframe.data[0]);
//        break;
    case 'G':
        monet_Gcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'H':
        monet_Hcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'I':
        monet_Icommand();
		monet_data.phoneLive = INT_MASK_ON; //This is after Icommand
        break;
    case 'J':	// Used for BLE firmware update
        monet_Jcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'K':
        monet_Kcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'L':
        monet_Lcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'M':
        monet_Mcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'm':
        monet_mCommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'N':
        monet_Ncommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'O':
	    monet_Ocommand(&monet_data.iorxframe.data[0]);
	    break;
    case 'P':	//////// MONET_BUBX_ON need flash operation
        monet_Pcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'Q':
        monet_Qcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'R':
        monet_Rcommand(&monet_data.iorxframe.data[0]);
        break;
    case 'S':  // Ignored. "Mostly replaced by the 'Z' command, ..., and can be deprecated", Michael Silveus, 2020.9.9. // Cannot be ignored, used in Simba. 20201011, QGH
        monet_Scommand(&monet_data.iorxframe.data[0]);
        break;
    case 'T':
        monet_Tcommand();
        break;
    case 'U':
	    monet_Ucommand();
	    break;
    case 'V':
		monet_Vcommand();
        break;
    case 'W':
        monet_Wcommand(&monet_data.iorxframe.data[0]);
        break;
    case 'X':
        monet_Xcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 'Y':
        monet_Ycommand();
        break;
    case 'Z': //////// Parameters 'C', 'D', 'G', 'P', 'S', 'T', 'Z' done. 'X' not done
        monet_Zcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case 0xAA:
        monet_AAcommand(&monet_data.iorxframe.data[0], monet_data.iorxframe.length);
        break;
    case '0':
        monet_0command(monet_data.iorxframe.data[0]);
        break;
    default:
        NRF_LOG_RAW_INFO("HandleRxCommand cmd(0x%x) not valid.\r", monet_data.iorxframe.cmd);
        NRF_LOG_FLUSH();
        break;
    }
}
