#ifndef RV3028_H_INCLUDED
#define RV3028_H_INCLUDED

#include <time.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define RV3028_REG_SECONDS                  0x00
#define RV3028_REG_MINUTES                  0x01
#define RV3028_REG_HOURS		    0x02
#define RV3028_REG_WEEKDAY                  0x03
#define RV3028_REG_DATE			    0x04
#define RV3028_REG_MONTH		    0x05
#define RV3028_REG_YEAR			    0x06
#define RV3028_REG_MINUTES_ALARM	    0x07
#define RV3028_REG_HOURS_ALARM		    0x08
#define RV3028_REG_WEEKDAY_ALARM	    0x09
#define RV3028_REG_TIMER_VALUE0             0x0A
#define RV3028_REG_TIMER_VALUE1             0x0B
#define RV3028_REG_TIMER_STATUS0            0x0C
#define RV3028_REG_TIMER_STATUS1            0x0D
#define RV3028_REG_STATUS                   0x0E
#define RV3028_REG_CONTROL1                 0x0F
#define RV3028_REG_CONTROL2                 0x10
#define RV3028_REG_GPBITS                   0x11
#define RV3028_REG_CLOCK_INT_MASK           0x12
#define RV3028_REG_EVENTCONTROL             0x13
#define RV3028_REG_COUNT_TS                 0x14
#define RV3028_REG_SECONDS_TS               0x15
#define RV3028_REG_MINUTES_TS               0x16
#define RV3028_REG_HOURS_TS		    0x17
#define RV3028_REG_DATE_TS		    0x18
#define RV3028_REG_MONTH_TS                 0x19
#define RV3028_REG_YEAR_TS                  0x1A
#define RV3028_REG_UNIXTIME0                0x1B
#define RV3028_REG_UNIXTIME1                0x1C
#define RV3028_REG_UNIXTIME2                0x1D
#define RV3028_REG_UNIXTIME3                0x1E
#define RV3028_REG_USER_RAM1                0x1F
#define RV3028_REG_USER_RAM2                0x20
#define RV3028_REG_PASSWORD0                0x21
#define RV3028_REG_PASSWORD1                0x22
#define RV3028_REG_PASSWORD2                0x23
#define RV3028_REG_PASSWORD3                0x24
#define RV3028_REG_EEADDRESS                0x25
#define RV3028_REG_EEDATA                   0x26
#define RV3028_REG_EECOMMAND                0x27
#define RV3028_REG_ID			    0x28

#define RV3028_EEPROM_EEPWE		    0x30
#define RV3028_EEPROM_PASSWORD0		    0x31
#define RV3028_EEPROM_PASSWORD1		    0x32
#define RV3028_EEPROM_PASSWORD2		    0x33
#define RV3028_EEPROM_PASSWORD3		    0x34
#define RV3028_EEPROM_CLKOUT		    0x35
#define RV3028_EEPROM_OFFSET		    0x36
#define RV3028_EEPROM_BACKUP		    0x37

#define RV3028_EECMD_FIRST                  0x00
#define RV3028_EECMD_UPDATE                 0x11
#define RV3028_EECMD_REFRESH                0x12
#define RV3028_EECMD_WRITE                  0x21
#define RV3028_EECMD_READ                   0x22

#define RV3028_BIT_TRPT           0x07
#define RV3028_BIT_AE_WD		    0x07
#define RV3028_BIT_AE_H			    0x07
#define RV3028_BIT_AE_M			    0x07
#define RV3028_BIT_TSE			    0x07
#define RV3028_BIT_EEBUSY         0x07
#define RV3028_BIT_CLKOE          0x07
#define RV3028_BIT_EHL			    0x06
#define RV3028_BIT_BSIE			    0x06
#define RV3028_BIT_CLKSY		    0x06
#define RV3028_BIT_TCE			    0x05
#define RV3028_BIT_BSF			    0x05
#define RV3028_BIT_WADA			    0x05
#define RV3028_BIT_AMPM			    0x05
#define RV3028_BIT_UIE            0x05
#define RV3028_BIT_FEDE			    0x04
#define RV3028_BIT_ET			    0x04
#define RV3028_BIT_TIE            0x04
#define RV3028_BIT_UF             0x04
#define RV3028_BIT_USEL           0x04
#define RV3028_BIT_AIE			    0x03
#define RV3028_BIT_PORIE		    0x03
#define RV3028_BIT_EERD           0x03
#define RV3028_BIT_TE             0x02
#define RV3028_BIT_AF			    0x02
#define RV3028_BIT_EIE			    0x02
#define RV3028_BIT_TSR			    0x02
#define RV3028_BIT_BSM            0x02
#define RV3028_BIT_TSOW			    0x01
#define RV3028_BIT_EVF			    0x01
#define RV3028_BIT_12_24		    0x01
#define RV3028_BIT_PORF			    0x00
#define RV3028_BIT_RESET		    0x00
#define RV3028_BIT_TSS			    0x00
#define RV3028_BIT_TD             0x00
#define RV3028_BIT_CUIE           0x00


 /**@brief RV3028 I2C device slave address.
  */
 #define RV3028_ADDRESS                      0x52

 /**@brief Error codes for the RV328 driver.
  */
 typedef enum
 {
    RV3028_NO_ERROR		= 0x00,			    /**< No error. */
    RV3028_INVALID_PARAM	= 0x01,			    /**< Invalid parameter passed to function call. */
    RV3028_TIMEOUT		= 0x02,			    /**< Communication timeout. */
    RV3028_NOT_INITIALIZED	= 0x03,			    /**< Device is not initialized. Please call the
								 RV3028_Init function. */
    RV3028_NOT_READY		= 0x04,			    /**< Device function is not ready. Please initialize them first. */
    RV3028_WP_ACTIVE		= 0x05,			    /**< Device is write protected. Please unprotect the device first. */
    RV3028_COMM_ERROR		= 0x06,			    /**< Communication error. */
 } rv3028_error_t;

 /**@brief Hour modes supported by the RV3028.
  */
 typedef enum
 {
    RV3028_HOURMODE_24		= 0x00,			    /**< 24 hour mode. */
    RV3028_HOURMODE_12		= 0x01,			    /**< 12 hour mode. */
 } rv3028_hourmode_t;

 /**@brief Output clock frequencies supported by the RV3028.
  */
 typedef enum
 {
    RV3028_CLKOUT_32KHZ		= 0x00,			    /**< 32.768 kHz output frequency. */
    RV3028_CLKOUT_8KHZ		= 0x01,			    /**< 8192 Hz output frequency. */
    RV3028_CLKOUT_1KHZ		= 0x02,			    /**< 1024 Hz output frequency. */
    RV3028_CLKOUT_64HZ		= 0x03,			    /**< 64 Hz output frequency. */
    RV3028_CLKOUT_32HZ		= 0x04,			    /**< 32 Hz output frequency. */
    RV3028_CLKOUT_1HZ		= 0x05,			    /**< 1 Hz output frequency. */
    RV3028_CLKOUT_PRE		= 0x06,			    /**< Predefined period countdown timer interrupt. */
 } rv3028_clkout_t;

 /**@brief Trickle charger series resistance options supported by the RV3028.
  */
 typedef enum
 {
    RV3028_TCT_3K		= 0x00,			    /**< 3 kOhms series resistance. */
    RV3028_TCT_5K		= 0x01,			    /**< 3 kOhms series resistance. */
    RV3028_TCT_9K		= 0x02,			    /**< 3 kOhms series resistance. */
    RV3028_TCT_15K		= 0x03,			    /**< 3 kOhms series resistance. */
 } rv3028_tcr_t;

 /**@brief Battery switchover modes for the RV3028.
  */
 typedef enum
 {
    RV3028_BAT_DISABLED		= 0x00,			    /**< Battery switchover disabled. */
    RV3028_BAT_DSM		= 0x01,			    /**< Battery direct switching mode (DSM). */
    RV3028_BAT_LSM		= 0x03,			    /**< Battery level switching mode (LSM). */
 } rv3028_bat_t;

 /**@brief Status flags that can be modified by the user.
  */
 typedef enum
 {
    RV3028_FLAG_POR		= (0x01 << 0x00),	    /**< Power On Reset flag. */
    RV3028_FLAG_EVENT		= (0x01 << 0x01),	    /**< Event flag. */
    RV3028_FLAG_ALARM		= (0x01 << 0x02),	    /**< Alarm flag. */
    RV3028_FLAG_COUNTDOWN	= (0x01 << 0x03),	    /**< Periodic Time Countdown flag. */
    RV3028_FLAG_UPDATE		= (0x01 << 0x04),	    /**< Periodic Time Update flag. */
    RV3028_FLAG_BATTERY		= (0x01 << 0x05),	    /**< Battery Switch flag. */
 } rv3028_flags_t;

 /**@brief Time stamp source for the RV3028.
  */
 typedef enum
 {
    RV3028_TS_EVENT		= 0x00,			    /**< Use an external event as time stamp source. */
    RV3028_TS_BAT		= 0x01,			    /**< Use the automatic backup switchover as time stamp source. */
 } rv3028_ts_src_t;

 /**@brief Event filtering options for the RV3028.
  */
 typedef enum
 {
    RV3028_FILTER_NO		= 0x00,			    /**< No filtering. Edge detection enabled. */
    RV3028_FILTER_256HZ		= 0x01,			    /**< Sampling period 3.9 ms. Level detection. */
    RV3028_FILTER_64HZ		= 0x02,			    /**< Sampling period 15.6 ms. Level detection. */
    RV3028_FILTER_8HZ		= 0x03,			    /**< Sampling period 125 ms. Level detection. */
 } rv3028_evt_filter_t;

 /**@brief Period time update interrupt sources.
  */
 typedef enum
 {
    RV3028_UPDATE_SECOND	= 0x00,			    /**< Periodic time update every second. */
    RV3028_UPDATE_MINUTE	= 0x01,			    /**< Periodic time update every minute. */
 } rv3028_ud_src_t;

 /**@brief Period countdown clock frequencies.
  */
 typedef enum
 {
    RV3028_COUNTDOWN_4096HZ	= 0x00,			    /**< Timer Clock Frequency: 4096 Hz. */
    RV3028_COUNTDOWN_64HZ	= 0x01,			    /**< Timer Clock Frequency: 64 Hz. */
    RV3028_COUNTDOWN_1HZ	= 0x02,			    /**< Timer Clock Frequency: 1 Hz. */
    RV3028_COUNTDOWN_1HZ60	= 0x03,			    /**< Timer Clock Frequency: 1/60 Hz. */
 } rv3028_cd_freq_t;

 typedef enum
 {
   RV3028_PERIODIC_UPDATE_1HZ = 0x00,
   RV3028_PERIODIC_UPDATE_1HZ60 = 0x01
 } rv3028_tu_freq_t;

 /**@brief		Bus communication function pointer which should be mapped to the platform specific read functions of the user.
  * @param Device_Addr	I2C device address.
  * @param Reg_Addr	Register address.
  * @param Reg_Data	Data from the specified address.
  * @param Length	Length of the reg_data array.
  * @return		Communication error code.
  */
 typedef uint8_t (*rv3028_read_fptr_t)(uint8_t Device_Addr, uint8_t Reg_Addr, uint8_t* p_Reg_Data, uint32_t Length);

 /**@brief		Bus communication function pointer which should be mapped to the platform specific write functions of the user.
  * @param Device_Addr	I2C device address.
  * @param Reg_Addr	Register address.
  * @param Reg_Data	Data to the specified address.
  * @param Length	Length of the reg_data array.
  * @return		Communication error code.
  */
 typedef uint8_t (*rv3028_write_fptr_t)(uint8_t Device_Addr, uint8_t Reg_Addr, const uint8_t* p_Reg_Data, uint32_t Length);

 /**@brief RV3028 device initialization object structure.
  */
 typedef struct
 {
    bool		DisableSync;			    /**< Boolean flag to disable the sync for the CLKOUT pin. */
    bool		EnableEventInt;			    /**< Set to #true to enable the event interrupt function on the INT pin. */
    bool		EventHighLevel;			    /**< Set to #true to enable the rising edge or high level event detection. */
    bool		EnableTS;			    /**< Set to #true to enable the time stamp mode. */
    bool		EnableTSOverwrite;		    /**< Set to #true to enable the overwrite mode in time stamp mode. */
    bool		EnableClkOut;			    /**< Boolean flag to enable the CLKOUT function. */
    bool		EnablePOR;			    /**< Boolean flag to enable the POR function on the INT pin. */
    bool		EnableBSIE;			    /**< Boolean flag to enable the BSIE function on the INT pin. */
    bool		EnableCharge;			    /**< Boolean flag to enable the trickle charger function.
								 NOTE: Only needed when \ref rv3028_t.BatteryMode is enabled. */

    rv3028_ts_src_t	TSMode;				    /**< Time stamp mode used by the RV3028.
								 NOTE: Only needed when \ref rv3028_t.EnableTS is set to #true. */
    rv3028_evt_filter_t Filter;				    /**< Event filter options. */
    rv3028_bat_t	BatteryMode;			    /**< Battery mode used by the RV3028. */
    rv3028_clkout_t	Frequency;			    /**< Output frequency for the CLKOUT pin.
								 NOTE: Only needed when \ref rv3028_t.EnableClkOut is set to #true. */
    rv3028_tcr_t	Resistance;			    /**< Trickle charger series resistance selection.
								 NOTE: Only needed when \ref rv3028_t.BatteryMode is enabled. */
    rv3028_hourmode_t	HourMode;			    /**< Hour mode for the RTC. */

    struct tm*		p_CurrentTime;			    /**< Pointer to initial time for the RTC. */

    uint32_t		CurrentUnixTime;		    /**< Initial Unix time for the RTC. */
    uint32_t		Password;			    /**< Initial password for the RTC. Set to a value > 0 to enable the password function. */
 } rv3028_init_t;

 /**@brief RV3028 device object structure.
  */
 typedef struct
 {
    uint8_t		HID;				    /**< Hardware ID from the RTC. */
    uint8_t		VID;				    /**< Version ID from the RTC. */
    uint8_t		DeviceAddr;			    /**< RTC device address. */

    bool		IsInitialized;			    /**< Boolean flag to indicate a successful initialization. */
    bool		IsPOREnabled;			    /**< Current state of the POR function of the INT pin. */
    bool		IsBSIEEnabled;			    /**< Current state of the BSIE function of the INT pin. */
    bool		IsEventIntEnabled;		    /**< Boolean flag to indicate the state of the EIE bit. */
    bool		IsEventHighLevel;		    /**< Boolean flag to indicate the use of high level events on EVI. */
    bool		IsPasswordEnabled;		    /**< Current state of the password function. */
    bool		IsAlarmEnabled;			    /**< Current state of the alarm function. */
    bool		IsTSEnabled;			    /**< Current state of the time stamp function. */
    bool		IsTSOverwriteEnabled;		    /**< Current state of the time stamp overwrite function. */
    bool		IsClkOutEnabled;		    /**< Current state of the CLKOUT function. */
    bool		IsChargeEnabled;		    /**< Current state of the charging function. */

    rv3028_evt_filter_t Filter;				    /**< Selected event filter options. */
    rv3028_ts_src_t	TSMode;				    /**< Time stamp mode used by the RV3028. */
    rv3028_bat_t	BatteryMode;			    /**< Battery mode used by the RV3028. */
    rv3028_hourmode_t	HourMode;			    /**< Current hour mode of the RTC. */
    rv3028_clkout_t	Frequency;			    /**< Output frequency for the CLKOUT pin. */
    rv3028_tcr_t	Resistance;			    /**< Trickle charger series resistance selection. */
    rv3028_read_fptr_t	p_Read;				    /**< Pointer to RV3028 I2C read function. */
    rv3028_write_fptr_t p_Write;			    /**< Pointer to RV3028 I2C write function. */
 } rv3028_t;

 /**@brief RV3028 alarm configuration object structure.
  */
 typedef struct
 {
    bool		EnableInterrupts;		    /**< Set to #true to enable enable the alarm interrupt on INT. */
    bool		EnableMinutesAlarm;		    /**< Set to #true to enable the minutes alarm. */
    bool		EnableHoursAlarm;		    /**< Set to #true to enable the hours alarm. */
    bool		EnableDayAlarm;			    /**< Set to #true to enable the weekday / date alarm. */
    bool		PM;				    /**< PM hours used by the hours alarm. 
								 NOTE: Only important when \ref rv3028_alarm_t.EnableHoursAlarm is set to #true.
								 NOTE: Only important when \ref rv3028_init_t.HourMode is set to #RV3028_HOURMODE_12. 
								 NOTE: The alarm value must be re-initialized when the hour mode is changed! */
    bool		UseDateAlarm;			    /** Set to #true to use the date alarm instead of the weekday alarm. */
    uint8_t		Minutes;			    /**< Value for the minutes alarm.
								 NOTE: Only important when \ref rv3028_alarm_t.EnableMinutesAlarm is set to #true. */
    uint8_t		Hours;				    /**< Value for the hours alarm.
								 NOTE: Only important when \ref rv3028_alarm_t.EnableHoursAlarm is set to #true. */
    uint8_t		Day;				    /**< Value for the weekday / date alarm.
								 NOTE: Only important when \ref rv3028_alarm_t.EnableDayAlarm is set to #true. */
 } rv3028_alarm_t;

 /**@brief RV3028 periodic countdown configuration object structure.
  */
 typedef struct
 {
    bool		EnableRepeat;                       /**< Set to #true to enable the repeat mode. */
    bool		UseInt;                             /**< Set to #true to enable the INT pin. */
    bool		UseClockOut;                        /**< Set to #true to enable the CLKOUT function of the timer. */
    rv3028_cd_freq_t    Frequency;                          /**< Countdown timer frequency. */
    uint16_t		Value;				    /**< Value for the countdown timer. */
 } rv3028_cd_config_t;


 typedef struct
 {
   bool UseClockOut;
   bool UseInt;
   rv3028_tu_freq_t Frequency;
 } rv3028_tu_config_t;

 /**@brief		Initialize the RTC.
  * @param p_Init	Pointer to device initialization structure.
  * @param p_Device	Pointer to RV3028 device structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_Init(rv3028_init_t* p_Init, rv3028_t* p_Device);

 /**@brief		Configure the periodic update interrupt.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Source	Update source.
  * @param UseInt	Set to #true to enable the INT pin.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_InitUpdate(rv3028_t* p_Device, rv3028_ud_src_t Source, bool UseInt);

 /**@brief		Configure the periodic countdown timer.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Config	Pointer to periodic countdown configuration structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_InitCountdown(rv3028_t* p_Device, rv3028_cd_config_t* p_Config);

 /**@brief		Disable the periodic countdown timer.
  * @param p_Device	Pointer to RV3028 device structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_DisableCountdown(rv3028_t* p_Device);

 rv3028_error_t RV3028_UpdateCountdown(rv3028_t* p_Device, uint16_t seconds);

 rv3028_error_t RV3028_InitPeriodicTimeUpdate(rv3028_t* p_Device, rv3028_tu_config_t* p_Config);
 rv3028_error_t RV3028_DisablePeriodicTimeUpdate(rv3028_t* p_Device);

 /**@brief		Disable the write protection of the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Password	Device password.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_DisableWP(rv3028_t* p_Device, uint32_t Password);

 /**@brief		Unlock the device.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Password	Password for the RTC.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_UnlockWP(rv3028_t* p_Device, uint32_t Password);

 /**@brief		Reset the clock prescaler of the device.
  * @param p_Device	Pointer to RV3028 device structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_Reset(rv3028_t* p_Device);

 /**@brief		Adjust the oscillator offset for frequency compensation purposes.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Offset	Oscillator offset.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_Compensate(rv3028_t* p_Device, uint16_t Offset);

 /**@brief		Read the status register to get the pending interrupt flags.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Flags	Pointer to interrupt status flags.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetFlags(rv3028_t* p_Device, uint8_t* p_Flags);

 /**@brief		Clear one or more interrupt status flags.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Mask		Interrupt status flags mask.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_ClearFlags(rv3028_t* p_Device, rv3028_flags_t Mask);

 /**@brief		Enable / Disable the POR function of the INT pin.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Enable	Enable / Disable status for the CLKOUT pin.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_EnablePOR(rv3028_t* p_Device, bool Enable);

 /**@brief		Check if a battery switchover has occured.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Active	#true when the battery is active.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_CheckBatterySwitch(rv3028_t* p_Device, bool* p_Active);

 /**@brief		Enable / Disable the CLKOUT pin.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Enable	Enable / Disable status for the CLKOUT pin.
  * @param DisableSync	Set to #true to disable the synchronization of the CLKOUT pin.
  *			NOTE: Only important when setting \p Enable to #true.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_EnableClkOut(rv3028_t* p_Device, bool Enable, bool DisableSync);

 /**@brief		Set the output frequency for the CLKOUT pin.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Frequency	Output frequency for the CLKOUT pin.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetClkOut(rv3028_t* p_Device, rv3028_clkout_t Frequency);

 /**@brief		Set the series resistance of the trickle charger.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Resistance	Series resistance selection.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetResistance(rv3028_t* p_Device, rv3028_tcr_t Resistance);

 /**@brief		Set the hour mode for the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Mode		Hour mode option.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetHourMode(rv3028_t* p_Device, rv3028_hourmode_t Mode);

 /**@brief		Write a single byte into the user EEPROM.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Address	EEPROM address.
  * @param Data		Data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetEEPROM(rv3028_t* p_Device, uint8_t Address, uint8_t Data);

 /**@brief		Read a single byte from the user EEPROM.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Address	EEPROM address.
  * @param p_Data	Pointer to data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetEEPROM(rv3028_t* p_Device, uint8_t Address, uint8_t* p_Data);

 /**@brief		Write a single byte into the user RAM.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Address	RAM address (1 or 2).
  * @param Data		Data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetRAM(rv3028_t* p_Device, uint8_t Address, uint8_t Data);

 /**@brief		Read a single byte into the user RAM.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Address	RAM address (1 or 2).
  * @param p_Data	Pointer to data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetRAM(rv3028_t* p_Device, uint8_t Address, uint8_t* p_Data);

 /**@brief		Write a byte into the GP bits register.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Data		Data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetGP(rv3028_t* p_Device, uint8_t Data);

 /**@brief		Modify the bits in the GP bits register.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Mask		Bit modification mask.
  * @param Value	New value for masked bits.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_ModifyGP(rv3028_t* p_Device, uint8_t Mask, uint8_t Value);

 /**@brief		Read a byte from the GP bits register.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Data	Pointer to data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetGP(rv3028_t* p_Device, uint8_t* p_Data);

 /**@brief		Write a Unix time into the RTC
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Time		Unix timestamp.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetUnixTime(rv3028_t* p_Device, uint32_t Time);

 /**@brief		Read the Unix time from the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Time	Pointer to Unix timestamp.
  * @return		Error code
  */
 rv3028_error_t RV3028_GetUnixTime(rv3028_t* p_Device, uint32_t* p_Time);

 /**@brief		Set the time of the RTC
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Time	Pointer to time structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetTime(rv3028_t* p_Device, struct tm* p_Time);

 /**@brief		Read the time from the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Time	Pointer to time structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetTime(rv3028_t* p_Device, struct tm* p_Time);

 /**@brief		Enable the time stamp function of the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Mode		Time stamp source mode.
  * @param OverWrite	Set to #true to enable the overwriting of an existing time stamp.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_EnableTS(rv3028_t* p_Device, rv3028_ts_src_t Mode, bool OverWrite);

 /**@brief		Disable the time stamp function of the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_DisableTS(rv3028_t* p_Device);

 /**@brief		Get the time from the time stamp register.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Time	Pointer to time structure.
  * @param p_Count	Pointer to the corresponding event occurrences.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetTS(rv3028_t* p_Device, struct tm* p_Time, uint8_t* p_Count);

 /**@brief		Enable and configure the alarm.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Alarm	Pointer to alarm configuration structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_EnableAlarm(rv3028_t* p_Device, rv3028_alarm_t* p_Alarm);

 /**@brief		Disable the minutes alarm.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Minutes	Set to #true to disable the minutes alarms.
  * @param Hours	Set to #true to disable the hours alarms.
  * @param Days		Set to #true to disable the weekdays / date alarms.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_DisableAlarm(rv3028_t* p_Device, bool Minutes, bool Hours, bool Days);

 rv3028_error_t RV3028_RegDump(rv3028_t *p_Device, uint8_t *regbuf, uint8_t n_regs);
#endif