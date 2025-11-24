#ifndef POWER_H_INCLUDED
#define POWER_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define ADC_REF_MV              (600)
#define RDIV_GAIN               (0.5)
#define PREAMP_GAIN             (1.0/6.0)
#define ADC_BITS                (12)
#define ADC_MAX                 (1<<ADC_BITS)

#define VBAT_EN_RISE_TIME_US    (250)

#define VBAT_FILTER_LENGTH_LOG2 (4)
#define VBAT_FILTER_LENGTH      (1<<VBAT_FILTER_LENGTH_LOG2)

#define BATTERY_EMPTY_MV        (3000)
#define BATTERY_FULL_MV         (4350)

#define BATTERY_VERTICAL_SCALE  (1.0 / 1000.0)

typedef enum {
    BATTERY_STATE_UNKNOWN = 0,
    BATTERY_STATE_LOW_BATTERY = 1,
    BATTERY_STATE_EMPTY_BATTERY = 2,
    BATTERY_STATE_FULL_BATTERY = 4,
    BATTERY_STATE_IN_USE_BATTERY = 8,
    BATTERY_STATE_ERROR = 16,
    
} pt10_battery_state_e;

typedef enum {
    BATTERY_CHARGE_UNKNOWN = 0,
    BATTERY_CHARGE_ERROR,
    BATTERY_CHARGE_CHARGING,
    BATTERY_CHARGE_COMPLETED,
    BATTERY_CHARGE_DISCONNECTED
    
} pt10_charge_state_e;

void set_usb_power_state(bool usb_power_state);
bool get_usb_power_state(void);

uint8_t get_battery_percent(void);
uint16_t get_battery_mv(void);

void update_power(void);
int32_t update_battery_voltage(void);

pt10_battery_state_e get_battery_state(void);

pt10_charge_state_e get_charge_state(void);



#endif