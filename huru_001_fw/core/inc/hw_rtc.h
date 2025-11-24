#ifndef HW_RTC_H_INCLUDED
#define HW_RTC_H_INCLUDED

#include "rv3028.h"

#define PASSWORD    (0x20)
#define RESET_TIME  (1609459200)

int32_t hw_rtc_init(void);
int32_t hw_rtc_get_unix_time(uint32_t *unix_time);
int32_t hw_rtc_set_unix_time(uint32_t unix_time);
int32_t hw_rtc_enable_periodic_countdown_timer(uint16_t seconds);
int32_t hw_rtc_disable_periodic_countdown_timer(void);
int32_t hw_rtc_update_periodic_countdown_timer(uint16_t seconds);
int32_t hw_rtc_enable_periodic_update(void);
int32_t hw_rtc_disable_periodic_update(void);
void hw_rtc_register_countdown_timer_callback(void (*callback)(void));
int32_t hw_rtc_enable_clkout(bool enable, bool disable_sync);
int32_t hw_rtc_dump_registers(void);

#endif