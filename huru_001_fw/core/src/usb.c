#define NRF_LOG_MODULE_NAME usb 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL
// #define NRF_LOG_LEVEL 4 // Debug

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_power.h"
#include "pt10.h"
#include "usb.h"

NRF_LOG_MODULE_REGISTER();

static void usbd_event_handler(nrf_drv_usbd_evt_t const * const p_event)
{

}

static void power_event_handler(nrf_drv_power_usb_evt_t event)
{
    switch(event)
    {
    case NRF_DRV_POWER_USB_EVT_DETECTED:
        NRF_LOG_INFO("Power detected");
        set_usb_power_state(true);
        break;
    case NRF_DRV_POWER_USB_EVT_REMOVED:
        NRF_LOG_INFO("Power removed");
        set_usb_power_state(false);
        break;
    default:
        break;
    }

}

int32_t usb_init(void)
{
    ret_code_t err;

    err = nrf_drv_usbd_init(usbd_event_handler);
    APP_ERROR_CHECK(err);

    if(!err)
    {
        err = nrf_drv_power_init(NULL);
        APP_ERROR_CHECK(err);
    }

    if(!err)
    {
        static const nrf_drv_power_usbevt_config_t config =
        {
            .handler = power_event_handler
        };
        err = nrf_drv_power_usbevt_init(&config);
    }

    return err;
}
