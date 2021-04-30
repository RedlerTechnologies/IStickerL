#include "state_machine.h"

#include "nrf_log_ctrl.h"
#include "nrfx_rtc.h"
#include "nrfx_wdt.h"

#define NRF_LOG_MODULE_NAME logic_state_machine
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

static void init_wdt(void);
static void init_rtc(void);

static void wdt_event_handler(void);
static void rtc_handler(nrfx_rtc_int_type_t int_type);

static nrfx_wdt_channel_id m_wdt_channel_id;

static const nrfx_rtc_t rtc = NRFX_RTC_INSTANCE(2);

void state_machine_init(void)
{
    //
    //init_wdt();
    //init_rtc();
}

static void init_wdt(void)
{
    ret_code_t err_code;

    const nrfx_wdt_config_t config = NRFX_WDT_DEAFULT_CONFIG;

    err_code = nrfx_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_wdt_channel_alloc(&m_wdt_channel_id);
    APP_ERROR_CHECK(err_code);

    nrfx_wdt_enable();
}

static void init_rtc(void)
{
    ret_code_t err_code;

    // Initialize RTC instance
    nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;
    config.prescaler         = 4095;

    err_code = nrfx_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    // Configure the RTC for 20 seconds wakeup
    err_code = nrfx_rtc_cc_set(&rtc, NRFX_RTC_INT_COMPARE0, 20 * 8, true);
    APP_ERROR_CHECK(err_code);

    nrfx_rtc_enable(&rtc);
}

void state_machine_feed_watchdog(void)
{
    //
    nrfx_wdt_channel_feed(m_wdt_channel_id);
}

static void wdt_event_handler(void)
{
    // NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
    NRFX_LOG_INFO("%s", __func__);
}

static void rtc_handler(nrfx_rtc_int_type_t int_type)
{
    if (int_type == NRFX_RTC_INT_COMPARE0) {
        NRFX_LOG_INFO("%s RTC Event", __func__);
    }
}
