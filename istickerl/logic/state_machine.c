#include "state_machine.h"

#include "nrf_log_ctrl.h"
#include "nrfx_wdt.h"

#define NRF_LOG_MODULE_NAME logic_statemanager
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

static void init_wdt(void);
static void wdt_event_handler(void);

static nrfx_wdt_channel_id m_wdt_channel_id;

void state_machine_init(void)
{
    //
    init_wdt();
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

void state_machine_feed_watchdog(void) { nrfx_wdt_channel_feed(m_wdt_channel_id); }

static void wdt_event_handler(void)
{
    // NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
    NRFX_LOG_INFO("%s", __func__);
}
