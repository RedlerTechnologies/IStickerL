#include "buzzer.h"

#include "hal/hal_boards.h"
#include "hal/hal_drivers.h"

#define NRF_LOG_MODULE_NAME cloud_wise_sdk_drivers_buzzer
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

/*
static uint16_t m_buzzerTune[5] = {HAL_BUZZER_PWM_CYCLE_COUNT - (0), //
                                   HAL_BUZZER_PWM_CYCLE_COUNT - (1 * (HAL_BUZZER_PWM_CYCLE_COUNT / 100)),
                                   HAL_BUZZER_PWM_CYCLE_COUNT - (2 * (HAL_BUZZER_PWM_CYCLE_COUNT / 100)),
                                   HAL_BUZZER_PWM_CYCLE_COUNT - (5 * (HAL_BUZZER_PWM_CYCLE_COUNT / 100))};
                                   */

static uint16_t m_buzzerTune[2] = {HAL_BUZZER_PWM_CYCLE_COUNT/10};

static nrf_pwm_sequence_t m_tuneSequence = {
    .values = m_buzzerTune, .length = NRF_PWM_VALUES_LENGTH(m_buzzerTune), .repeats = 80, .end_delay = 0};

static uint16_t m_buzzerTune1[2] = {HAL_BUZZER_PWM_CYCLE_COUNT/10};

static nrf_pwm_sequence_t m_tuneSequence1 = {
    .values = m_buzzerTune1, .length = NRF_PWM_VALUES_LENGTH(m_buzzerTune), .repeats = 1000, .end_delay = 0};

bool buzzer_init(void)
{
    return true;
}

bool buzzer_train_test(uint8_t repeats)
{
    #ifdef BUZZER_DISABLE
    nrfx_pwm_simple_playback(hal_buzzer, &m_tuneSequence, repeats, NRFX_PWM_FLAG_STOP);
    return true;
    #endif
}

bool buzzer_train(uint8_t repeats)
{
    #ifndef BUZZER_DISABLE
    nrfx_pwm_simple_playback(hal_buzzer, &m_tuneSequence, repeats, NRFX_PWM_FLAG_STOP);
    return true;
    #endif
}

bool buzzer_long(uint16_t time_ms)
{
    #ifndef BUZZER_DISABLE
    m_tuneSequence1.repeats = time_ms;
    nrfx_pwm_simple_playback(hal_buzzer, &m_tuneSequence1, 1, NRFX_PWM_FLAG_STOP);
    return true;
    #endif
}

bool buzzer_end(void)
{
    //
    return true;
}