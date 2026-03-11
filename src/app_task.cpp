#include "app_task.h"
#include "app/matter_init.h"
#include "app/task_executor.h"
#include "board/board.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <platform/CHIPDeviceEvent.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::DeviceLayer;

/* Debounce time in milliseconds */
#define DEBOUNCE_MS 50

/* Sensors */
static const struct gpio_dt_spec s0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec s1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

/* GPIO callbacks */
static struct gpio_callback s0_cb_data;
static struct gpio_callback s1_cb_data;

/* Debounce timestamps */
static int64_t s0_last_trigger = 0;
static int64_t s1_last_trigger = 0;

/* Timer */
static struct k_timer sBatteryTimer;

/* ADC */
static const struct device *adc_dev;
static int16_t adc_sample;

static struct adc_channel_cfg vdd_channel_cfg = {
    .gain             = ADC_GAIN_1_6,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 0,
    .input_positive   = SAADC_CH_PSELP_PSELP_VDD,
};

static struct adc_sequence adc_sequence = {
    .channels    = BIT(0),
    .buffer      = &adc_sample,
    .buffer_size = sizeof(adc_sample),
    .resolution  = 12,
};

/* ---------------------------------------------------------- */
/* Battery                                                      */
/* ---------------------------------------------------------- */

static uint16_t read_battery_mv()
{
    if (!adc_dev) return 0;
    adc_read(adc_dev, &adc_sequence);
    int32_t mv = (int32_t)adc_sample * 3600 / 4096;
    if (mv < 0) mv = 0;
    return (uint16_t)mv;
}

static void update_battery()
{
    uint16_t v_mv = read_battery_mv();
    uint8_t pct = 0;

    if (v_mv >= 3000)      pct = 200;
    else if (v_mv <= 2000) pct = 0;
    else                   pct = (uint8_t)((v_mv - 2000) * 200 / 1000);

    chip::app::Clusters::PowerSource::Attributes::BatVoltage::Set(0, v_mv);
    chip::app::Clusters::PowerSource::Attributes::BatPercentRemaining::Set(0, pct);
}

/* ---------------------------------------------------------- */
/* Sensor state report                                         */
/* ---------------------------------------------------------- */

static void report_sensor_states()
{
    /* Sensor 1 - normal logic */
    /* Open = dry, Closed = wet */
    chip::app::Clusters::BooleanState::Attributes::StateValue::Set(
        1, gpio_pin_get_dt(&s0) > 0);

    /* Sensor 2 - to have inverted logic (&s1) == 0); */
    chip::app::Clusters::BooleanState::Attributes::StateValue::Set(
        2, gpio_pin_get_dt(&s1) > 0);
}

/* ---------------------------------------------------------- */
/* Matter event handler                                        */
/* ---------------------------------------------------------- */

static void MatterEventHandler(const ChipDeviceEvent *event, intptr_t arg)
{
    Nrf::Board::DefaultMatterEventHandler(event, arg);

    switch (event->Type)
    {
    case DeviceEventType::kThreadConnectivityChange:
        if (event->ThreadConnectivityChange.Result == kConnectivity_Established)
        {
            /* Turn off LED via board library */
            Nrf::GetBoard().GetLED(Nrf::DeviceLeds::LED1).Set(false);

            /* Report current states */
            report_sensor_states();
            update_battery();
        }
        break;

    case DeviceEventType::kCHIPoBLEAdvertisingChange:
        /* Turn off LED after BLE advertising stops */
        if (!event->CHIPoBLEAdvertisingChange.Result)
        {
            Nrf::GetBoard().GetLED(Nrf::DeviceLeds::LED1).Set(false);
        }
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------- */
/* Sensor interrupts with debounce                             */
/* ---------------------------------------------------------- */

static void sensor0_changed(const struct device *dev,
                             struct gpio_callback *cb, uint32_t pins)
{
    int64_t now = k_uptime_get();
    if (now - s0_last_trigger < DEBOUNCE_MS) return;
    s0_last_trigger = now;

    Nrf::PostTask([] {
        /* Sensor 1 - normal logic */
        /* Open = dry, Closed = wet */
        int val = gpio_pin_get_dt(&s0);
        
        /* below is inverted logic but i am just reversing the float 
            chip::app::Clusters::BooleanState::Attributes::StateValue::Set(1, val == 0); it could also be done Set(2, !val); */
        chip::app::Clusters::BooleanState::Attributes::StateValue::Set(1, val > 0);
    });
}

static void sensor1_changed(const struct device *dev,
                             struct gpio_callback *cb, uint32_t pins)
{
    int64_t now = k_uptime_get();
    if (now - s1_last_trigger < DEBOUNCE_MS) return;
    s1_last_trigger = now;

    Nrf::PostTask([] {
        /* Sensor 2 - inverted logic (physically reversed reed switch) */
        /* Open = wet, Closed = dry - both open = low power */
        int val = gpio_pin_get_dt(&s1);
        chip::app::Clusters::BooleanState::Attributes::StateValue::Set(2, val > 0);
    });
}

/* ---------------------------------------------------------- */
/* Timer callbacks                                             */
/* ---------------------------------------------------------- */

static void BatteryTimerCallback(k_timer *timer)
{
    Nrf::PostTask([] { update_battery(); });
}

/* ---------------------------------------------------------- */
/* AppTask                                                     */
/* ---------------------------------------------------------- */

CHIP_ERROR AppTask::Init()
{
    ReturnErrorOnFailure(Nrf::Matter::PrepareServer());

    if (!Nrf::GetBoard().Init()) {
        LOG_ERR("User interface initialization failed.");
        return CHIP_ERROR_INCORRECT_STATE;
    }

    /* Register our event handler */
    ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(
        MatterEventHandler, 0));

    /* Configure sensor GPIOs */
    gpio_pin_configure_dt(&s0, GPIO_INPUT);
    gpio_pin_configure_dt(&s1, GPIO_INPUT);

    /* Configure interrupts on both edges */
    gpio_pin_interrupt_configure_dt(&s0, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&s1, GPIO_INT_EDGE_BOTH);

    /* Register GPIO callbacks */
    gpio_init_callback(&s0_cb_data, sensor0_changed, BIT(s0.pin));
    gpio_init_callback(&s1_cb_data, sensor1_changed, BIT(s1.pin));
    gpio_add_callback(s0.port, &s0_cb_data);
    gpio_add_callback(s1.port, &s1_cb_data);

    /* Init ADC */
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    if (device_is_ready(adc_dev)) {
        adc_channel_setup(adc_dev, &vdd_channel_cfg);
    } else {
        adc_dev = nullptr;
    }

    /* Battery update every 6 hours */
    k_timer_init(&sBatteryTimer, BatteryTimerCallback, nullptr);
    k_timer_start(&sBatteryTimer, K_HOURS(6), K_HOURS(6));

    return Nrf::Matter::StartServer();
}

CHIP_ERROR AppTask::StartApp()
{
    ReturnErrorOnFailure(Init());
    while (true) {
        Nrf::DispatchNextTask();
    }
    return CHIP_NO_ERROR;
}