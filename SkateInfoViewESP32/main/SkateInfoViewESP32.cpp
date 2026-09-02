/*
 * SkateInfoView
 * An electric skateboard battery monitor based on ESP32
 */



#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include "esp_mac.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/spi_master.h"
#include "hal/spi_types.h"
#include "esp_log.h"
#include <string.h>
#include "esp_system.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include <stdlib.h>
#include <math.h>
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "cJSON.h"
#include "Pizzatronix.h"

#include "RGB_LED.h"
#include "TCS34725.h"
#include "BoardConfiguration.h"
#include "ESPCANController.h"
#include "MCP2515_CANController.h"
#include "TMP117.h"
#include "driver/i2c_master.h"



static const char *TAG = "skateinfo";

// Hardware mapping - adjust these for your board/pins
#define HALL_LEFT_A_GPIO        GPIO_NUM_40
#define HALL_LEFT_B_GPIO        GPIO_NUM_39
#define HALL_LEFT_C_GPIO        GPIO_NUM_38

#define HALL_RIGHT_A_GPIO       GPIO_NUM_21
#define HALL_RIGHT_B_GPIO       GPIO_NUM_18
#define HALL_RIGHT_C_GPIO       GPIO_NUM_17

#define UART_ISO_TX             GPIO_NUM_37
#define UART_ISO_RX             GPIO_NUM_36

#define I2C_SDA                 GPIO_NUM_10
#define I2C_SCL                 GPIO_NUM_9

#define EXP_5V_GPIO             GPIO_NUM_41
#define EXP_12V_GPIO            GPIO_NUM_42
#define PWR_LATCH_GPIO          GPIO_NUM_2
#define ESC_PWR_SENSE_GPIO      GPIO_NUM_1

// Board "BOOT" strap pin; free for use as a regular input once the app is running
#define MODE_BUTTON_GPIO        GPIO_NUM_0

#define HEADLIGHT_OUT_GPIO      GPIO_NUM_15
#define HEADLIGHT_LEDC_CHANNEL  LEDC_CHANNEL_3
#define BRAKELIGHT_OUT_GPIO     GPIO_NUM_16
#define BRAKELIGHT_LEDC_CHANNEL LEDC_CHANNEL_4


// SPI pins for MCP2515 CAN Controller
#define SPI_MOSI_PIN         GPIO_NUM_13
#define SPI_MISO_PIN         GPIO_NUM_12
#define SPI_CLK_PIN          GPIO_NUM_14
#define SPI_CS_PIN           GPIO_NUM_11

// TX/RX pins for integrated ESP2 CAN Controller
#define ESP_CANRX            GPIO_NUM_12
#define ESP_CANTX            GPIO_NUM_13

// ADC channels (update to match new wiring)
#define ADC_BATT_CHANNEL     ADC_CHANNEL_4  // ADC1_4 is actually IO5. For battery voltage
#define ADC_CURR_HS_CHANNEL  ADC_CHANNEL_3  // ADC1_3 is actually IO4. For battery current
#define ADC_CURR_REF_CHANNEL ADC_CHANNEL_2  // ADC1_2 is actually IO3. For battery current reference
#define ADC_12V_CURR_CHANNEL ADC_CHANNEL_5  // ADC1_5 is actually IO6. For 12V Power rail current
#define ADC_5V_CURR_CHANNEL  ADC_CHANNEL_6  // ADC1_6 is actually IO7. For 5V Power rail current

// Sampling / timings
#define PWR_POLL_MS             100
#define RPM_POLL_MS             300
#define IDLE_POWEROFF_MS        300000

// Power Macros
#define BATT_CELL_MIN           3.1         // Voltage at which cell is considered to be 0% charged         
#define BATT_CELL_MAX           4.0         // Voltage at which cell is considered to be 100% charged
#define BATT_CELL_COUNT         12          // Number of series cell in pack. TODO: make this NVS storeable

#if CONFIG_BT_NIMBLE_ENABLED
/* Main BLE Stack - initializes advertising and services */
BLEStack ble("SK01");

/* UART/Telemetry Service instance - initialized during BLE setup */
static BLEUARTService uart_service(
    "12342001-B5A3-F393-E0A9-E50E24DCCA9E",
    "12342002-B5A3-F393-E0A9-E50E24DCCA9E",
    "12342003-B5A3-F393-E0A9-E50E24DCCA9E",
    16);

/* OTA Service instance - initialized during BLE setup */
static BLEOTAService ota_service;

/* Separate UART service used by the CAN analyzer console. */
static BLEUARTService can_console_uart_service(64);
static BLESerialConsole can_console(can_console_uart_service);
/* USB CDC console; only installed when BoardConfiguration selects STANDARD_MENU. */
static USBSerialConsole usb_console;
/* Combines BLE and USB transports; USB legs are inert until usb_console.begin() succeeds. */
ISerialInterfaceCombiner console_combiner(&can_console, &usb_console);
static constexpr uint16_t CANALYZER_RX_BUFFER_SIZE = 64;
static CANMessage canalyzer_rx_buffer[CANALYZER_RX_BUFFER_SIZE];
static uint16_t canalyzer_rx_buffer_index = 0;
static constexpr uint16_t CANALYZER_TX_BUFFER_SIZE = 20;
static CANMessage canalyzer_tx_buffer[CANALYZER_TX_BUFFER_SIZE];
static uint16_t canalyzer_tx_buffer_index = 0;
static CANAnalyzerStack *canalyzer = nullptr;

static void canalyzer_tx_callback(const CANMessage &message, void *)
{
    canalyzer_tx_buffer[canalyzer_tx_buffer_index] = message;
    canalyzer_tx_buffer_index = static_cast<uint16_t>(
        (canalyzer_tx_buffer_index + 1) % CANALYZER_TX_BUFFER_SIZE);
}

static uint64_t canalyzer_get_millis()
{
    return static_cast<uint64_t>(esp_timer_get_time() / 1000);
}

static void root_menu_on_enter()
{
    console_combiner.writeLine("Welcome to the SkateInfoView configuration menu");
}

static MenuItem root_menu("root", root_menu_on_enter);
static void diagnostics_menu_on_enter()
{
    console_combiner.writeLine("Entered diagnostics menu");
}

static void canalyzer_menu_on_enter()
{
    if (canalyzer != nullptr) {
        canalyzer->printHelp();
    }
}

static MenuItem diagnostics_menu("diagnostics", diagnostics_menu_on_enter);
static MenuSystem menu_system(&root_menu);
#endif

// Shared state
ILED* rgbLed;
static volatile uint32_t pulse_count_left_A = 0; // increments from ISR
static volatile uint32_t pulse_count_left_B = 0; // increments from ISR
static volatile uint32_t pulse_count_left_C = 0; // increments from ISR
static volatile uint32_t pulse_count_right_A = 0; // increments from ISR
static volatile uint32_t pulse_count_right_B = 0; // increments from ISR
static volatile uint32_t pulse_count_right_C = 0; // increments from ISR
static volatile uint32_t pulses_left_A = 0;
static volatile uint32_t pulses_left_B = 0;
static volatile uint32_t pulses_left_C = 0;
static volatile uint32_t pulses_right_A = 0;
static volatile uint32_t pulses_right_B = 0;
static volatile uint32_t pulses_right_C = 0;
static uint32_t rev_count_left = 0;
static uint32_t rev_count_right = 0;
static uint32_t rpm_left = 0;
static uint32_t rpm_right = 0;
static uint32_t avg_rpm = 0;
static float speed_mph = 0.0f;
static double trip_distance_miles;
static float mAH_consumption = 0.0f;
static int state_of_charge = 0;
static uint32_t pack_voltage_mv;
static uint32_t last_integration_time_ms = 0;
static volatile bool esc_powered = false;
static volatile bool mode_button_pressed = false; // active-low Mode button on GPIO0
static volatile bool mode_toggle_pending = false;
static volatile int64_t mode_button_press_time_us = 0;
static SemaphoreHandle_t pulse_mutex = NULL;

// ADC handle for esp-idf v5 oneshot API
static adc_oneshot_unit_handle_t adc_handle = NULL;
static SemaphoreHandle_t gatt_tx_mutex = NULL;

// I2C and Temperature sensor
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static TMP117* tmp117_sensor = NULL;
static float device_temperature_c = 0.0f;

// Accessory current globals (12V and 5V rails)
static float accessory_current_12v_ma = 0.0f;
static float accessory_current_5v_ma = 0.0f;
static float accessory_watts = 0.0f;
static volatile float battery_current_ma = 0.0f;
static uint8_t headlight_duty_cycle = 128;
static uint8_t brake_light_duty_cycle = 0;

struct SkateInfoState {
    uint32_t pack_voltage_mv = 0;
    uint32_t pack_current_ma = 0;
    uint8_t soc = 0;
    int32_t pack_energy_mah = 0;
    int8_t speed_mph = 0;
    int8_t board_temperature_c = 0;
};

SkateInfoState skateInfoState;
static ICANController *canBus = NULL;

// Smoothed ADC raw averages provided by the ADC sampler task
// Raw averaged ADC values stored as integer decimals (atomic uint32)
static uint32_t adc_avg_12v_bits = 0;
static uint32_t adc_avg_5v_bits = 0;
static uint32_t adc_avg_batt_bits = 0;
static uint32_t adc_avg_curr_hs_bits = 0;
static uint32_t adc_avg_curr_ref_bits = 0;

// ADC sampler task: alternately sample accessory channels 10x each tick
static void adc_sampler_task(void *arg)
{
    (void)arg;
    // 0=12V,1=5V,2=BattV,3=Ref,4=CurrHS
    int idx = 0;

    while (1) {
        if (adc_handle == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int channel = -1;
        switch (idx) {
            case 0: channel = ADC_12V_CURR_CHANNEL; break;
            case 1: channel = ADC_5V_CURR_CHANNEL; break;
            case 2: channel = ADC_BATT_CHANNEL; break;
            case 3: channel = ADC_CURR_REF_CHANNEL; break;
            case 4: channel = ADC_CURR_HS_CHANNEL; break;
        }

        if (channel >= 0) {
            int tmp = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, (adc_channel_t)channel, &tmp)); // dummy
            esp_rom_delay_us(10);
            int sum = 0;
            for (int i = 0; i < 10; ++i) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, (adc_channel_t)channel, &tmp));
                sum += tmp;
            }
            uint32_t avg = (uint32_t)(sum / 10);
            switch (idx) {
                case 0: __atomic_store_n(&adc_avg_12v_bits, avg, __ATOMIC_RELAXED); break;
                case 1: __atomic_store_n(&adc_avg_5v_bits, avg, __ATOMIC_RELAXED); break;
                case 2: __atomic_store_n(&adc_avg_batt_bits, avg, __ATOMIC_RELAXED); break;
                case 3: __atomic_store_n(&adc_avg_curr_ref_bits, avg, __ATOMIC_RELAXED); break;
                case 4: __atomic_store_n(&adc_avg_curr_hs_bits, avg, __ATOMIC_RELAXED); break;
            }
        }

        idx = (idx + 1) % 5;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Battery configuration
IntegratedBMS *bms = NULL;
static const uint32_t pack_idle_current_ma = 200;

INVSStorage *nvs = NULL;

static void board_configuration_task(void *arg)
{
    (void)arg;
    TickType_t last_save_tick = xTaskGetTickCount();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        const TickType_t now = xTaskGetTickCount();
        const bool periodic_save_due =
            (now - last_save_tick) >= pdMS_TO_TICKS(120000);
        bool stale_save_claimed = false;
        if (nvs != NULL) {
            stale_save_claimed =
                __atomic_exchange_n(&board_configuration_stale, false, __ATOMIC_ACQ_REL);
        }

        if (nvs != NULL && (stale_save_claimed || periodic_save_due)) {
            esp_err_t err = board_configuration_save(*nvs, boardConfiguration);
            if (err == ESP_OK) {
                last_save_tick = now;
            } else {
                __atomic_store_n(&board_configuration_stale, true, __ATOMIC_RELEASE);
                ESP_LOGE(TAG, "Failed to save board configuration: %s", esp_err_to_name(err));
            }
        }
    }
}

/* BLE RX message queue structure and handle */
typedef struct {
    uint16_t len;
    uint8_t data[512];
} ble_rx_message_t;

static QueueHandle_t ble_rx_queue = NULL;
static const uint16_t BLE_RX_QUEUE_SIZE = 10;

static uint32_t readPackVoltageMillivolts(bool useBulkReader){
    const float voltDividerRatio = 25.325f;
    // Prefer sampler-provided averaged raw value if available
    
    if(useBulkReader){
        uint32_t bits = __atomic_load_n(&adc_avg_batt_bits, __ATOMIC_RELAXED);
        if (bits != 0) {
            float raw_avg = (float)bits;
            float battVoltageCorr = raw_avg * voltDividerRatio;
            return (uint32_t)battVoltageCorr;
        }
        return 0;
    }
    else{
        // Fallback: direct ADC read
        int raw_batt = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_BATT_CHANNEL, &raw_batt));
        esp_rom_delay_us(10);
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_BATT_CHANNEL, &raw_batt));
        float battVoltageCorr = ((float)raw_batt * voltDividerRatio);
        return (uint32_t)battVoltageCorr;
    }
}

// ISR for hall pulse
static void IRAM_ATTR hall_isr_handler(void *arg)
{
    // keep ISR short
    int gpio_num = (int)(intptr_t)arg;

    // Compare which pin triggered the ISR and handle accordingly.
    // Only do minimal, ISR-safe work here.
    switch (gpio_num)
    {
    case HALL_LEFT_A_GPIO:
        pulse_count_left_A = pulse_count_left_A + 1;
        break;
    case HALL_LEFT_B_GPIO:
        pulse_count_left_B = pulse_count_left_B + 1;
        break;
    case HALL_LEFT_C_GPIO:
        pulse_count_left_C = pulse_count_left_C + 1;
        break;
    case HALL_RIGHT_A_GPIO:
        pulse_count_right_A = pulse_count_right_A + 1;
        break;
    case HALL_RIGHT_B_GPIO:
        pulse_count_right_B = pulse_count_right_B + 1;
        break;
    case HALL_RIGHT_C_GPIO:
        pulse_count_right_C = pulse_count_right_C + 1;
        break;
    default:
        break;
    }
}

// Apply a pending mode change from task context.
static void toggle_uart_mode()
{
    boardConfiguration.uartConfig = (boardConfiguration.uartConfig == STANDARD_MENU)
                                         ? STANDARD_ESPLOG
                                         : STANDARD_MENU;
    __atomic_store_n(&board_configuration_stale, true, __ATOMIC_RELEASE);

#if CONFIG_BT_NIMBLE_ENABLED
    if (boardConfiguration.uartConfig == STANDARD_MENU) {
        if (!usb_console.begin()) {
            ESP_LOGE(TAG, "Failed to initialize USB serial console");
        }
        esp_log_level_set("*", ESP_LOG_NONE);
    } else {
        esp_log_level_set("*", ESP_LOG_INFO);
        ESP_LOGI(TAG, "USB Serial/JTAG reserved for OpenOCD debug console");
    }
#endif
}

static void IRAM_ATTR mode_button_isr_handler(void *arg)
{
    static volatile int64_t last_isr_time_us = 0;
    const int64_t now_us = esp_timer_get_time();
    if (now_us - last_isr_time_us < 10000) {
        return;
    }
    last_isr_time_us = now_us;

    const bool pressed = (gpio_get_level(MODE_BUTTON_GPIO) == 0); // active-low button
    mode_button_pressed = pressed;

    if (pressed) {
        mode_button_press_time_us = now_us;
    } else {
        // A release within one second is a short press; defer the mode change.
        if (now_us - mode_button_press_time_us > 3000000) {
            __atomic_store_n(&mode_toggle_pending, true, __ATOMIC_RELEASE);
        }
    }
}

static void rpm_task(void *arg)
{
    const BoardConfiguration* configuration = static_cast<const BoardConfiguration*>(arg);
    if (configuration == nullptr) {
        vTaskDelete(nullptr);
        return;
    }

    static int64_t last_time_us = esp_timer_get_time();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // every second

        int64_t now_us = esp_timer_get_time();
        double dt_s = (double)(now_us - last_time_us) / 1000000.0;
        if (dt_s <= 0.0) dt_s = 0.001; // guard against zero
        last_time_us = now_us;

        // atomically get & clear pulse_count (ISR increments pulse_count)
        pulses_left_A = __atomic_exchange_n(&pulse_count_left_A, 0U, __ATOMIC_ACQ_REL);
        pulses_left_B = __atomic_exchange_n(&pulse_count_left_B, 0U, __ATOMIC_ACQ_REL);
        pulses_left_C = __atomic_exchange_n(&pulse_count_left_C, 0U, __ATOMIC_ACQ_REL);
        pulses_right_A = __atomic_exchange_n(&pulse_count_right_A, 0U, __ATOMIC_ACQ_REL);
        pulses_right_B = __atomic_exchange_n(&pulse_count_right_B, 0U, __ATOMIC_ACQ_REL);
        pulses_right_C = __atomic_exchange_n(&pulse_count_right_C, 0U, __ATOMIC_ACQ_REL);

        // Compute revolutions during the interval
        const uint8_t motor_enable_mask =
        configuration->motorSenseConfiguration.motorEnableMask;
        uint32_t pulses_left_total = 0;
        uint32_t pulses_right_total = 0;

        if ((motor_enable_mask & (1U << 0)) != 0) {
            pulses_left_total += pulses_left_A;
        }
        if ((motor_enable_mask & (1U << 1)) != 0) {
            pulses_left_total += pulses_left_B;
        }
        if ((motor_enable_mask & (1U << 2)) != 0) {
            pulses_left_total += pulses_left_C;
        }
        if ((motor_enable_mask & (1U << 3)) != 0) {
            pulses_right_total += pulses_right_A;
        }
        if ((motor_enable_mask & (1U << 4)) != 0) {
            pulses_right_total += pulses_right_B;
        }
        if ((motor_enable_mask & (1U << 5)) != 0) {
            pulses_right_total += pulses_right_C;
        }
        const MotorSenseConfiguration& motor_configuration =
            configuration->motorSenseConfiguration;
        double revs_left = 0.0;
        double revs_right = 0.0;
        if (motor_configuration.motor1PulsesPerRevolution > 0) {
            revs_left = (double)pulses_left_total /
                        (double)motor_configuration.motor1PulsesPerRevolution;
        }
        if (motor_configuration.motor2PulsesPerRevolution > 0) {
            revs_right = (double)pulses_right_total /
                         (double)motor_configuration.motor2PulsesPerRevolution;
        }

        // RPS and RPM
        double rps_left = revs_left / dt_s;
        double rps_right = revs_right / dt_s;
        rpm_left = (uint32_t)(rps_left * 60.0);
        rpm_right = (uint32_t)(rps_right * 60.0);

        // Update revolution counters (integral count)
        rev_count_left += (uint32_t)revs_left;
        rev_count_right += (uint32_t)revs_right;

        // Use both wheels when available, otherwise use the active wheel.
        if (rpm_left > 0 && rpm_right > 0) {
            avg_rpm = (rpm_left + rpm_right) / 2;
        } else if (rpm_left > 0) {
            avg_rpm = rpm_left;
        } else {
            avg_rpm = rpm_right;
        }

        // Compute speed (assume wheelSizeMillimeterTenths is diameter in 0.1 mm units -> circumference = pi * d)
        double wheel_circ_m = ((double)configuration->wheelSizeMillimeterTenths / 10000.0) * M_PI;
        double avg_rps = avg_rpm / 60.0;
        double speed_m_per_s = avg_rps * wheel_circ_m;
        // Convert m/s to MPH (1 m/s = 2.2369362920544 mph)
        speed_mph = speed_m_per_s * 2.2369362920544;

        // Integrate distance directly from meters to miles.
        trip_distance_miles += (speed_m_per_s * dt_s) / 1609.344;
    }
}

static void sensor_task(void *arg)
{
    (void)arg;

    last_integration_time_ms = esp_log_timestamp();

    while (1) {
        //uint32_t now = esp_log_timestamp();

        // Use averaged raw ADC values for current/reference provided by adc_sampler_task
        float raw_curr = 0.0f;
        float ref = 0.0f;
        uint32_t hs_adc = __atomic_load_n(&adc_avg_curr_hs_bits, __ATOMIC_RELAXED);
        uint32_t ref_adc = __atomic_load_n(&adc_avg_curr_ref_bits, __ATOMIC_RELAXED);
        raw_curr = (float)hs_adc;
        ref = (float)ref_adc;

        // Use averaged raw ADC values produced by adc_sampler_task
        float raw_12v_avg; 
        float raw_5v_avg;
        uint32_t acc_12vs_adc = __atomic_load_n(&adc_avg_12v_bits, __ATOMIC_RELAXED);
        uint32_t acc_5vs_adc = __atomic_load_n(&adc_avg_5v_bits, __ATOMIC_RELAXED);
        raw_12v_avg = (float)acc_12vs_adc;
        raw_5v_avg = (float)acc_5vs_adc;
        
        const float shuntRatio = 42.0f;
        const float adc_cal = 5.0f;

        float batteryVoltage = (float)readPackVoltageMillivolts(true);

        // Battery current calculation for bidirectional amp
        // We'll average the ADC reference reading (`ref`) and the last 10
        // instantaneous current samples separately. Use the averaged ref
        // when computing instantaneous current, then smooth current by
        // a 10-sample circular buffer.

        // --- Reference averaging (10-sample moving average on raw ADC units)
        enum { REF_BUF_SIZE = 10 };
        static float ref_buf[REF_BUF_SIZE] = {0};
        static int ref_idx = 0;
        static int ref_count = 0;
        ref_buf[ref_idx] = (float)ref;
        ref_idx = (ref_idx + 1) % REF_BUF_SIZE;
        if (ref_count < REF_BUF_SIZE) ref_count++;
        float ref_avg = 0.0f;
        for (int i = 0; i < ref_count; ++i) ref_avg += ref_buf[i];
        if (ref_count > 0) ref_avg /= (float)ref_count;

        // Compute instantaneous current using averaged reference
        float battCurrentRaw = (float)raw_curr;
        float battCurrentmA_instant = ((battCurrentRaw - ref_avg + adc_cal) * 1000.0f / shuntRatio);

        // 10-sample moving average buffer for current (static so it persists across loop iterations)
        enum { CURR_BUF_SIZE = 10 };
        static float curr_buf[CURR_BUF_SIZE] = {0};
        static int curr_idx = 0;
        static int curr_count = 0;

        curr_buf[curr_idx] = battCurrentmA_instant;
        curr_idx = (curr_idx + 1) % CURR_BUF_SIZE;
        if (curr_count < CURR_BUF_SIZE) curr_count++;
        
        // --- Accessory current averaging (10-sample moving average each)
        enum { ACC_CURR_BUF_SIZE = 10 };
        static float acc12_buf[ACC_CURR_BUF_SIZE] = {0};
        static int acc12_idx = 0;
        static int acc12_count = 0;
        static float acc5_buf[ACC_CURR_BUF_SIZE] = {0};
        static int acc5_idx = 0;
        static int acc5_count = 0;

        // Convert averaged raw ADC to mA for accessories (use sampler's 10-sample average)
        const PowerSystemConfiguration& power_configuration =
            boardConfiguration.powerSystemConfiguration;
        float acc12_mA_instant = power_configuration.adc_counts_per_amp_12V == 0
                                     ? 0.0f
                                     : (raw_12v_avg * 1000.0f /
                                        static_cast<float>(power_configuration.adc_counts_per_amp_12V));
        float acc5_mA_instant = power_configuration.adc_counts_per_amp_5V == 0
                                    ? 0.0f
                                    : (raw_5v_avg * 1000.0f /
                                       static_cast<float>(power_configuration.adc_counts_per_amp_5V));

        acc12_buf[acc12_idx] = acc12_mA_instant;
        acc12_idx = (acc12_idx + 1) % ACC_CURR_BUF_SIZE;
        if (acc12_count < ACC_CURR_BUF_SIZE) acc12_count++;

        acc5_buf[acc5_idx] = acc5_mA_instant;
        acc5_idx = (acc5_idx + 1) % ACC_CURR_BUF_SIZE;
        if (acc5_count < ACC_CURR_BUF_SIZE) acc5_count++;

        accessory_current_12v_ma = 0.0f;
        for (int i = 0; i < acc12_count; ++i) accessory_current_12v_ma += acc12_buf[i];
        if (acc12_count > 0) accessory_current_12v_ma /= (float)acc12_count;

        accessory_current_5v_ma = 0.0f;
        for (int i = 0; i < acc5_count; ++i) accessory_current_5v_ma += acc5_buf[i];
        if (acc5_count > 0) accessory_current_5v_ma /= (float)acc5_count;

        // Compute accessory power (watts) = sum( I(A) * V )
        accessory_watts =
            (accessory_current_12v_ma / 1000.0f) *
                (static_cast<float>(power_configuration.voltage_12V_millivolts) / 1000.0f) +
            (accessory_current_5v_ma / 1000.0f) *
                (static_cast<float>(power_configuration.voltage_5V_millivolts) / 1000.0f);

        float battCurrentmA = 0.0f;
        for (int i = 0; i < curr_count; ++i) battCurrentmA += curr_buf[i];
        if (curr_count > 0) battCurrentmA /= (float)curr_count;
        battery_current_ma = battCurrentmA;

        if(bms != NULL){
            bms->update(-(int32_t)battCurrentmA, batteryVoltage, 25.0f);
            mAH_consumption = (float)bms->getEnergyConsumed();
            state_of_charge = (int)bms->getSoC();
        }

        ESP_LOGI(TAG, "5V_raw=%.1f 5V_A=%.1f 12V_raw=%.1f 12V_A=%.1f", raw_5v_avg, accessory_current_5v_ma, raw_12v_avg, accessory_current_12v_ma);

        //ESP_LOGI(TAG, "battRaw=%d battCorr=%.0f mA=%.1f mAH=%.3f", raw_batt, battVoltageCorr, battCurrentmA, mAH_consumption);

        /* Store current sensor values for telemetry task */
        /* (These are static globals used by ble_telemetry_task) */

        vTaskDelay(pdMS_TO_TICKS(RPM_POLL_MS));
    }
}

static void tmp117_sensor_task(void *arg)
{
    (void)arg;

    while (1) {
        if (tmp117_sensor != NULL) {
            esp_err_t ret = tmp117_sensor->readTemperature(&device_temperature_c);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to read TMP117 temperature: %s", esp_err_to_name(ret));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(RPM_POLL_MS));
    }
}

static void write_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = static_cast<uint8_t>(value);
    destination[1] = static_cast<uint8_t>(value >> 8);
    destination[2] = static_cast<uint8_t>(value >> 16);
    destination[3] = static_cast<uint8_t>(value >> 24);
}

static int8_t to_can_int8(float value)
{
    if (value > 127.0f) return 127;
    if (value < -128.0f) return -128;
    return static_cast<int8_t>(value);
}

static void canbus_task(void *arg)
{
    (void)arg;

    while (canBus == NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    while (1) {
        if (bms != NULL) {
            skateInfoState.pack_voltage_mv = bms->getPackVoltageMillivolts();
            skateInfoState.pack_current_ma = bms->getPackCurrentMilliamps();
            skateInfoState.soc = static_cast<uint8_t>(bms->getSoC());
            skateInfoState.pack_energy_mah = bms->getEnergyConsumed();
        }

        skateInfoState.speed_mph = to_can_int8(speed_mph);
        skateInfoState.board_temperature_c = to_can_int8(device_temperature_c);

        CANMessage power_message;
        power_message.addr = 0x200;
        write_u32_le(&power_message.bytes[0], skateInfoState.pack_voltage_mv);
        write_u32_le(&power_message.bytes[4], skateInfoState.pack_current_ma);

        CANMessage status_message;
        status_message.addr = 0x201;
        status_message.bytes[0] = skateInfoState.soc;
        write_u32_le(&status_message.bytes[1],
                     static_cast<uint32_t>(skateInfoState.pack_energy_mah));
        status_message.bytes[5] = static_cast<uint8_t>(skateInfoState.speed_mph);
        status_message.bytes[6] = static_cast<uint8_t>(skateInfoState.board_temperature_c);
        status_message.bytes[7] = 0;

        if (canBus->send(power_message) != ICANController::CANResult::Success ||
            canBus->send(status_message) != ICANController::CANResult::Success) {
            ESP_LOGW(TAG, "Failed to transmit CAN telemetry");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

#if CONFIG_BT_NIMBLE_ENABLED

static bool hardware_feature_is_enabled(uint8_t index)
{
    switch (index) {
        case 0: return boardConfiguration.hardwareFeatureEnable.feature0;
        case 1: return boardConfiguration.hardwareFeatureEnable.feature1;
        case 2: return boardConfiguration.hardwareFeatureEnable.feature2;
        case 3: return boardConfiguration.hardwareFeatureEnable.feature3;
        case 4: return boardConfiguration.hardwareFeatureEnable.feature4;
        case 5: return boardConfiguration.hardwareFeatureEnable.feature5;
        case 6: return boardConfiguration.hardwareFeatureEnable.feature6;
        case 7: return boardConfiguration.hardwareFeatureEnable.feature7;
        case 8: return boardConfiguration.hardwareFeatureEnable.feature8;
        case 9: return boardConfiguration.hardwareFeatureEnable.feature9;
        case 10: return boardConfiguration.hardwareFeatureEnable.feature10;
        case 11: return boardConfiguration.hardwareFeatureEnable.feature11;
        case 12: return boardConfiguration.hardwareFeatureEnable.feature12;
        case 13: return boardConfiguration.hardwareFeatureEnable.feature13;
        case 14: return boardConfiguration.hardwareFeatureEnable.feature14;
        case 15: return boardConfiguration.hardwareFeatureEnable.tmp117SensorEnabled;
        case 16: return boardConfiguration.hardwareFeatureEnable.canbusEnabled;
        case 17: return boardConfiguration.hardwareFeatureEnable.useMCP2515;
        case 18: return boardConfiguration.hardwareFeatureEnable.motorSenseEnable;
        default: return false;
    }
}

static void toggle_hardware_feature(uint8_t index)
{
    switch (index) {
        case 0: boardConfiguration.hardwareFeatureEnable.feature0 = !boardConfiguration.hardwareFeatureEnable.feature0; break;
        case 1: boardConfiguration.hardwareFeatureEnable.feature1 = !boardConfiguration.hardwareFeatureEnable.feature1; break;
        case 2: boardConfiguration.hardwareFeatureEnable.feature2 = !boardConfiguration.hardwareFeatureEnable.feature2; break;
        case 3: boardConfiguration.hardwareFeatureEnable.feature3 = !boardConfiguration.hardwareFeatureEnable.feature3; break;
        case 4: boardConfiguration.hardwareFeatureEnable.feature4 = !boardConfiguration.hardwareFeatureEnable.feature4; break;
        case 5: boardConfiguration.hardwareFeatureEnable.feature5 = !boardConfiguration.hardwareFeatureEnable.feature5; break;
        case 6: boardConfiguration.hardwareFeatureEnable.feature6 = !boardConfiguration.hardwareFeatureEnable.feature6; break;
        case 7: boardConfiguration.hardwareFeatureEnable.feature7 = !boardConfiguration.hardwareFeatureEnable.feature7; break;
        case 8: boardConfiguration.hardwareFeatureEnable.feature8 = !boardConfiguration.hardwareFeatureEnable.feature8; break;
        case 9: boardConfiguration.hardwareFeatureEnable.feature9 = !boardConfiguration.hardwareFeatureEnable.feature9; break;
        case 10: boardConfiguration.hardwareFeatureEnable.feature10 = !boardConfiguration.hardwareFeatureEnable.feature10; break;
        case 11: boardConfiguration.hardwareFeatureEnable.feature11 = !boardConfiguration.hardwareFeatureEnable.feature11; break;
        case 12: boardConfiguration.hardwareFeatureEnable.feature12 = !boardConfiguration.hardwareFeatureEnable.feature12; break;
        case 13: boardConfiguration.hardwareFeatureEnable.feature13 = !boardConfiguration.hardwareFeatureEnable.feature13; break;
        case 14: boardConfiguration.hardwareFeatureEnable.feature14 = !boardConfiguration.hardwareFeatureEnable.feature14; break;
        case 15: boardConfiguration.hardwareFeatureEnable.tmp117SensorEnabled = !boardConfiguration.hardwareFeatureEnable.tmp117SensorEnabled; break;
        case 16: boardConfiguration.hardwareFeatureEnable.canbusEnabled = !boardConfiguration.hardwareFeatureEnable.canbusEnabled; break;
        case 17: boardConfiguration.hardwareFeatureEnable.useMCP2515 = !boardConfiguration.hardwareFeatureEnable.useMCP2515; break;
        case 18: boardConfiguration.hardwareFeatureEnable.motorSenseEnable = !boardConfiguration.hardwareFeatureEnable.motorSenseEnable; break;
        default: break;
    }
}

static void set_hardware_feature(uint8_t index, bool enabled)
{
    switch (index) {
        case 15: boardConfiguration.hardwareFeatureEnable.tmp117SensorEnabled = enabled; break;
        case 16: boardConfiguration.hardwareFeatureEnable.canbusEnabled = enabled; break;
        case 17: boardConfiguration.hardwareFeatureEnable.useMCP2515 = enabled; break;
        case 18: boardConfiguration.hardwareFeatureEnable.motorSenseEnable = enabled; break;
        default: break;
    }
}

static void add_hardware_feature_command(MenuItem& menu, std::string name, uint8_t index)
{
    menu.addCommand(name, nullptr, [name, index](std::string* command) {
        const std::string argument = command->size() > name.size() &&
                                             command->compare(0, name.size(), name) == 0 &&
                                             (*command)[name.size()] == ' '
                                         ? command->substr(name.size() + 1)
                                         : "";
        bool enabled = hardware_feature_is_enabled(index);
        if (*command == name || argument == "toggle") {
            toggle_hardware_feature(index);
            enabled = hardware_feature_is_enabled(index);
        } else if (argument == "true" || argument == "on") {
            set_hardware_feature(index, true);
            enabled = true;
        } else if (argument == "false" || argument == "off") {
            set_hardware_feature(index, false);
            enabled = false;
        } else {
            console_combiner.writeLine("Usage: " + name + " toggle|true|false (current: " +
                                       (enabled ? "enabled" : "disabled") + ")");
            return;
        }

        __atomic_store_n(&board_configuration_stale, true, __ATOMIC_RELEASE);
        console_combiner.writeLine(name + " set to " + (enabled ? "enabled" : "disabled"));
    });
}

static void add_hardware_feature_alias(MenuItem& menu, std::string name, uint8_t index)
{
    add_hardware_feature_command(menu, name, index);
}

static void hardware_features_menu_on_enter()
{
    console_combiner.writeLine("Hardware feature toggles:");
    for (uint8_t index = 0; index < 15; ++index) {
        console_combiner.writeLine("  feature" + std::to_string(index) + "=" +
                                   (hardware_feature_is_enabled(index) ? "enabled" : "disabled"));
    }
    console_combiner.writeLine("  tmp117SensorEnabled=" +
                               std::string(hardware_feature_is_enabled(15) ? "enabled" : "disabled"));
    console_combiner.writeLine("  canbusEnabled=" +
                               std::string(hardware_feature_is_enabled(16) ? "enabled" : "disabled"));
    console_combiner.writeLine("  useMCP2515=" +
                               std::string(hardware_feature_is_enabled(17) ? "enabled" : "disabled"));
    console_combiner.writeLine("  motorSenseEnable=" +
                               std::string(hardware_feature_is_enabled(18) ? "enabled" : "disabled"));
}

static void initialize_menu_system()
{
    static MenuItem hardware_features_menu("features", hardware_features_menu_on_enter);
    for (uint8_t index = 0; index < 15; ++index) {
        add_hardware_feature_command(hardware_features_menu,
                                     (std::string("feature") + std::to_string(index)).c_str(),
                                     index);
    }
    add_hardware_feature_command(hardware_features_menu, "tmp117SensorEnabled", 15);
    add_hardware_feature_command(hardware_features_menu, "canbusEnabled", 16);
    add_hardware_feature_alias(hardware_features_menu, "canbusEnable", 16);
    add_hardware_feature_command(hardware_features_menu, "useMCP2515", 17);
    add_hardware_feature_command(hardware_features_menu, "motorSenseEnable", 18);
    root_menu.addCommand("features", &hardware_features_menu, nullptr);
    static MenuItem power_menu("power");
    power_menu.addCommand("headlight", nullptr, [](std::string* command) {
        const size_t name_length = strlen("headlight");
        if (*command == "headlight") {
            console_combiner.writeLine("Usage: headlight <duty 0-255> (current: " +
                                       std::to_string(headlight_duty_cycle) + ")");
            return;
        }

        char* end = nullptr;
        const unsigned long value = strtoul(command->c_str() + name_length, &end, 0);
        if (command->compare(0, name_length, "headlight") == 0 &&
            command->size() > name_length && (*command)[name_length] == ' ' &&
            end != command->c_str() + name_length && *end == '\0' && value <= UINT8_MAX) {
            headlight_duty_cycle = static_cast<uint8_t>(value);
            console_combiner.writeLine("Headlight duty set to " + std::to_string(value));
        }
    });
    power_menu.addCommand("brake", nullptr, [](std::string* command) {
        const size_t name_length = strlen("brake");
        if (*command == "brake") {
            console_combiner.writeLine("Usage: brake <duty 0-255> (current: " +
                                       std::to_string(brake_light_duty_cycle) + ")");
            return;
        }

        char* end = nullptr;
        const unsigned long value = strtoul(command->c_str() + name_length, &end, 0);
        if (command->compare(0, name_length, "brake") == 0 &&
            command->size() > name_length && (*command)[name_length] == ' ' &&
            end != command->c_str() + name_length && *end == '\0' && value <= UINT8_MAX) {
            brake_light_duty_cycle = static_cast<uint8_t>(value);
            console_combiner.writeLine("Brake light duty set to " + std::to_string(value));
        }
    });
    root_menu.addCommand("power", &power_menu, nullptr);
    board_configuration_menu_initialize();
    root_menu.addCommand("board", &board_configuration_menu, nullptr);
    root_menu.addCommand("help", nullptr, [](std::string*) {
        console_combiner.writeLine("Commands: help, status, board, power, features, diagnostics");
    });
    root_menu.addCommand("status", nullptr, [](std::string* command) {
        console_combiner.writeLine(std::string("Status action received: ") + *command);
    });
    root_menu.addCommand("diagnostics", &diagnostics_menu, nullptr);

    diagnostics_menu.addCommand("hello", nullptr, [](std::string*) {
        console_combiner.writeLine("Hello from the nested diagnostics menu");
    });
    diagnostics_menu.addCommand("echo", nullptr, [](std::string* command) {
        console_combiner.writeLine(std::string("Echo: ") + *command);
    });
    diagnostics_menu.addCommand("help", nullptr, [](std::string*) {
        console_combiner.writeLine("Diagnostics commands: hello, echo, back, root");
    });
    if (canalyzer != nullptr) {
        canalyzer->canalyzerMenu()->setOnEnter(canalyzer_menu_on_enter);
        diagnostics_menu.addCommand("canalyzer", canalyzer->canalyzerMenu(), nullptr);
    }
}

static void menu_task(void *arg)
{
    (void)arg;

    while (1) {
        std::vector<std::string> console_lines;
        if (console_combiner.processLines(console_lines)) {
            for (const std::string& line : console_lines) {
                if (!menu_system.processCommand(line)) {
                    console_combiner.writeLine(std::string("Unknown menu command: ") + line);
                }
            }
        }

        if (canalyzer != nullptr) {
            canalyzer->receiveCANFrames();
            canalyzer->processCANFrames();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#endif /* CONFIG_BT_NIMBLE_ENABLED */

/* Handler functions for BLE control commands */

/**
 * Handler for mode control command
 * @param mode: string mode value (e.g., "cruise", "sport", etc.)
 */
static void modeHandler(const char *mode)
{
    ESP_LOGI(TAG, "Mode command received: %s", mode);
    // TODO: Update global mode state variable
    // Example: strncpy(current_mode, mode, sizeof(current_mode) - 1);
}

/**
 * Handler for headlight control command
 * @param brightness: 0-255 brightness level
 */
static void headlightHandler(uint8_t brightness)
{
    ESP_LOGI(TAG, "Headlight command received: brightness=%d", brightness);
    // TODO: Update global headlight state variable
    // Example: headlight_brightness = brightness;
}

/**
 * Handler for taillight control command
 * @param brightness: 0-255 brightness level
 */
static void taillightHandler(uint8_t brightness)
{
    ESP_LOGI(TAG, "Taillight command received: brightness=%d", brightness);
    // TODO: Update global taillight state variable
    // Example: taillight_brightness = brightness;
}

static void handleResetHistoricalCapacity()
{
    if (bms == nullptr) {
        ESP_LOGW(TAG, "BMS pointer is null, cannot reset historical capacity");
        return;
    }

    ESP_LOGI(TAG, "Resetting BMS historical capacity");
    bms->resetHistoricalCapacity();
}

static void handleResetEnergyConsumption()
{
    if (bms == nullptr) {
        ESP_LOGW(TAG, "BMS pointer is null, cannot reset energy consumption");
        return;
    }

    ESP_LOGI(TAG, "Resetting BMS energy consumption using OCV, packV=%lu mV",
             pack_voltage_mv);

    bms->resetEnergyConsumptionFromOpenCircuitVoltage(pack_voltage_mv);
}



/**
 * Process commands from the BLE RX queue
 * Parses JSON commands using cJSON and dispatches to appropriate handlers
 * Supports: "hdl" (headlight, 0-255), "tll" (taillight, 0-255), "mode" (string)
 */
static void ble_process_command_queue(void)
{
    ble_rx_message_t msg;

    /* Non-blocking check for queued messages */
    if (xQueueReceive(ble_rx_queue, &msg, 0) == pdPASS) {
        /* Null-terminate the message data */
        if (msg.len < sizeof(msg.data)) {
            msg.data[msg.len] = '\0';
        } else {
            msg.data[sizeof(msg.data) - 1] = '\0';
        }

        const char *json_str = (const char *)msg.data;
        ESP_LOGI(TAG, "Processing command: %s", json_str);

        /* Parse JSON using cJSON */
        cJSON *root = cJSON_Parse(json_str);
        if (root == NULL) {
            ESP_LOGW(TAG, "Failed to parse JSON: %s", json_str);
            return;
        }

        /* Handle headlight (hdl) command */
        cJSON *hdl_item = cJSON_GetObjectItem(root, "hdl");
        if (hdl_item != NULL && cJSON_IsNumber(hdl_item)) {
            int hdl_value = hdl_item->valueint;
            if (hdl_value >= 0 && hdl_value <= 255) {
                headlightHandler((uint8_t)hdl_value);
            } else {
                ESP_LOGW(TAG, "Invalid headlight value: %d", hdl_value);
            }
        }

        /* Handle taillight (tll) command */
        cJSON *tll_item = cJSON_GetObjectItem(root, "tll");
        if (tll_item != NULL && cJSON_IsNumber(tll_item)) {
            int tll_value = tll_item->valueint;
            if (tll_value >= 0 && tll_value <= 255) {
                taillightHandler((uint8_t)tll_value);
            } else {
                ESP_LOGW(TAG, "Invalid taillight value: %d", tll_value);
            }
        }

        /* Handle mode command */
        cJSON *mode_item = cJSON_GetObjectItem(root, "mode");
        if (mode_item != NULL && cJSON_IsString(mode_item)) {
            modeHandler(mode_item->valuestring);
        }

        /* Handle BMS reset historical capacity (bms_rhc) */
        cJSON *rhc_item = cJSON_GetObjectItem(root, "bms_rhc");
        if (rhc_item != NULL && cJSON_IsBool(rhc_item)) {
            if (cJSON_IsTrue(rhc_item)) {
                handleResetHistoricalCapacity();
            }
        }

        /* Handle BMS reset energy consumption (bms_rec) */
        cJSON *rec_item = cJSON_GetObjectItem(root, "bms_rec");
        if (rec_item != NULL && cJSON_IsBool(rec_item)) {
            if (cJSON_IsTrue(rec_item)) {
                handleResetEnergyConsumption();
            }
        }


        /* Clean up cJSON object */
        cJSON_Delete(root);
    }
}

static void power_control_task(void *arg){
    // Instantiate the RGB LED (example pins: 6, 7, 8)
    rgbLed = new ESP32LED(8, 47, 48);

    RGBColor color{
        .r = 0,
        .g = 255,
        .b = 0
    };

    rgbLed->setState(LEDPattern::Breathe, color);

    static TickType_t last_esc_on_time = xTaskGetTickCount();

    while(1){
        /* Process any pending BLE control commands */
        ble_process_command_queue();

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, HEADLIGHT_LEDC_CHANNEL,
                                      headlight_duty_cycle));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, HEADLIGHT_LEDC_CHANNEL));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, BRAKELIGHT_LEDC_CHANNEL,
                                      brake_light_duty_cycle));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, BRAKELIGHT_LEDC_CHANNEL));

        if(boardConfiguration.hardwareFeatureEnable.motorSenseEnable)
        {
            esc_powered = gpio_get_level(ESC_PWR_SENSE_GPIO);
        }
        else
        {
            esc_powered = fabsf(battery_current_ma) > 1000.0f;
        }


        //esc_powered = (rpm_left > 0) || (rpm_right > 0);

        // Menu/debug UART mode gets a lighter shade of the same hue
        const uint8_t base_r = (boardConfiguration.uartConfig == STANDARD_MENU) ? 64 : 0;

        if(esc_powered){
            color.b = 255;
            color.g = 255;
            color.r = base_r;

            last_esc_on_time = xTaskGetTickCount();
        }
        else{
            color.b = 0;
            color.g = 255;
            color.r = base_r;

            if(pdTICKS_TO_MS(xTaskGetTickCount() - last_esc_on_time) > IDLE_POWEROFF_MS){
                color.b = 0;
                color.g = 255;
                color.r = 255;

                gpio_set_level(PWR_LATCH_GPIO, 0);
            }
        }

        rgbLed->setState(LEDPattern::Breathe, color);

        vTaskDelay(pdMS_TO_TICKS(PWR_POLL_MS));
    }

    
}

#if CONFIG_BT_NIMBLE_ENABLED

/* Process BLE command JSON and update device parameters */
static void ble_process_command(const uint8_t *data, uint16_t length)
{
    if (data == nullptr || length == 0) {
        return;
    }

    ESP_LOGI(TAG, "Processing BLE command: %.*s", static_cast<int>(length),
             reinterpret_cast<const char *>(data));

    if (ble_rx_queue == NULL) {
        ESP_LOGW(TAG, "BLE RX queue not initialized");
        return;
    }

    ble_rx_message_t msg;
    msg.len = length >= sizeof(msg.data) ? sizeof(msg.data) - 1 : length;
    memcpy(msg.data, data, msg.len);

    /* Send to queue with timeout; drop message if queue is full */
    BaseType_t res = xQueueSend(ble_rx_queue, &msg, pdMS_TO_TICKS(100));
    if (res != pdPASS) {
        ESP_LOGW(TAG, "BLE RX queue full, dropping message");
    }
}

/* Initialize NimBLE and register GATT services */
static void ble_init(void)
{  
    /* Initialize UART/Telemetry service */
    uart_service.setReceiveCallback(ble_process_command);
    
    /* Initialize OTA service */
    ota_service.setStartCallback([](uint32_t expected_size) {
        ESP_LOGI(TAG, "OTA Start: expecting %lu bytes", expected_size);
    });
    ota_service.setProgressCallback([](uint32_t received, uint32_t total) {
        ESP_LOGD(TAG, "OTA Progress: %lu/%lu bytes", received, total);
    });
    ota_service.setFinishCallback([](bool success, const char* error_msg) {
        if (success) {
            ESP_LOGI(TAG, "OTA finished successfully, device will reboot");
        } else {
            ESP_LOGE(TAG, "OTA failed: %s", error_msg ? error_msg : "unknown error");
        }
    });

    ble.init({ &uart_service, &ota_service, &can_console_uart_service });
    
}

/* Task to broadcast telemetry data as JSON */
static void ble_telemetry_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for BLE to initialize
    TickType_t last_telemetry_tick = xTaskGetTickCount();

    while (1) {
        if (__atomic_exchange_n(&mode_toggle_pending, false, __ATOMIC_ACQ_REL)) {
            toggle_uart_mode();
        }

        uart_service.send_queue_messages();
        can_console_uart_service.send_queue_messages();

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_telemetry_tick) >= pdMS_TO_TICKS(500)) {
            last_telemetry_tick = now;
            cJSON *telemetry = cJSON_CreateObject();
            if (telemetry == NULL) {
                ESP_LOGE(TAG, "Failed to create JSON object");
            } else {
                /* Add telemetry fields */
                char buf_temp[32];
                snprintf(buf_temp, sizeof(buf_temp), "%.1f", device_temperature_c);
                cJSON_AddStringToObject(telemetry, "tb", buf_temp);
                cJSON_AddNumberToObject(telemetry, "pLA", pulses_left_A);
                cJSON_AddNumberToObject(telemetry, "pLB", pulses_left_B);
                cJSON_AddNumberToObject(telemetry, "pLC", pulses_left_C);
                cJSON_AddNumberToObject(telemetry, "pRA", pulses_right_A);
                cJSON_AddNumberToObject(telemetry, "pRB", pulses_right_B);
                cJSON_AddNumberToObject(telemetry, "pRC", pulses_right_C);
                cJSON_AddNumberToObject(telemetry, "pL", pulses_left_A + pulses_left_B + pulses_left_C);
                cJSON_AddNumberToObject(telemetry, "pR", pulses_right_A + pulses_right_B + pulses_right_C);
                cJSON_AddNumberToObject(telemetry, "rL", rpm_left);
                cJSON_AddNumberToObject(telemetry, "rR", rpm_right);
                char buf_speed[32];
                snprintf(buf_speed, sizeof(buf_speed), "%.1f", speed_mph);
                cJSON_AddRawToObject(telemetry, "spd", buf_speed);
                char buf_odometer[32];
                snprintf(buf_odometer, sizeof(buf_odometer), "%.2f", trip_distance_miles);
                cJSON_AddRawToObject(telemetry, "odo", buf_odometer);
                char buf_accessory_watts[32];
                snprintf(buf_accessory_watts, sizeof(buf_accessory_watts), "%.1f", accessory_watts);
                cJSON_AddRawToObject(telemetry, "accessory_watts", buf_accessory_watts);
                cJSON_AddNumberToObject(telemetry, "accessory_current_12v_ma",
                                        static_cast<int>(accessory_current_12v_ma));
                cJSON_AddNumberToObject(telemetry, "accessory_current_5v_ma",
                                        static_cast<int>(accessory_current_5v_ma));

                if(bms != NULL){
                    cJSON_AddNumberToObject(telemetry, "bV", bms->getPackVoltageMillivolts());
                    cJSON_AddNumberToObject(telemetry, "bI", bms->getPackCurrentMilliamps());
                    cJSON_AddNumberToObject(telemetry, "sc", (uint32_t)bms->getSoC());
                    cJSON_AddNumberToObject(telemetry, "ec", bms->getEnergyConsumed());
                }

                /* Serialize to string */
                char *json_str = cJSON_PrintUnformatted(telemetry);
                if (json_str == NULL) {
                    ESP_LOGE(TAG, "Failed to serialize JSON");
                } else {
                    /* Send telemetry via UART service */
                    uart_service.sendTelemetry(json_str);
                    cJSON_free(json_str);
                }

                cJSON_Delete(telemetry);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#endif /* CONFIG_BT_NIMBLE_ENABLED */

/**
 * Initialize SPI bus and device for MCP2515 CAN controller
 * Returns the spi_device_handle_t for the MCP2515 device
 */
static spi_device_handle_t spi_mcp2515_init(void)
{
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = SPI_MISO_PIN;
    buscfg.mosi_io_num = SPI_MOSI_PIN;
    buscfg.sclk_io_num = SPI_CLK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;

    // Initialize the SPI bus on SPI2_HOST (HSPI)
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return NULL;
    }
    ESP_LOGI(TAG, "SPI bus initialized on SPI2_HOST");

    // Configure SPI device for MCP2515
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1 * 1000 * 1000;  // 1 MHz
    devcfg.mode = 0;                           // SPI_MODE0 (CPOL=0, CPHA=0)
    devcfg.spics_io_num = SPI_CS_PIN;
    devcfg.queue_size = 7;

    spi_device_handle_t handle;
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return NULL;
    }
    ESP_LOGI(TAG, "MCP2515 SPI device added to bus with CS on GPIO %d", SPI_CS_PIN);

    return handle;
}

/* --- UART (ISO) helper: simple init and RX task
 * Uses pins defined by UART_ISO_TX and UART_ISO_RX.
 * Provides a placeholder `uart_iso_handle_data` that will be called
 * whenever bytes are received. Keep handling minimal; implement
 * actual parsing in `uart_iso_handle_data` later.
 */
static void uart_iso_handle_data(const uint8_t* data, size_t len)
{
    // Placeholder: user should implement processing of received data here.
    // Keep this function short and non-blocking.
    (void)data;
    (void)len;
}

static void uart_iso_rx_task(void *arg)
{
    (void)arg;
    const int buf_size = 1024 * 2;
    uint8_t *data = (uint8_t*) malloc(buf_size);
    if (!data) {
        ESP_LOGE(TAG, "uart_iso_rx_task: failed to allocate buffer");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        int len = uart_read_bytes(UART_NUM_2, data, buf_size, pdMS_TO_TICKS(100));
        if (len > 0) {
            uart_iso_handle_data(data, (size_t)len);
        }
        // small delay to yield
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(data);
}

static void uart_iso_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Configure UART parameters on UART2 (adjust if you want a different UART)
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));

    // Set UART pins to the ISO pins defined in this file
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_ISO_TX, UART_ISO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    const int uart_buffer_size = (1024 * 2);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 0, NULL, 0));

    // Start RX task to poll for incoming bytes; this keeps the implementation simple
    xTaskCreate(uart_iso_rx_task, "uart_iso_rx", 2048, NULL, 5, NULL);
}

static void configurePowerSenseGPIO(gpio_num_t gpio){
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

static void configureModeButtonGPIO(gpio_num_t gpio){
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio, mode_button_isr_handler, (void*)(intptr_t)gpio);
}

static void configureHallGPIO(gpio_num_t gpio){
    /* Configure hall GPIO */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
        gpio_isr_handler_add(gpio, hall_isr_handler, (void*)(intptr_t)gpio);

}

static void adc_init(void){
    // ADC init (oneshot API for esp-idf v5+)
    if (adc_handle == NULL) {
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_BATT_CHANNEL, &chan_cfg));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CURR_HS_CHANNEL, &chan_cfg));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CURR_REF_CHANNEL, &chan_cfg));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_12V_CURR_CHANNEL, &chan_cfg));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_5V_CURR_CHANNEL, &chan_cfg));
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting SkateInfoView ESP32-S3");
    esp_ota_mark_app_valid_cancel_rollback();

    /* Create resources */
    pulse_mutex = xSemaphoreCreateMutex();
    ble_rx_queue = xQueueCreate(BLE_RX_QUEUE_SIZE, sizeof(ble_rx_message_t));

    gpio_config_t io_conf_exp_latch = {
        .pin_bit_mask = (1ULL << PWR_LATCH_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf_exp_latch);
    gpio_set_level(PWR_LATCH_GPIO, 1);

    gpio_config_t io_conf_exp_5V = {
        .pin_bit_mask = (1ULL << EXP_5V_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf_exp_5V);
    gpio_set_level(EXP_5V_GPIO, 1);

    gpio_config_t io_conf_exp_12V = {
        .pin_bit_mask = (1ULL << EXP_12V_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf_exp_12V);
    gpio_set_level(EXP_12V_GPIO, 1);

    // Initialize UART for ISO communications (uses `UART_ISO_TX`/`UART_ISO_RX`)
    uart_iso_init();
    nvs = new NVSStorage("bms");
    ESP_ERROR_CHECK(nvs->init());
    esp_err_t board_configuration_err = board_configuration_load(*nvs, boardConfiguration);
    if (board_configuration_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load board configuration: %s", esp_err_to_name(board_configuration_err));
    }

    // Initialize I2C bus (SDA on IO10, SCL on IO9) for TMP117 and SSD1306 OLED
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));
    ESP_LOGI(TAG, "I2C bus created on I2C_NUM_0 (SDA=IO10, SCL=IO9)");

    // Longer delay to ensure I2C bus is ready
    vTaskDelay(pdMS_TO_TICKS(100));
    
    if (boardConfiguration.hardwareFeatureEnable.tmp117SensorEnabled) {
        // Probe I2C bus first to verify hardware is present
        esp_err_t probe_ret = i2c_master_probe(i2c_bus_handle, 0x48, -1);
        if (probe_ret != ESP_OK) {
            ESP_LOGW(TAG, "I2C probe failed for TMP117 at 0x48: %s (device may not be connected)", esp_err_to_name(probe_ret));
        } else {
            ESP_LOGI(TAG, "I2C probe successful - TMP117 device found at 0x48");
        }

        // Initialize TMP117 temperature sensor on shared I2C bus
        tmp117_sensor = new TMP117(i2c_bus_handle, 0x48);  // Default TMP117 address
        esp_err_t tmp117_ret = tmp117_sensor->init();
        if (tmp117_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize TMP117 sensor: %s", esp_err_to_name(tmp117_ret));
        } else {
            ESP_LOGI(TAG, "TMP117 temperature sensor initialized successfully");
        }
    }

    if (boardConfiguration.hardwareFeatureEnable.motorSenseEnable) {
        configureHallGPIO(HALL_LEFT_A_GPIO);
        configureHallGPIO(HALL_LEFT_B_GPIO);
        configureHallGPIO(HALL_LEFT_C_GPIO);
        configureHallGPIO(HALL_RIGHT_A_GPIO);
        configureHallGPIO(HALL_RIGHT_B_GPIO);
        configureHallGPIO(HALL_RIGHT_C_GPIO);
        configurePowerSenseGPIO(ESC_PWR_SENSE_GPIO);
    }
    configureModeButtonGPIO(MODE_BUTTON_GPIO);

    adc_init();
   
    ESP32LED::configurePWMPin(HEADLIGHT_OUT_GPIO, HEADLIGHT_LEDC_CHANNEL);
    ESP32LED::configurePWMPin(BRAKELIGHT_OUT_GPIO, BRAKELIGHT_LEDC_CHANNEL);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, HEADLIGHT_LEDC_CHANNEL,
                                  headlight_duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, HEADLIGHT_LEDC_CHANNEL));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, BRAKELIGHT_LEDC_CHANNEL,
                                  brake_light_duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, BRAKELIGHT_LEDC_CHANNEL));

    if (boardConfiguration.hardwareFeatureEnable.canbusEnabled) {
        bool status = false;
        if (boardConfiguration.hardwareFeatureEnable.useMCP2515) {
            // SPI pins: MOSI=13, MISO=12, SCK=14, CS=11
            spi_device_handle_t spi_handle = spi_mcp2515_init();
            if (spi_handle != NULL) {
                canBus = new MCP2515_CANController(spi_handle, MCP_8MHZ, MCP_ANY);
                status = canBus->begin(500000);
            } else {
                ESP_LOGE(TAG, "Failed to initialize SPI for MCP2515");
            }
        } else {
            canBus = new ESPCANController(ESP_CANTX, ESP_CANRX);
            status = canBus->begin(500000);
        }

        if (!status) {
            ESP_LOGE(TAG, "Failed to initialize %s CAN controller",
                     boardConfiguration.hardwareFeatureEnable.useMCP2515 ? "MCP2515" : "ESP");
            delete canBus;
            canBus = NULL;
        } else {
            ESP_LOGI(TAG, "%s CAN controller initialized successfully at 500 kbps",
                     boardConfiguration.hardwareFeatureEnable.useMCP2515 ? "MCP2515" : "ESP");
        }
    } else {
        ESP_LOGI(TAG, "CAN bus disabled by board configuration");
    }

#if CONFIG_BT_NIMBLE_ENABLED
    // Only claim the USB Serial/JTAG peripheral for the menu console when requested;
    // otherwise leave it free for esp_log output and OpenOCD debugging.
    if (boardConfiguration.uartConfig == STANDARD_MENU) {
        if (!usb_console.begin()) {
            ESP_LOGE(TAG, "Failed to initialize USB serial console");
        }
        esp_log_level_set("*", ESP_LOG_NONE);
    } else {
        ESP_LOGI(TAG, "USB Serial/JTAG reserved for OpenOCD debug console");
    }
#endif

    BatteryConfiguration default_battery_config{
        .crc32 = 0,
        .battery_curve_mv = { 3100, 3300, 3425, 3550, 3625, 3700, 3775, 3850, 3925, 4000, 4100 },
        .battery_designed_capacity_mAh = 8500,
        .battery_cells_parallel = 3,
        .battery_cells_series = 12,
        .temperature_curve_temperatures_c = {-25, -10, 0, 10, 20, 30, 40, 50, 60, 70},
        .discharge_limits_ma = { 10000, 15000, 30000, 60000, 60000, 60000, 60000, 60000, 40000, 10000 },
        .charge_limits_ma = { 2000, 3000, 10000, 15000, 15000, 15000, 15000, 10000, 3000, 1000 }
    };

    vTaskDelay(pdMS_TO_TICKS(200));

    // NOTE: This needs to have some delay after the power latch is set or we don't get battery voltage.
    // Might only be a problem when powered by USB.
    pack_voltage_mv = readPackVoltageMillivolts(false);    
    
    ESP_LOGI(TAG, "Read initial pack voltage to be %lu", pack_voltage_mv);

    bms = new IntegratedBMS(nvs);
    bms->init(default_battery_config, pack_voltage_mv, pack_idle_current_ma);

    ESP_LOGI(TAG, "BMS initial values %lu", bms->getEnergyRemaining());

    xTaskCreate(board_configuration_task, "board_config", 2048, NULL, 4, NULL);

    /* Start sensor and rpm tasks */
    if (boardConfiguration.hardwareFeatureEnable.motorSenseEnable) {
        xTaskCreate(rpm_task, "rpm_task", 4096, &boardConfiguration, 5, NULL);
    }
    // ADC sampler runs faster and provides averaged raw ADC values for accessories
    xTaskCreate(adc_sampler_task, "adc_sampler", 2048, NULL, 6, NULL);
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    if (boardConfiguration.hardwareFeatureEnable.tmp117SensorEnabled) {
        xTaskCreate(tmp117_sensor_task, "tmp117_sensor_task", 2048, NULL, 5, NULL);
    }
    if (canBus != NULL) {
        xTaskCreate(canbus_task, "canbus_task", 4096, NULL, 5, NULL);
    }
    xTaskCreate(power_control_task, "power_control_Task", 4096, NULL, 6, NULL);

#if CONFIG_BT_NIMBLE_ENABLED
    /* Initialize BLE and start telemetry task */
    ble_init();
    if (canBus != NULL) {
        canalyzer = new CANAnalyzerStack(
            console_combiner,
            *canBus,
            canalyzer_rx_buffer,
            CANALYZER_RX_BUFFER_SIZE,
            &canalyzer_rx_buffer_index,
            canalyzer_tx_buffer,
            CANALYZER_TX_BUFFER_SIZE,
            &canalyzer_tx_buffer_index,
            canalyzer_get_millis,
            canalyzer_tx_callback);
    }
    initialize_menu_system();
    xTaskCreate(menu_task, "ble_menu_task", 4096, NULL, 5, NULL);
    xTaskCreate(ble_telemetry_task, "ble_telemetry_task", 4096, NULL, 5, NULL);
#endif
}
