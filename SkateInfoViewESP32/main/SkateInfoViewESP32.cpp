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

#include "RGB_LED.h"
#include "TCS34725.h"
#include "mcp_can.h"

/* ----------------------------
 * NimBLE / GATT server
 * ---------------------------- */

#if CONFIG_BT_NIMBLE_ENABLED
extern "C" {
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
}
#endif

static const char *TAG = "skateinfo";

// Hardware mapping - adjust these for your board/pins
#define HALL_LEFT_A_GPIO     GPIO_NUM_40
#define HALL_LEFT_B_GPIO     GPIO_NUM_39
#define HALL_LEFT_C_GPIO     GPIO_NUM_38

#define HALL_RIGHT_A_GPIO     GPIO_NUM_21
#define HALL_RIGHT_B_GPIO     GPIO_NUM_18
#define HALL_RIGHT_C_GPIO     GPIO_NUM_17

#define UART_ISO_TX          GPIO_NUM_37
#define UART_ISO_RX          GPIO_NUM_36

#define EXP_5V_GPIO          GPIO_NUM_41
#define EXP_12V_GPIO         GPIO_NUM_42
#define PWR_LATCH_GPIO       GPIO_NUM_2
#define ESC_PWR_SENSE_GPIO   GPIO_NUM_1
#define WHEEL_RPM_RATIO_L    12
#define WHEEL_RPM_RATIO_R    12
// Wheel diameter in millimeters (used to compute circumference)
#define WHEEL_SIZE_MM        90
#define WHEEL_SIZE_MM        103            // Size of wheel used for speed calculation from RPM. TODO: Configurable in NVS


// SPI pins for MCP2515 CAN Controller
#define SPI_MOSI_PIN         GPIO_NUM_13
#define SPI_MISO_PIN         GPIO_NUM_12
#define SPI_CLK_PIN          GPIO_NUM_14
#define SPI_CS_PIN           GPIO_NUM_11

// ADC channels (update to match new wiring)
#define ADC_BATT_CHANNEL     ADC_CHANNEL_4  // ADC1_4 is actually IO5. For battery voltage
#define ADC_CURR_HS_CHANNEL  ADC_CHANNEL_3  // ADC1_3 is actually IO4. For battery current
#define ADC_CURR_REF_CHANNEL ADC_CHANNEL_2  // ADC1_2 is actually IO3. For battery current reference


// Sampling / timings
#define PWR_POLL_MS             100
#define RPM_POLL_MS             300
#define IDLE_POWEROFF_MS        300000

// Power Macros
#define BATT_CELL_MIN           3.0         // Voltage at which cell is considered to be 0% charged         
#define BATT_CELL_MAX           4.10        // Voltage at which cell is considered to be 100% charged
#define BATT_CELL_COUNT         12          // Number of series cell in pack. TODO: make this NVS storeable

/* GATT service/characteristic UUIDs (Nordic UART Service).
 * Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * RX char (write from central): 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 * TX char (notify from peripheral): 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 */
#define GATT_SVC_UUID_STR       "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define GATT_CHR_RX_UUID_STR    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define GATT_CHR_TX_UUID_STR    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#if CONFIG_BT_NIMBLE_ENABLED
/* Statically allocate BLE UUID structures for C++ compilation. The
 * original IDF macros use C compound literals and take their address,
 * which is not allowed in C++. Declare static constants and use their
 * addresses instead.
 * Nordic UART Service uses 128-bit UUIDs.
 */
static const ble_uuid128_t gatt_svc_uuid_struct = {
    .u = { .type = BLE_UUID_TYPE_128 },
    .value = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E}
};
static const ble_uuid128_t gatt_chr_rx_uuid_struct = {
    .u = { .type = BLE_UUID_TYPE_128 },
    .value = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E}
};
static const ble_uuid128_t gatt_chr_tx_uuid_struct = {
    .u = { .type = BLE_UUID_TYPE_128 },
    .value = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E}
};
#endif


// Shared state
ILED* rgbLed;
static volatile uint32_t pulse_count_left = 0; // increments from ISR
static volatile uint32_t pulse_count_right = 0; // increments from ISR
static uint32_t rev_count_left = 0;
static uint32_t rev_count_right = 0;
static uint32_t rpm_left = 0;
static uint32_t rpm_right = 0;
static uint32_t avg_rpm = 0;
static float speed_mph = 0.0f;
static double trip_distance_miles;
static float mAH_consumption = 0.0f;
static int state_of_charge = 0;
static uint32_t last_integration_time_ms = 0;
static volatile bool esc_powered = false;
static SemaphoreHandle_t pulse_mutex = NULL;

// ADC handle for esp-idf v5 oneshot API
static adc_oneshot_unit_handle_t adc_handle = NULL;
static SemaphoreHandle_t gatt_tx_mutex = NULL;

/* GATT TX buffer and handle (declared early so sensor_task can use it when
 * BLE is enabled). The actual GATT registration will set the value handle.
 */
static uint8_t gatt_svr_tx_val[512];
static uint16_t gatt_svr_tx_val_handle = 0;

// ISR for hall pulse
static void IRAM_ATTR hall_isr_handler(void *arg)
{
    // keep ISR short
    int gpio_num = (int)(intptr_t)arg;

    // Compare which pin triggered the ISR and handle accordingly.
    // Only do minimal, ISR-safe work here.
    if (gpio_num == HALL_LEFT_A_GPIO || 
        gpio_num == HALL_LEFT_B_GPIO || 
        gpio_num == HALL_LEFT_C_GPIO) 
    {
        pulse_count_left++;
    } 
    else if (gpio_num == HALL_RIGHT_A_GPIO || 
             gpio_num == HALL_RIGHT_B_GPIO || 
             gpio_num == HALL_RIGHT_C_GPIO) 
    {
        pulse_count_right++;
    }
}

// Placeholder: notify connected BLE peers for a characteristic
static void ble_notify_placeholder(const char *name, int value)
{
    // Implement NimBLE/BT GATT notification here. This function is a stub
    // so the project builds without BLE and gives a clear hook for later.
    (void)name;
    (void)value;
}

static void rpm_task(void *arg)
{
    (void)arg;
    uint32_t last_count = 0;
    static int64_t last_time_us = esp_timer_get_time();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // every second

        int64_t now_us = esp_timer_get_time();
        double dt_s = (double)(now_us - last_time_us) / 1000000.0;
        if (dt_s <= 0.0) dt_s = 0.001; // guard against zero
        last_time_us = now_us;

        // atomically get & clear pulse_count (ISR increments pulse_count)
        uint32_t pulses_left = __atomic_exchange_n(&pulse_count_left, 0U, __ATOMIC_ACQ_REL);
        uint32_t pulses_right = __atomic_exchange_n(&pulse_count_right, 0U, __ATOMIC_ACQ_REL);

        // Compute revolutions during the interval
        double revs_left = (double)pulses_left / (double)WHEEL_RPM_RATIO_L;
        double revs_right = (double)pulses_right / (double)WHEEL_RPM_RATIO_R;

        // RPS and RPM
        double rps_left = revs_left / dt_s;
        double rps_right = revs_right / dt_s;
        rpm_left = (uint32_t)(rps_left * 60.0);
        rpm_right = (uint32_t)(rps_right * 60.0);

        // Update revolution counters (integral count)
        rev_count_left += (uint32_t)revs_left;
        rev_count_right += (uint32_t)revs_right;

        // Average RPM across both wheels
        avg_rpm = (((double)rpm_left) + ((double)rpm_right)) / 2.0;

        // Compute speed (assume WHEEL_SIZE_MM is diameter in mm -> circumference = pi * d)
        double wheel_circ_m = (WHEEL_SIZE_MM / 1000.0) * M_PI;
        double avg_rps = avg_rpm / 60.0;
        double speed_m_per_s = avg_rps * wheel_circ_m;
        // Convert m/s to MPH (1 m/s = 2.2369362920544 mph)
        speed_mph = speed_m_per_s * 2.2369362920544;

        // Integrate distance (meters)
        trip_distance_miles += (speed_mph * dt_s) / 3600.0;
    }
}

static void sensor_task(void *arg)
{
    (void)arg;

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
    }

    last_integration_time_ms = esp_log_timestamp();

    while (1) {
        uint32_t now = esp_log_timestamp();

        int raw_batt = 0;
        int raw_curr = 0;
        int ref = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CURR_REF_CHANNEL, &ref));  //Read this first to activate the channel to prevent ghost reading
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_BATT_CHANNEL, &raw_batt));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CURR_HS_CHANNEL, &raw_curr));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CURR_REF_CHANNEL, &ref));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CURR_HS_CHANNEL, &raw_curr));
        const float voltDividerRatio = 39.5f;
        const float shuntRatio = 42.0f;
        const float adc_cal = 35.0f;

        // Translate raw ADC (0-4095) to approximate values used in original sketch
        // Note: you should calibrate these formulas for your voltage divider and sense resistor
        float battVoltageRaw = (float)raw_batt; // similar to analogRead(A1)
        float battVoltageCorr = (battVoltageRaw * 1000.0f / voltDividerRatio); // update formula as needed

        // SOC estimate
        const float cell_voltage_range = BATT_CELL_MAX - BATT_CELL_MIN;
        state_of_charge = (100 * ((battVoltageCorr * 0.001 / BATT_CELL_COUNT) - BATT_CELL_MIN)) / cell_voltage_range;
        if(state_of_charge > 100) state_of_charge = 100;
        else if(state_of_charge < 0) state_of_charge = 0;

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

        float battCurrentmA = 0.0f;
        for (int i = 0; i < curr_count; ++i) battCurrentmA += curr_buf[i];
        if (curr_count > 0) battCurrentmA /= (float)curr_count;

        // integrate mAh
        uint32_t dt_ms = now - last_integration_time_ms;
        if (dt_ms > 0) {
            mAH_consumption += battCurrentmA * ((float)dt_ms / 3600000.0f);
            last_integration_time_ms = now;
        }

        //ESP_LOGI(TAG, "battRaw=%d battCorr=%.0f mA=%.1f mAH=%.3f", raw_batt, battVoltageCorr, battCurrentmA, mAH_consumption);

        // Send periodic JSON sensor notification over BLE (TX characteristic)
    #if CONFIG_BT_NIMBLE_ENABLED
        {
            char json[192];
            int n = snprintf(json, sizeof(json), "{\"batt_mV\":%.0f,\"curr_mA\":%.1f,\"mAH\":%d,\"soc\":%d,\"trip\":%0.2f,\"mph\":%.1f,\"reva\":%" PRIu32 ",\"rev_l\":%" PRIu32 ",\"rev_r\":%" PRIu32 "}",
                     battVoltageCorr, battCurrentmA, (int)mAH_consumption, state_of_charge, trip_distance_miles, speed_mph, avg_rpm, rpm_left, rpm_right);
            int copy_len = n > (int)sizeof(gatt_svr_tx_val) - 1 ? (int)sizeof(gatt_svr_tx_val) - 1 : n;
            if (gatt_tx_mutex) xSemaphoreTake(gatt_tx_mutex, portMAX_DELAY);
            memcpy(gatt_svr_tx_val, json, copy_len);
            gatt_svr_tx_val[copy_len] = '\0';
            if (gatt_tx_mutex) xSemaphoreGive(gatt_tx_mutex);
            /* notify subscribed centrals */
            ble_gatts_chr_updated(gatt_svr_tx_val_handle);
        }
    #endif

        vTaskDelay(pdMS_TO_TICKS(RPM_POLL_MS));
    }
}

static void power_control_task(void *arg){
    // Instantiate the RGB LED (example pins: 6, 7, 8)
    rgbLed = new ESP32LED(8, 7, 6);

    RGBColor color{
        .r = 0,
        .g = 255,
        .b = 0
    };

    rgbLed->setPattern(LEDPattern::Breathe, color);

    static TickType_t last_esc_on_time = xTaskGetTickCount();

    while(1){
        //esc_powered = gpio_get_level(ESC_PWR_SENSE_GPIO);
        esc_powered = (rpm_left > 0) || (rpm_right > 0);

        if(esc_powered){
            color.b = 255;
            color.g = 255;
            color.r = 0;

            last_esc_on_time = xTaskGetTickCount();
        }
        else{
            color.b = 0;
            color.g = 255;
            color.r = 0;

            if(pdTICKS_TO_MS(xTaskGetTickCount() - last_esc_on_time) > IDLE_POWEROFF_MS){
                color.b = 0;
                color.g = 255;
                color.r = 255;

                gpio_set_level(PWR_LATCH_GPIO, 0);
            }
        }

        rgbLed->setPattern(LEDPattern::Breathe, color);

        vTaskDelay(pdMS_TO_TICKS(PWR_POLL_MS));
    }

    
}

#if CONFIG_BT_NIMBLE_ENABLED

/* Forward declarations */
static int gatt_svr_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg);
static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

/* GATT service definition */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t *)&gatt_svc_uuid_struct,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = (ble_uuid_t *)&gatt_chr_rx_uuid_struct,
                .access_cb = gatt_svr_chr_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = (ble_uuid_t *)&gatt_chr_tx_uuid_struct,
                .access_cb = gatt_svr_chr_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gatt_svr_tx_val_handle,
            },
            {
                0
            }
        }
    },
    {
        0
    }
};

/* Utility to copy ombuf to flat buffer */
static int gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len, void *dst, uint16_t *out_len)
{
    int rc;
    uint16_t om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    rc = ble_hs_mbuf_to_flat(om, dst, max_len, out_len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

static int gatt_svr_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    const ble_uuid_t *uuid = ctxt->chr->uuid;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            /* Received data from central */
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            if (len > sizeof(gatt_svr_tx_val) - 1) len = sizeof(gatt_svr_tx_val) - 1;
            rc = gatt_svr_write(ctxt->om, 0, sizeof(gatt_svr_tx_val) - 1, gatt_svr_tx_val, &len);
            if (rc != 0) return rc;
            gatt_svr_tx_val[len] = '\0';
            ESP_LOGI(TAG, "BLE RX: %s", (char*)gatt_svr_tx_val);

            /* Create an ACK JSON and notify back on TX characteristic */
            char resp[128];
            int r = snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"len\":%d}", (int)len);
            if (r > 0) {
                /* copy to the tx value storage (protect with mutex) */
                int copy_len = r > (int)sizeof(gatt_svr_tx_val)-1 ? (int)sizeof(gatt_svr_tx_val)-1 : r;
                if (gatt_tx_mutex) xSemaphoreTake(gatt_tx_mutex, portMAX_DELAY);
                memcpy(gatt_svr_tx_val, resp, copy_len);
                gatt_svr_tx_val[copy_len] = '\0';
                if (gatt_tx_mutex) xSemaphoreGive(gatt_tx_mutex);
                /* notify subscribed centrals */
                ble_gatts_chr_updated(gatt_svr_tx_val_handle);
            }
            return 0;
        }
        case BLE_GATT_ACCESS_OP_READ_CHR: {
            /* If central reads a characteristic, supply current TX value */
            if (attr_handle == gatt_svr_tx_val_handle) {
                rc = os_mbuf_append(ctxt->om, gatt_svr_tx_val, strlen((char*)gatt_svr_tx_val));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            return BLE_ATT_ERR_UNLIKELY;
        }
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];
    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            ESP_LOGI(TAG, "registered service %s with handle=%d", ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
            break;
        case BLE_GATT_REGISTER_OP_CHR:
            ESP_LOGI(TAG, "registering characteristic %s def_handle=%d val_handle=%d", ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf), ctxt->chr.def_handle, ctxt->chr.val_handle);
            break;
        case BLE_GATT_REGISTER_OP_DSC:
            ESP_LOGI(TAG, "registering descriptor %s handle=%d", ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
            break;
        default:
            break;
    }
}

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    /* Advertise our 128-bit service UUID */
    static const ble_uuid128_t adv_uuids[] = { gatt_svc_uuid_struct };
    fields.uuids128 = (ble_uuid128_t *)adv_uuids;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    /* Device name */
    const char *name = "SK01";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
        return;
    }
}

static void ble_app_on_sync(void)
{
    int rc;
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Use public address type */
    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    assert(rc == 0);

    /* Set name */
    ble_svc_gap_device_name_set("SK01");

    /* Start advertising */
    ble_app_advertise();
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* Initialize NimBLE and register GATT services (only compiled when NimBLE enabled) */
static void ble_init(void)
{
    int rc;
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing NimBLE");

    /* Initialize NVS (required by BLE) */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize NimBLE stack */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d", ret);
        return;
    }

    /* Configure NimBLE host callbacks */
    ble_hs_cfg.reset_cb = NULL;
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;

    /* Initialize GATT services */
    ble_svc_gap_init();
    ble_svc_gatt_init();
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed %d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed %d", rc);
        return;
    }

    /* Start the NimBLE host in its own RTOS task */
    nimble_port_freertos_init(ble_host_task);
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

    // Configure CS pin as output (SPI driver should handle it, but ensure it's set up)
    gpio_config_t dig_conf = {};
    dig_conf.pin_bit_mask = (1ULL << GPIO_NUM_10);
    dig_conf.mode = GPIO_MODE_OUTPUT;
    dig_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    dig_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dig_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&dig_conf);
    gpio_set_level(GPIO_NUM_10, 0);  // CS high initially (inactive)

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

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting SkateInfoView ESP32-S3");

    /* Create resources */
    pulse_mutex = xSemaphoreCreateMutex();
    gatt_tx_mutex = xSemaphoreCreateMutex();

    // Initialize UART for ISO communications (uses `UART_ISO_TX`/`UART_ISO_RX`)
    uart_iso_init();

    configureHallGPIO(HALL_LEFT_A_GPIO);
    configureHallGPIO(HALL_LEFT_B_GPIO);
    configureHallGPIO(HALL_LEFT_C_GPIO);
    configureHallGPIO(HALL_RIGHT_A_GPIO);
    configureHallGPIO(HALL_RIGHT_B_GPIO);
    configureHallGPIO(HALL_RIGHT_C_GPIO);

    configurePowerSenseGPIO(ESC_PWR_SENSE_GPIO);

    /* Start sensor and rpm tasks */
    xTaskCreate(rpm_task, "rpm_task", 4096, NULL, 5, NULL);
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(power_control_task, "power_control_Task", 4096, NULL, 6, NULL);
   
    configurePWMPin(15, LEDC_CHANNEL_3);
    configurePWMPin(16, LEDC_CHANNEL_4);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4));

#if CONFIG_BT_NIMBLE_ENABLED
    /* Initialize NimBLE and start host task */
    ble_init();

#endif

    

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

    gpio_config_t io_conf_exp_latch = {
        .pin_bit_mask = (1ULL << PWR_LATCH_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf_exp_latch);
    gpio_set_level(PWR_LATCH_GPIO, 1);

    //TCS34725 tcs(GPIO_NUM_9, GPIO_NUM_10);
    //tcs.init();

    /*for(;;){
        RGBWColor c;
        tcs.getRawColor(&c);
    
        ESP_LOGI("Color", "Color: %d %d %d", c.red, c.green, c.blue);
        vTaskDelay(pdMS_TO_TICKS(500));
    }*/

    // --- MCP2515 CAN Controller Initialization
    // SPI pins: MOSI=13, MISO=12, SCK=14, CS=11
    // Initialize SPI bus and get device handle
    spi_device_handle_t spi_handle = spi_mcp2515_init();
    if (spi_handle == NULL) {
        ESP_LOGE(TAG, "Failed to initialize SPI for MCP2515");
        return;
    }

    // Create MCP_CAN instance with the SPI device handle
    MCP_CAN *canBus = new MCP_CAN(spi_handle);
    
    // Initialize CAN bus: 500 kbps, 8 MHz oscillator, standard/extended ID mode
    INT8U status = canBus->begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
    if (status != CAN_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCP2515 CAN controller, status: %d", status);
    } else {
        ESP_LOGI(TAG, "MCP2515 CAN controller initialized successfully at 500 kbps");
    }
    canBus->setMode(MCP_NORMAL);

    /*for(;;){
        uint8_t data[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};
        uint8_t re = canBus->sendMsgBuf(0x100, 8, data);
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI("CANBUS", "TX CAN %d", (int)re);
    }*/
}
