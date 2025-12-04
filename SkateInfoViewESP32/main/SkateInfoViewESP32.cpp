/*
 * SkateInfoView
 * An electric skateboard battery monitor based on ESP32
 */



#include <stdio.h>
#include <inttypes.h>
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
#include "esp_log.h"
#include <string.h>
#include "esp_system.h"
#include "driver/ledc.h"

#include "RGB_LED.h"

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
#define HALL_GPIO            GPIO_NUM_38
#define NUM_WHEEL_MAG        6


// ADC channels (update to match new wiring)
#define ADC_BATT_CHANNEL     ADC_CHANNEL_4  // ADC1_4 is actually IO5. For battery voltage
#define ADC_CURR_HS_CHANNEL  ADC_CHANNEL_3  // ADC1_3 is actually IO4. For battery current

// Sampling / timings
#define POLL_MS              300

/* GATT service/characteristic UUIDs (16-bit used here).
 * Service: 0xFFE0
 * RX char (write from central): 0xFFE1
 * TX char (notify from peripheral): 0xFFE2
 */
#define GATT_SVC_UUID           0xFFE0
#define GATT_CHR_RX_UUID        0xFFE1
#define GATT_CHR_TX_UUID        0xFFE2

#if CONFIG_BT_NIMBLE_ENABLED
/* Statically allocate BLE UUID structures for C++ compilation. The
 * original IDF macros use C compound literals and take their address,
 * which is not allowed in C++. Declare static constants and use their
 * addresses instead.
 */
static const ble_uuid16_t gatt_svc_uuid_struct = { .u = { .type = BLE_UUID_TYPE_16 }, .value = GATT_SVC_UUID };
static const ble_uuid16_t gatt_chr_rx_uuid_struct = { .u = { .type = BLE_UUID_TYPE_16 }, .value = GATT_CHR_RX_UUID };
static const ble_uuid16_t gatt_chr_tx_uuid_struct = { .u = { .type = BLE_UUID_TYPE_16 }, .value = GATT_CHR_TX_UUID };
#endif


// Shared state
static volatile uint32_t pulse_count = 0; // increments from ISR
static uint32_t rev_count = 0;
static uint32_t last_poll_time = 0;
static float mAH_consumption = 0.0f;
static uint32_t last_integration_time_ms = 0;

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
    pulse_count++;
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
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // every second

        // atomically get & clear pulse_count (ISR increments pulse_count)
        uint32_t pulses = __atomic_exchange_n(&pulse_count, 0U, __ATOMIC_ACQ_REL);

        // RPS calculation: pulses per second divided by magnets per revolution
        uint32_t rps = pulses / NUM_WHEEL_MAG;
        uint32_t rpm = rps * 60;
        rev_count += rps;

        ESP_LOGI(TAG, "Pulses: %" PRIu32 "  RPS: %" PRIu32 " RPM: %" PRIu32 " rev_count: %" PRIu32, pulses, rps, rpm, rev_count);

        // Placeholder BLE notify
        ble_notify_placeholder("rpmlvl", (int)rps);
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
    }

    last_integration_time_ms = esp_log_timestamp();

    while (1) {
        uint32_t now = esp_log_timestamp();

        int raw_batt = 0;
        int raw_curr = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_BATT_CHANNEL, &raw_batt));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CURR_HS_CHANNEL, &raw_curr));

        // Translate raw ADC (0-4095) to approximate values used in original sketch
        // Note: you should calibrate these formulas for your voltage divider and sense resistor
        float battVoltageRaw = (float)raw_batt; // similar to analogRead(A1)
        float battVoltageCorr = (battVoltageRaw * 1000.0f / 79.125f); // update formula as needed
        // Battery current calculation for bidirectional amp
        float battCurrentRaw = (float)raw_curr;
        float battCurrentmA = ((battCurrentRaw - 2048.0f) * 1000.0f / 125.0f); // 2048 is mid-scale for 12-bit ADC

        // integrate mAh
        uint32_t dt_ms = now - last_integration_time_ms;
        if (dt_ms > 0) {
            mAH_consumption += battCurrentmA * ((float)dt_ms / 3600000.0f);
            last_integration_time_ms = now;
        }

        ESP_LOGI(TAG, "battRaw=%d battCorr=%.0f mA=%.1f mAH=%.3f", raw_batt, battVoltageCorr, battCurrentmA, mAH_consumption);

        // Send periodic JSON sensor notification over BLE (TX characteristic)
    #if CONFIG_BT_NIMBLE_ENABLED
        {
            char json[128];
            int n = snprintf(json, sizeof(json), "{\"batt_mV\":%.0f,\"curr_mA\":%.1f,\"mAH\":%.3f,\"rev\":%" PRIu32 "}",
                     battVoltageCorr, battCurrentmA, mAH_consumption, (uint32_t)rev_count);
            int copy_len = n > (int)sizeof(gatt_svr_tx_val) - 1 ? (int)sizeof(gatt_svr_tx_val) - 1 : n;
            if (gatt_tx_mutex) xSemaphoreTake(gatt_tx_mutex, portMAX_DELAY);
            memcpy(gatt_svr_tx_val, json, copy_len);
            gatt_svr_tx_val[copy_len] = '\0';
            if (gatt_tx_mutex) xSemaphoreGive(gatt_tx_mutex);
            /* notify subscribed centrals */
            ble_gatts_chr_updated(gatt_svr_tx_val_handle);
        }
    #endif

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
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

    /* Advertise our 16-bit service UUID */
    static const ble_uuid16_t adv_uuids[] = { gatt_svc_uuid_struct };
    fields.uuids16 = (ble_uuid16_t *)adv_uuids;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

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

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting SkateInfoView ESP32-S3");

    /* Create resources */
    pulse_mutex = xSemaphoreCreateMutex();
    gatt_tx_mutex = xSemaphoreCreateMutex();

    /* Configure hall GPIO */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HALL_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(HALL_GPIO, hall_isr_handler, NULL);

    /* Start sensor and rpm tasks */
    xTaskCreate(rpm_task, "rpm_task", 4096, NULL, 5, NULL);
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
   

#if CONFIG_BT_NIMBLE_ENABLED
    /* Initialize NimBLE and start host task */
    ble_init();

#endif

    // Instantiate the RGB LED (example pins: 6, 7, 8)
    ILED* rgbLed = new ESP32LED(8, 7, 6);

    RGBColor color{
        .r = 0,
        .g = 255,
        .b = 255
    };

    rgbLed->setPattern(LEDPattern::Breathe, color);
}
