#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"

#define GATT_SVC_ENV_UUID 0x181A
#define GATT_PM2_5_UUID 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
#define GATT_PM10_UUID 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFE
#define GATT_HUMIDITY_UUID 0x2A6F
#define GATT_TEMPERATURE_UUID 0x2A6E

char *TAG = "Air-Purifier-BLE-Server";
uint8_t ble_addr_type;
void ble_app_advertise(void);

static TimerHandle_t tx_timer;
static bool notify_state;
static uint16_t conn_handle;

uint16_t pm2_5_handle;
uint16_t pm10_handle;
uint16_t humidity_handle;
uint16_t temperature_handle;

static uint16_t pm2_5 = 0;         // 0 kgm-3 
static uint16_t pm10 = 0;          // 0 kgm-3 
static uint16_t humidity = 0;      // 0.00%
static int16_t temperature = 300000; // 27C (300 - 273)

// Write data to ESP32 defined as server
// static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
// {
//     printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
//     return 0;
// }

// Read data from ESP32 defined as server
static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, "Data from the server", strlen("Data from the server"));
    return 0;
}

// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(GATT_SVC_ENV_UUID), // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID128_DECLARE(GATT_PM2_5_UUID),
          .flags = BLE_GATT_CHR_F_NOTIFY,
          .val_handle = &pm2_5_handle,
          .access_cb = device_read},
         {.uuid = BLE_UUID128_DECLARE(GATT_PM10_UUID),
          .flags = BLE_GATT_CHR_F_NOTIFY,
          .val_handle = &pm10_handle,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(GATT_HUMIDITY_UUID),
          .flags = BLE_GATT_CHR_F_NOTIFY,
          .val_handle = &humidity_handle,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(GATT_TEMPERATURE_UUID),
          .flags = BLE_GATT_CHR_F_NOTIFY,
          .val_handle = &temperature_handle,
          .access_cb = device_read},
         {0}}},
    {0}};

static void
tx_timer_stop(void)
{
    xTimerStop(tx_timer, 1000 / portTICK_PERIOD_MS);
}

static void
tx_timer_reset(void)
{
    int rc;

    if (xTimerReset(tx_timer, 1000 / portTICK_PERIOD_MS) == pdPASS)
    {
        rc = 0;
    }
    else
    {
        rc = 1;
    }

    assert(rc == 0);
}

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            // Connection failed; resume advertising
            ble_app_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
        ble_app_advertise();
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                          "val_handle=%d\n",
                    event->subscribe.cur_notify, pm2_5_handle);
        if (event->subscribe.attr_handle == pm2_5_handle)
        {
            notify_state = event->subscribe.cur_notify;
            tx_timer_reset();
        }
        else if (event->subscribe.attr_handle != pm2_5_handle)
        {
            notify_state = event->subscribe.cur_notify;
            tx_timer_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);

        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                          "val_handle=%d\n",
                    event->subscribe.cur_notify, pm10_handle);
        if (event->subscribe.attr_handle == pm10_handle)
        {
            notify_state = event->subscribe.cur_notify;
            tx_timer_reset();
        }
        else if (event->subscribe.attr_handle != pm10_handle)
        {
            notify_state = event->subscribe.cur_notify;
            tx_timer_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);

        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                          "val_handle=%d\n",
                    event->subscribe.cur_notify, humidity_handle);
        if (event->subscribe.attr_handle == humidity_handle)
        {
            notify_state = event->subscribe.cur_notify;
            tx_timer_reset();
        }
        else if (event->subscribe.attr_handle != humidity_handle)
        {
            notify_state = event->subscribe.cur_notify;
            tx_timer_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);

        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                          "val_handle=%d\n",
                    event->subscribe.cur_notify, temperature_handle);
        if (event->subscribe.attr_handle == temperature_handle)
        {
            notify_state = event->subscribe.cur_notify;
            tx_timer_reset();
        }
        else if (event->subscribe.attr_handle != temperature_handle)
        {
            notify_state = event->subscribe.cur_notify;
            tx_timer_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
        break;
    default:
        break;
    }
    return 0;
}

static void update_values(TimerHandle_t xTimer)
{
    static uint8_t pm2_5_bytes[2];
    static uint8_t pm10_bytes[2];
    static uint8_t humidity_bytes[2];
    static uint8_t temperature_bytes[2];
    int rc;
    struct os_mbuf *om;
    
    if (!notify_state)
    {
        tx_timer_stop();
        pm2_5 = 0;
        pm10 = 0;
        humidity = 0;
        temperature = 300000;
        return;
    }

    // Little endian
    pm2_5_bytes[0] = pm2_5 & 0xFF;
    pm2_5_bytes[1] = pm2_5 >> 8;

    pm10_bytes[0] = pm10 & 0xFF;
    pm10_bytes[1] = pm10 >> 8;

    humidity_bytes[0] = humidity & 0xFF;
    humidity_bytes[1] = humidity >> 8;

    temperature_bytes[0] = temperature & 0xFF;
    temperature_bytes[1] = temperature >> 8;

    pm2_5 += 1;
    if (pm2_5 >= 10)
    {
        pm2_5 = 0;
    }

    pm10 += 2;
    if (pm10 >= 16)
    {
        pm10 = 0;
    }

    humidity += 5;
    if (humidity >= 30)
    {
        humidity = 0;
    }

    temperature += 500;
    if (temperature >= 350000)
    {
        temperature = 300000;
    }

    om = ble_hs_mbuf_from_flat(pm2_5_bytes, sizeof(pm2_5_bytes));
    rc = ble_gatts_notify_custom(conn_handle, pm2_5_handle, om);
    assert(rc == 0);

    om = ble_hs_mbuf_from_flat(pm10_bytes, sizeof(pm10_bytes));
    rc = ble_gatts_notify_custom(conn_handle, pm10_handle, om);
    assert(rc == 0);

    om = ble_hs_mbuf_from_flat(humidity_bytes, sizeof(humidity_bytes));
    rc = ble_gatts_notify_custom(conn_handle, humidity_handle, om);
    assert(rc == 0);

    om = ble_hs_mbuf_from_flat(temperature_bytes, sizeof(temperature_bytes));
    rc = ble_gatts_notify_custom(conn_handle, temperature_handle, om);
    assert(rc == 0);

    tx_timer_reset();
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void app_main()
{
    nvs_flash_init();                                       // 1 - Initialize NVS flash using
    esp_nimble_hci_init();                                  // 2 - Initialize the host controller interface
    nimble_port_init();                                     // 3 - Initialize the host stack
    tx_timer = xTimerCreate("tx_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *) 0, update_values);
    ble_svc_gap_device_name_set("Air-Purifier-BLE-Server"); // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                                     // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                                    // 4 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);                         // 4 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);                          // 4 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;                   // 5 - Initialize application


    nimble_port_freertos_init(host_task); // 6 - Run the thread
}