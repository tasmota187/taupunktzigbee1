#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sensors.h"

#include "esp_zb_core.h"
#include "esp_zb_zcl_common.h"
#include "esp_zb_zcl_on_off.h"
#include "esp_zb_cluster_list.h"
#include "esp_zb_device.h"
#include "esp_zb_endpoint.h"
#include "esp_zb_callbacks.h"

static const char *TAG = "zb_coordinator";
#define EP_SWITCH 10
#define TAU_THRESHOLD 12.0f

static uint16_t dest_short_addr = 0;
static uint8_t dest_endpoint = 0;

static void zdo_signal_handler(esp_zb_app_signal_t *sig) {
    esp_zb_app_signal_type_t type = esp_zb_app_signal_get_type(sig);
    if (type == ZB_BDB_SIGNAL_STEERING) {
        ESP_LOGI(TAG, "Network steering finished");
    }
}

static void add_onoff_control_cluster(esp_zb_ep_list_t *ep) {
    esp_zb_zcl_on_off_cfg_t cfg = {.on_off = false};
    esp_zb_attribute_list_t *cluster = esp_zb_on_off_cluster_create(&cfg);
    esp_zb_cluster_list_add_on_off_cluster(ep->cluster_list, cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

extern void display_update(float tau);

void app_main(void) {
    sensors_init();

    esp_zb_cfg_t zb_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_cfg);

    esp_zb_ep_list_t *ep = esp_zb_endpoint_create(EP_SWITCH);
    add_onoff_control_cluster(ep);

    esp_zb_device_register(ep);
    esp_zb_core_action_handler_register(zdo_signal_handler);
    esp_zb_bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
    esp_zb_start();

    while (1) {
        float T1, RH1, T2, RH2;
        if (bme_read1(&T1, &RH1) && bme_read2(&T2, &RH2)) {
            float dp1 = calc_taupunkt(T1, RH1);
            float dp2 = calc_taupunkt(T2, RH2);
            float dp = fminf(dp1, dp2);
            display_update(dp);

            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_on_off_cmd_t cmd = {
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                .zcl_basic_cmd.dst_addr_u.addr_short = dest_short_addr,
                .zcl_basic_cmd.dst_endpoint = dest_endpoint,
                .zcl_basic_cmd.src_endpoint = EP_SWITCH,
                .on_off_cmd_id = (dp >= TAU_THRESHOLD) ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID
            };
            esp_zb_zcl_on_off_cmd_req(&cmd);
            esp_zb_lock_release();
        }
        esp_zb_main_loop_iteration();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
