/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


/****************************************************************************
*
* This file is used for eddystone receiver.
*
****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"

#include "esp_eddystone_protocol.h"
#include "esp_eddystone_api.h"
const char* DEMO_TAG = "EDDYSTONE_DEMO";

#if (EDDYSTONE_FRAME_TYPE == EDDYSTONE_URL_FRAME)
extern esp_eddystone_frame_t frame_URL;
#elif (EDDYSTONE_FRAME_TYPE == EDDYSTONE_UID_FRAME)
extern esp_eddystone_frame_t frame_UID;
#endif

/* declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
static void esp_eddystone_show_inform(const esp_eddystone_result_t* res);


#if (EDDYSTONE_MODE == EDDYSTONE_RECEIVER)
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

#elif (EDDYSTONE_MODE == EDDYSTONE_SENDER)
static esp_ble_adv_params_t ble_adv_params = { 
	.adv_int_min		=0x20,
	.adv_int_max		=0x40,
	.adv_type		=ADV_TYPE_NONCONN_IND,
	.own_addr_type		=BLE_ADDR_TYPE_PUBLIC,
	.channel_map		=ADV_CHNL_ALL,
	.adv_filter_policy	=ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};
#endif


static void esp_eddystone_show_inform(const esp_eddystone_result_t* res)
{
    switch(res->common.frame_type)
    {
        case EDDYSTONE_FRAME_TYPE_UID: {
            ESP_LOGI(DEMO_TAG, "Eddystone UID inform:");
            ESP_LOGI(DEMO_TAG, "Measured power(RSSI at 0m distance):%d dbm", res->inform.uid.ranging_data);
            ESP_LOGI(DEMO_TAG, "EDDYSTONE_DEMO: Namespace ID:0x");
            esp_log_buffer_hex(DEMO_TAG, res->inform.uid.namespace_id, 10);
            ESP_LOGI(DEMO_TAG, "EDDYSTONE_DEMO: Instance ID:0x");
            esp_log_buffer_hex(DEMO_TAG, res->inform.uid.instance_id, 6);
            break;
        }
        case EDDYSTONE_FRAME_TYPE_URL: {
            ESP_LOGI(DEMO_TAG, "Eddystone URL inform:");
            ESP_LOGI(DEMO_TAG, "Measured power(RSSI at 0m distance):%d dbm", res->inform.url.tx_power);
            ESP_LOGI(DEMO_TAG, "URL: %s", res->inform.url.url);
            break;
        }
        case EDDYSTONE_FRAME_TYPE_TLM: {
            ESP_LOGI(DEMO_TAG, "Eddystone TLM inform:");
            ESP_LOGI(DEMO_TAG, "version: %d", res->inform.tlm.version);
            ESP_LOGI(DEMO_TAG, "battery voltage: %d mV", res->inform.tlm.battery_voltage);
            ESP_LOGI(DEMO_TAG, "beacon temperature in degrees Celsius: %6.1f", res->inform.tlm.temperature);
            ESP_LOGI(DEMO_TAG, "adv pdu count since power-up: %d", res->inform.tlm.adv_count);
            ESP_LOGI(DEMO_TAG, "time since power-up: %d s", (res->inform.tlm.time)/10);
            break;
        }
        default:
            break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    esp_err_t err;

    switch(event)
    {
	case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:{
#if (EDDYSTONE_MODE == EDDYSTONE_SENDER)
		esp_ble_gap_start_advertising(&ble_adv_params);
#endif
		break;
	 }
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
#if (EDDYSTONE_MODE == EDDYSTONE_RECEIVER)
            uint32_t duration = 0;
            esp_ble_gap_start_scanning(duration);
#endif
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
            if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"Scan start failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"Start scanning...");
            }
            break;
        }
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
	    if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS){
		    ESP_LOGE(DEMO_TAG, "Adv start failed: %s", esp_err_to_name(err));
	    }
	    else {
		    ESP_LOGE(DEMO_TAG, "Adv start : %s", esp_err_to_name(err));
	    }
	    break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
            switch(scan_result->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    esp_eddystone_result_t eddystone_res;
                    memset(&eddystone_res, 0, sizeof(eddystone_res));
                    esp_err_t ret = esp_eddystone_decode(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len, &eddystone_res);
                    if (ret) {
                        // error:The received data is not an eddystone frame packet or a correct eddystone frame packet.
                        // just return
                        return;
                    } else {
                        // The received adv data is a correct eddystone frame packet.
                        // Here, we get the eddystone infomation in eddystone_res, we can use the data in res to do other things.
                        // For example, just print them:
                        ESP_LOGI(DEMO_TAG, "--------Eddystone Found----------");
                        esp_log_buffer_hex("EDDYSTONE_DEMO: Device address:", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                        ESP_LOGI(DEMO_TAG, "RSSI of packet:%d dbm", scan_result->scan_rst.rssi);
                        esp_eddystone_show_inform(&eddystone_res);
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:{
            if((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"Scan stop failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"Stop scan successfully");
            }
            break;
        }
	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
	    if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
		    ESP_LOGE(DEMO_TAG, "Adv stop failed: %s", esp_err_to_name(err));
	    }
	    else {
		    ESP_LOGI(DEMO_TAG, "Stop adv successfully");
	    }
	    break;
        default:
            break;
    }
}

void esp_eddystone_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(DEMO_TAG,"Register callback");

    /*<! register the scan callback function to the gap module */
    if((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG,"gap register error: %s", esp_err_to_name(status));
        return;
    }
}

void esp_eddystone_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_eddystone_appRegister();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_eddystone_init();
#if (EDDYSTONE_MODE == EDDYSTONE_RECEIVER)
    /*<! set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);
    ESP_LOGE(DEMO_TAG, "EDDYSTONE_RECEIVER MODE\n");

#elif (EDDYSTONE_MODE == EDDYSTONE_SENDER)
    esp_eddystone_packet_t eddystone_adv_data;
    ESP_LOGE(DEMO_TAG, "EDDYSTONE_SENDER MODE\n");

/********************************************************************************************/
    esp_err_t status = esp_ble_config_eddystone_data (&frame_URL, &eddystone_adv_data);
/********************************************************************************************/

    if (status == ESP_OK){
	    ESP_LOGE(DEMO_TAG, "config eddystone data is %s\n", esp_err_to_name(status));
	    esp_ble_gap_config_adv_data_raw((uint8_t*)&eddystone_adv_data, sizeof(eddystone_adv_data));
    }
    else {
	    ESP_LOGE(DEMO_TAG, "Config eddystone data failed: %s\n", esp_err_to_name(status));
    }
#endif
}
