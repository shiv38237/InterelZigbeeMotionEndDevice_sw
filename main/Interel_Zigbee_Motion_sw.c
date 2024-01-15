/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author:Ruchita Shekhada
 *      Date & Time:2023-11-20 && 09:40:00 
 *
 *
 *                      CONFIDENTIAL INFORMATION
 *                      ------------------------
 *      This Document contains Confidential Information or Trade Secrets,
 *      or both, which are the property of Bacancy System LLP.
 *      This document may not be copied, reproduced, reduced to any
 *      electronic medium or machine readable form or otherwise
 *      duplicated and the information herein may not be used,
 *      disseminated or otherwise disclosed, except with the prior
 *      written consent of Bacancy System LLP.
 *
 */
/*!
 * \file     Interel_Zigbee_Motion_sw.c
 * \brief   Define
 */
/************************************************************************
 * Include Header Files
 ************************************************************************/
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include <sys/time.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_attribute.h"
#include "zigbee_attribute.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/zb_zcl_basic.h"
#include "zcl/zb_zcl_power_config.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_check.h"
#include "esp_sleep.h"
#include "pyq1548.h"
#include "ltr_329Optical.h"
#include "STHS34PF80.h"
#include "i2c_service.h"
#include "esp_ota_client.h"
#include "esp_ota_ops.h"
#include "batterylevelcheck.h"
#include "NVS_sensorConfig.h"
#include "led_control.h"
#include "button_control.h"
#include "interel_zigbee_motion_sw.h"
/************************************************************************
 * Define Macros
 ************************************************************************/
#define BASIC_CLUSTER_MANUF_NAME            "INTEREL BUILDING AUTOMATION"
#define BASIC_CLUSTER_MODEL_IDENTIFIER      "MS-A03"
#define MANUFACTURE_CODE                    0x1489
#define TIMER_WAKEUP_TIME_US                (20 * 1000 * 1000) 
#define REPORTING_TIMER_10MIN               (10 * 60 * 1000 * 1000)  //1 minute
#define FACTORY_RESET_TIMER_10MIN           (1 * 60 * 1000 * 1000)  //1 minute
#define PYQ_NOT_CONFIGURED_OOR              0x3FFF
/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/
typedef struct zdo_info_ctx_s {
    uint8_t endpoint;
    uint16_t short_addr;
} zdo_info_user_ctx_t;
/************************************************************************
 * Define Private Variables
 ************************************************************************/
static const char *TAG = "INTERNAL_ZIGBEE_MOTION_SW";
static const char *ST_TAG = "timer_wakeup";
static const esp_partition_t *s_ota_partition = NULL;
static esp_ota_handle_t s_ota_handle = 0;
/************************************************************************
 * Define Global Variables
 ************************************************************************/
bool connected = false;
esp_zb_occupancy_sensing_cluster_cfig_t occupancyCluster_data;
esp_zb_temperature_meas_cluster_cfig_t TempCluster_data;
esp_zb_illuminance_meas_cluster_cfig_t illuminanceCluster_data;
esp_zb_power_config_cluster_cfig_t PowerConfigCluster_data;
esp_zb_identify_cluster_cfig_t identifyCluster_data;
esp_zb_ias_zone_cluster_cfig_t zoneCluster_data;
esp_zb_ieee_addr_t extended_pan_id;
esp_timer_handle_t pyq_mot_occupancy_timer;
esp_timer_handle_t St_mot_occupancy_timer;
esp_timer_handle_t St_pres_occupancy_timer;
esp_timer_handle_t reporting_timer;
esp_timer_handle_t factory_reset_timer;
uint8_t pyq_mot_timer_elapsed = 0;
uint8_t St_mot_timer_elapsed = 0;
uint8_t St_pres_timer_elapsed = 0;
uint8_t reporting_timer_elapsed = 0;
uint8_t check_is_coordinater_connected = 0;
uint8_t ota_start = 0;
/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/
/**
 * @name   vReport_clusters_attribute
 * @brief: report cluster attribute value at particular endpoint
*  @param[in]  Src_endpoint 
 * @param[in]  ClusterID 
 * @param[in]  AttributeID 
 * @param[in]  Cluster_role 
 * @param[out] void
 */
static void vReport_clusters_attribute(uint8_t Src_endpoint ,uint16_t ClusterID ,uint16_t AttributeID ,uint8_t Cluster_role);
/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/
/**
 * @brief : Call back to reset pyq occupancy flag
*/
void pyq_mot_timer_callback(void *arg)
{    
    pyq_mot_timer_elapsed = 1U;
}
/**
 * @brief :Call back to reset sths34pf80 motion occupancy flag
*/
void St_mot_timer_callback(void *arg)
{    
    St_mot_timer_elapsed = 1U;
}
/**
 * @brief :  Call back to reset sths34pf80 presense occupancy flag
*/
void St_pres_timer_callback(void *arg)
{    
    St_pres_timer_elapsed = 1U;
}
/**
 * @brief :  Call back to reset reporting timer flag
*/
void reporting_timer_callback(void *arg)
{    
    reporting_timer_elapsed = 1U;
}
/**
 * @brief :  Call back for factory reset timer
*/
void factory_reset_timer_callback(void *arg)
{    
    check_is_coordinater_connected = 1U;
}
esp_err_t example_register_timer_wakeup(void)
{
    ESP_RETURN_ON_ERROR(esp_sleep_enable_timer_wakeup(TIMER_WAKEUP_TIME_US), ST_TAG, "Configure timer as wakeup source failed");
    ESP_LOGI(TAG, "timer wakeup source is ready");
    return ESP_OK;
}
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}


void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    printf("zigbee signal type>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> %x \n",sig_type);
    switch (sig_type) {
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    {
        connected = false; 
    }
    case ESP_ZB_BDB_SIGNAL_STEERING:
    if (err_status != ESP_OK) {
        ESP_LOGW(TAG, "Stack %s failure with %s status, steering",esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
    } else {
        /* device auto start successfully and on a formed network */
        esp_zb_get_extended_pan_id(extended_pan_id);
        ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                    extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                    extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                    esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        connected = true;   
        // if((true == esp_timer_is_active(factory_reset_timer)))
        // {
        //     esp_timer_stop(factory_reset_timer);
        //     ESP_LOGE(TAG,"factory_reset_timer stop on connection");
        // } 
    }
    break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
    ESP_LOGI(TAG, "----------------------DEVICE LEFT--------------------");
    leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
    if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET)
    {
        ESP_LOGI(TAG, "Reset device");
        connected = false; 
        // if((true == esp_timer_is_active(factory_reset_timer)))
        // {
        //     /*Do nothing*/
        //     printf(">>>>>>>>>>>>>>>>>>>factory_reset_timer:already on>>>>>>>>>>>>>>>>>>>>>>>\n");
        // }
        // else
        // {
        //     printf("factory_reset_timer start again on disconnct : ESP_ZB_ZDO_SIGNAL_LEAVE \n");
        //     esp_timer_start_periodic(factory_reset_timer, FACTORY_RESET_TIMER_10MIN);
        // }
    }
    break;
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
    {
        ESP_LOGI(TAG, "----------------------coordinator LEFT--------------------");
        connected = false;
        // if((true == esp_timer_is_active(factory_reset_timer)))
        // {
        //     /*Do nothing*/
        //     printf(">>>>>>>>>>>>>>>>>>>factory_reset_timer:already on>>>>>>>>>>>>>>>>>>>>>>>\n");
        // }
        // else
        // {
        //     printf("factory_reset_timer start again on disconnct : ESP_ZB_ZDO_DEVICE_UNAVAILABLE \n");
        //     esp_timer_start_periodic(factory_reset_timer, FACTORY_RESET_TIMER_10MIN);
        // }
    }
    break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}
static void vReport_clusters_attribute(uint8_t Src_endpoint ,uint16_t ClusterID ,uint16_t AttributeID ,uint8_t Cluster_role)
{
    printf(" -------report attribute Src_endpoint 0X%x -------\n",Src_endpoint);
	esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = 0X01,
            .src_endpoint = Src_endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ClusterID,
        .attributeID = AttributeID ,
        .cluster_role = Cluster_role,
    };
	esp_zb_zcl_report_attr_cmd_req(&cmd);
}
/**
 * @brief: timer to set open collector on presense detection
 */
esp_err_t timers_init(void)
{
    esp_err_t ret = ESP_FAIL;
    const esp_timer_create_args_t pyq_timer_args = {
            .callback = &pyq_mot_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "one-shot-pyq"
    };
    ret = esp_timer_create(&pyq_timer_args, &pyq_mot_occupancy_timer);
    const esp_timer_create_args_t St_mot_timer_args = {
            .callback = &St_mot_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "one-shot-stMotion"
    };
    ret = esp_timer_create(&St_mot_timer_args, &St_mot_occupancy_timer);
    const esp_timer_create_args_t St_pres_timer_args = {
            .callback = &St_pres_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "one-shot-stPresense"
    };
    ret = esp_timer_create(&St_pres_timer_args, &St_pres_occupancy_timer);
    const esp_timer_create_args_t reporting_timer_args = {
            .callback = &reporting_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "periodic_reporting"
    };
    ret = esp_timer_create(&reporting_timer_args, &reporting_timer);
    return ret;
}
/**
 * @brief: initialize motion sensor's gpio
 */
esp_err_t gpioInit(void)
{
    esp_err_t err = ESP_FAIL;
    gpio_config_t io_conf = {0U};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = ((1ULL << PYQ_SUPPLY_PIN) | (1ULL << STH_SUPPLY_PIN) | (1ULL << LUX_SUPPLY_PIN) | (1ULL << OPEN_COLLECTOR_PIN));
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pull_up_en = 0U;
    io_conf.pull_down_en = 0U;
    err = gpio_config(&io_conf);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Failed to init power up sensor GPIO as Output, return Status: %d", err);
        err = ESP_FAIL;
    }
    else
    {
        gpio_set_level(STH_SUPPLY_PIN,1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(LUX_SUPPLY_PIN,1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(PYQ_SUPPLY_PIN,1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return err;
}

static esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t messsage)
{
    static uint32_t total_size = 0;
    static uint32_t offset = 0;
    static int64_t start_time = 0;
    esp_err_t ret = ESP_OK;
    if (messsage.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        switch (messsage.upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
        ota_start = 1;
            ESP_LOGI(TAG, "-- OTA upgrade start");
            start_time = esp_timer_get_time();
            s_ota_partition = esp_ota_get_next_update_partition(NULL);
            assert(s_ota_partition);
            ret = esp_ota_begin(s_ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to begin OTA partition, status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            total_size = messsage.ota_header.image_size;
            offset += messsage.payload_size;
            ESP_LOGI(TAG, "-- OTA Client receives data: progress [%ld/%ld]", offset, total_size);
            if (messsage.payload_size && messsage.payload) {
                ret = esp_ota_write(s_ota_handle, (const void *)messsage.payload, messsage.payload_size);
                ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write OTA data to partition, status: %s", esp_err_to_name(ret));
            }
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "-- OTA upgrade apply");
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = offset == total_size ? ESP_OK : ESP_FAIL;
            ESP_LOGI(TAG, "-- OTA upgrade check status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_LOGI(TAG, "-- OTA Finish");
            ESP_LOGI(TAG,
                     "-- OTA Information: version: 0x%lx, manufactor code: 0x%x, image type: 0x%x, total size: %ld bytes, cost time: %lld ms,",
                     messsage.ota_header.file_version, messsage.ota_header.manufacturer_code, messsage.ota_header.image_type,
                     messsage.ota_header.image_size, (esp_timer_get_time() - start_time) / 1000);
            ret = esp_ota_end(s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to end OTA partition, status: %s", esp_err_to_name(ret));
            ret = esp_ota_set_boot_partition(s_ota_partition);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set OTA boot partition, status: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Prepare to restart system");
            esp_restart();
            break;
        default:
            ESP_LOGI(TAG, "OTA status: %d", messsage.upgrade_status);
            break;
        }
    }
    return ret;
}
static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        if (message->info.cluster==0x406){
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d),endpoint(0x%x)", variable->status,
                 message->info.cluster, variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0,message->info.src_endpoint);
        }
        else {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d), endpoint(0x%x)", variable->status,
                 message->info.cluster, variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? *(uint16_t *)variable->attribute.data.value : 0,message->info.src_endpoint);
        }
        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        ret = zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        printf(" -------------------- read response -------------------\n");
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}
void update_attribute(void * q)
{
    // ----------------------------------------
    int8_t tx_power = 0;
    esp_zb_zcl_attr_t *occtounoccdelay = 0;
    esp_zb_zcl_attr_t *tempminValue= 0;
    esp_zb_zcl_attr_t *tempmaxValue= 0;
    esp_zb_zcl_attr_t *identify_timeval = 0;
    esp_zb_zcl_attr_t *zone_stateval = 0;
    esp_zb_zcl_attr_t *fc00_val = 0;
    esp_zb_zcl_reporting_info_t *minReportingInterval = 0;
    esp_zb_zcl_reporting_info_t *maxReportingInterval = 0;
    int16_t i32PirValue = 0;
    uint8_t u8PyqMotionStatus = false;
    uint8_t u8StMotionstatus = 0;
    uint32_t u32Ltr329ChannelValue = 0x00;
    uint16_t u16Channel0 = 0;
    uint16_t u16Channel1 = 0;
    stsensor_data_t stsensordata = {0};
    uint16_t u16BatteryPercentage = 0;
    uint16_t u16BatteryLevel = 0;
    uint8_t temp_data = 0x00;
    uint8_t ic_type = 0;
    uint8_t occupancy_temp = 0;  
    uint8_t occupancy_illuminanace = 0;  
    uint8_t was_pyqMotion_detected = 0; 
    uint8_t was_stMotion_detected = 0; 
    uint8_t was_stPresense_detected = 0; 
    esp_zb_zcl_attr_location_info_t attr_info = {0};
    uint8_t prev_occupancy = 0;
    int16_t prev_temp = 0;
    uint16_t prev_illuminance = 0;
    uint8_t prev_Batvoltage = 0;
    uint16_t occupancy_delay = 0;
    MS_vDatabaseInit();
    MS_RestoreNVSData();
    gpioInit();
    vPyqGPIO_init();
    i2c_master_init();
    LED_init();
    Button_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    vWritePyqRegval(pyq_configData.PYQconf_data); // write configuration value as 0x10
    i8Shthspf80SensorInit(sths_configData);
    i32LTR329DefaultInit(LTR_configData.lux_als_measurement_rate ,LTR_configData.lux_operating_mode_gain_setting);
    voltage_calculate_init();
    timers_init(); 
    example_register_timer_wakeup();
    printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> update_attribute >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
	while (true)
	{
        if(check_is_coordinater_connected)
        {
            printf("----------------------------- check coordinator connceted?????????????????????????????????");
            if(!connected)
            {
                esp_zb_factory_reset();
                esp_restart();
            }
            check_is_coordinater_connected = 0;
        }
        if (connected && (!ota_start)) 
        {
            #if 0
            esp_zb_get_tx_power(&tx_power);
            printf("--------tx_power----%d \n",tx_power);
           
            attr_info.endpoint_id = ZDO_ENDPOINT;
            attr_info.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
            attr_info.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
            attr_info.attr_id = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID;
            attr_info.manuf_code = MANUFACTURE_CODE;
            maxReportingInterval = esp_zb_zcl_find_reporting_info(attr_info);
            if(maxReportingInterval)
            {
                printf("--------Pointer not null_________________\n");
            }
            else
            {
                printf("--------Pointer  null_________________\n");
            }
            ESP_LOGI(" ","[maxReportingInterval] latest value is-----%d-----------\n",maxReportingInterval->manuf_code);
            #endif
            
            occtounoccdelay = esp_zb_zcl_get_attribute(HA_MOTION_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_OCC_TO_UNOCC_DELAY_ID);
            printf("[occupy to unoccuupy] latest value is-----%04X-----------\n",*(uint16_t*)occtounoccdelay->data_p);
            occupancy_delay = *(uint16_t*)occtounoccdelay->data_p;
            identify_timeval = esp_zb_zcl_get_attribute(ZDO_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID);
            printf("[identify_time] latest value is-----%04X-----------\n",*(uint16_t*)identify_timeval->data_p);
            #if 0           
            zone_stateval = esp_zb_zcl_get_attribute(HA_IAS_ZONE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATE_ID);
            printf("[Zone state] latest value is-----%04X-----------\n",*(uint8_t*)zone_stateval->data_p);
            identify_timeval = esp_zb_zcl_get_attribute(ZDO_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID);
            printf("[identify_time] latest value is-----%04X-----------\n",*(uint16_t*)identify_timeval->data_p);
            fc00_val = esp_zb_zcl_get_attribute(HA_IAS_ZONE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 0xfc00);
            printf("[fc00_val] latest value is-----%lx-----------\n",*(uint32_t*)fc00_val->data_p);
            tempmaxValue = esp_zb_zcl_get_attribute(HA_MOTION_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_OCC_TO_UNOCC_DELAY_ID);
            printf("[tempmaxValue] latest value is-----%04X-----------\n",*(uint16_t*)occtounoccdelay->data_p);
            tempminValue = esp_zb_zcl_get_attribute(HA_MOTION_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_OCC_TO_UNOCC_DELAY_ID);
            printf("[tempminValue] latest value is-----%04X-----------\n",*(uint16_t*)occtounoccdelay->data_p);
            lastState = esp_zb_zcl_get_attribute(0x01, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID) ;
            printf("[display_attribute]----------letest value is-----%04X-----------\n",*(uint8_t*)lastState->data_p);
            #endif
/*--------------------------------------get sensors data-----------------------------------------------------------*/
            i32PirValue = i32ReadLowPowerPyro();// function to read the configuration and PIR data
            if(((i32PirValue < pyq_configData.pyq_lower_thresold) || (i32PirValue > pyq_configData.pyq_upper_thresold))
            && ( PYQ_NOT_CONFIGURED_OOR != i32PirValue))
            {
                u8PyqMotionStatus = true;
                printf("PYQ Motion detected: motion row value :[%d]\n",i32PirValue);              
            } 
            else 
            {
                u8PyqMotionStatus = false; 
                printf("NO PYQ motion\n");
            }
            // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            LTR_configData.lux_operating_mode_gain_setting = 0x1D;   //active mode
            LTR329_set_op_mode_Gain(LTR_configData.lux_operating_mode_gain_setting);
            if (i32GetLTR329ChannelData(&u32Ltr329ChannelValue) == 0x43)
            {
                printf("-------------- LTR329 Read Timeout ----------------\n");
            } else {
                u16Channel0 = (u32Ltr329ChannelValue >> 16) & 0xFFFF;
                u16Channel1 = u32Ltr329ChannelValue & 0xFFFF;
                printf("HEX: [%8X] Channel 0 : [%d]  Channel 1 : [%d]\n",(int)u32Ltr329ChannelValue,u16Channel0,u16Channel1);
                LTR329_set_standby_mode();
            }
            illuminanceCluster_data.measured_value = u16Channel0; //10^((u16Channel0-1)/10000); //u16Channel0
            printf("converted lux value %d \n",illuminanceCluster_data.measured_value);
            // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            u8StMotionstatus = u8GetMotionPresenceValue(&stsensordata);
            if(u8StMotionstatus != 0)
            {
                printf("St Data read success: Motion: [%d] Presense : [%d] \n",stsensordata.i16Motion,stsensordata.i16Presence);
                printf("St Motion flag: [%d] Presense flag: [%d] \n",stsensordata.u8MotionFlag,stsensordata.u8PresenceFlag);
            } else {
                printf("St Data read fail \n");
            }
            TempCluster_data.measured_value = (int16_t) (stsensordata.i16AmbTemp/10);
            printf("TempCluster_data.measured_value  %d \n", TempCluster_data.measured_value);

/*----------------------------logic for occupacncy attribut-----------------------------------------------------*/
            if(true == u8PyqMotionStatus )
            {
                MODIFY_BIT(occupancyCluster_data.occupancy,0,u8PyqMotionStatus);
                was_pyqMotion_detected = 1;
            }
            else
            {
                if(was_pyqMotion_detected)
                {
                    if((true == esp_timer_is_active(pyq_mot_occupancy_timer)))
                    {
                        /*Do nothing*/
                        printf(">>>>>>>>>>>>>>>>>>>pyq_mot_occupancy_timer:already on>>>>>>>>>>>>>>>>>>>>>>>\n");
                    }
                    else
                    {
                        printf("pyq_mot_occupancy timer on once only and open collector gpio high\n");
                        esp_timer_start_once(pyq_mot_occupancy_timer, (occupancy_delay *TIME_SECOND_TO_MICROSECOND));
                    }
                }
            }
            if(pyq_mot_timer_elapsed)
            {
                if(true == u8PyqMotionStatus )
                {
                    /*do nothing*/
                }
                else
                {
                    MODIFY_BIT(occupancyCluster_data.occupancy,0,u8PyqMotionStatus);
                    was_pyqMotion_detected = 0;
                }
            }
            if(1U == stsensordata.u8MotionFlag)
            {
                MODIFY_BIT(occupancyCluster_data.occupancy,1,stsensordata.u8MotionFlag);
                was_stMotion_detected = 1;
            }
            else
            {
                if(was_stMotion_detected)
                {
                    if((true == esp_timer_is_active(St_mot_occupancy_timer)))
                    {
                        /*Do nothing*/
                        printf(">>>>>>>>>>>>>>>>>>>stsensor motion timer:already on>>>>>>>>>>>>>>>>>>>>>>>\n");
                    }
                    else
                    {
                        printf("stsensor motion timer timer on once only and open collector gpio high\n");
                        esp_timer_start_once(St_mot_occupancy_timer, (occupancy_delay *TIME_SECOND_TO_MICROSECOND));
                    }
                }
            }
            if(St_mot_timer_elapsed)
            {
                if(1U == stsensordata.u8MotionFlag)
                {
                    /*do nothing*/
                }
                else
                {
                    MODIFY_BIT(occupancyCluster_data.occupancy,1,stsensordata.u8MotionFlag);
                    was_stMotion_detected = 0;
                }
            }
            if(1U == stsensordata.u8PresenceFlag)
            {
                MODIFY_BIT(occupancyCluster_data.occupancy,2,stsensordata.u8PresenceFlag);
                was_stPresense_detected = 1;
            }
            else
            {
                if(was_stPresense_detected)
                {
                    if((true == esp_timer_is_active(St_pres_occupancy_timer)))
                    {
                        /*Do nothing*/
                        printf(">>>>>>>>>>>>>>>>>>>stsensor presense timer:already on>>>>>>>>>>>>>>>>>>>>>>>\n");
                    }
                    else
                    {
                        printf("stsensor presense timer timer on once only and open collector gpio high\n");
                        esp_timer_start_once(St_pres_occupancy_timer, (occupancy_delay *TIME_SECOND_TO_MICROSECOND));
                    }
                }
            }
            if(St_pres_timer_elapsed)
            {
                if(1U == stsensordata.u8PresenceFlag)
                {
                    /*do nothing*/
                }
                else
                {
                    MODIFY_BIT(occupancyCluster_data.occupancy,2,stsensordata.u8PresenceFlag);
                    St_pres_timer_elapsed = 0;
                }
            }
                if((1U == stsensordata.u8PresenceFlag) || (1U == stsensordata.u8MotionFlag)
                || (true == u8PyqMotionStatus ))
            { 
                printf("sths_configData.ambient_temp_upper_thresold %d \n",sths_configData.ambient_temp_upper_thresold * 100);
                printf("sths_configData.ambient_temp_lower_thresold %d \n",sths_configData.ambient_temp_lower_thresold * 100);
                    if((stsensordata.i16AmbTemp > (sths_configData.ambient_temp_upper_thresold * 100)) || 
                (stsensordata.i16AmbTemp < (sths_configData.ambient_temp_lower_thresold * 100)))
                {
                    printf("inside occupancy + temp Condition>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
                    occupancy_temp = 1;
                }
                else
                {
                    if(occupancy_temp)
                    {
                        occupancy_temp = 2; /*to update attribute after set to reset condition*/
                    }
                }
                printf("LTR_configData.lux_level_upper_thresold  %d u16Channel0 %d\n",LTR_configData.lux_level_upper_thresold,u16Channel0);
                if(u16Channel0 > LTR_configData.lux_level_upper_thresold) /*what should targetLevel */
                {
                    occupancy_illuminanace = 1;
                    printf("inside occupancy + illuminance Condition>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
                }
                else
                {
                    if(occupancy_illuminanace)
                    {
                        occupancy_illuminanace = 2; /*to update attribute after set to reset condition*/
                    }
                }
            }
/*--------------------------------------------battery level read-----------------------------------------------*/
            u16BatteryLevel = (uint16_t)get_battery_level();
            ESP_LOGI(TAG, "Battery level: [%d] mV\n",u16BatteryLevel);
            PowerConfigCluster_data.battery_voltage = (uint8_t)(u16BatteryLevel/100);
#if 1
            if(occupancyCluster_data.occupancy != prev_occupancy)
            {
                prev_occupancy = occupancyCluster_data.occupancy;
            /* Write new occupancy value */
                esp_zb_zcl_status_t state_occupancy = esp_zb_zcl_set_attribute_val(
                HA_MOTION_SENSOR_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
                &occupancyCluster_data.occupancy,
                false);
                vReport_clusters_attribute(HA_MOTION_SENSOR_ENDPOINT,ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        
            /* Write new occupancy with temperature value */
                esp_zb_zcl_status_t state_occupancy_with_temperature = esp_zb_zcl_set_attribute_val(
                HA_MOTION_SENSOR_WITH_TEMP_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
                &occupancyCluster_data.occupancy,
                false);
                if(1U == occupancy_temp || 2U == occupancy_temp )
                {
                    vReport_clusters_attribute(HA_MOTION_SENSOR_WITH_TEMP_ENDPOINT,ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
                    if(2U == occupancy_temp)
                    {
                        occupancy_temp = 0;
                    }
                }
            /* Write new occupancy with illuminance value */
                esp_zb_zcl_status_t state_occupancy_with_illuminance = esp_zb_zcl_set_attribute_val(
                HA_MOTION_SENSOR_WITH_LIGHT_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
                &occupancyCluster_data.occupancy,
                false);
                if(1U == occupancy_illuminanace || 2U == occupancy_illuminanace)
                {
                    vReport_clusters_attribute(HA_MOTION_SENSOR_WITH_LIGHT_ENDPOINT,ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
                    if(2U == occupancy_illuminanace)
                    {
                        occupancy_illuminanace = 0;
                    }
                }
            }
            else if(prev_temp != TempCluster_data.measured_value)
            {
                prev_temp = TempCluster_data.measured_value;
                /* Write new temperature value */
                esp_zb_zcl_status_t state_temperature = esp_zb_zcl_set_attribute_val(
                HA_TEMP_SENSOR_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                &TempCluster_data.measured_value,
                false);
                vReport_clusters_attribute(HA_TEMP_SENSOR_ENDPOINT,ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            }
            else if(prev_illuminance != illuminanceCluster_data.measured_value)
            {
                prev_illuminance = illuminanceCluster_data.measured_value;
                /* Write new illuminance value */
                esp_zb_zcl_status_t state_illuminance = esp_zb_zcl_set_attribute_val(
                HA_LIGHT_SENSOR_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
                &illuminanceCluster_data.measured_value,
                false);
                vReport_clusters_attribute(HA_LIGHT_SENSOR_ENDPOINT ,ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT ,ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            }
            else if(prev_Batvoltage != PowerConfigCluster_data.battery_voltage)
            {
                prev_Batvoltage = PowerConfigCluster_data.battery_voltage;
                /* Write new voltage value */
                // esp_zb_zcl_status_t state_voltage = esp_zb_zcl_set_attribute_val(
                // ZDO_ENDPOINT,
                // ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                // ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                // ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                // &PowerConfigCluster_data.battery_voltage,
                // false);
                // vReport_clusters_attribute(ZDO_ENDPOINT ,ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG ,ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            }
            else
            {
                /*report zone state*/
                // esp_zb_zcl_status_t IASzone_state = esp_zb_zcl_set_attribute_val(
                // HA_IAS_ZONE_ENDPOINT,
                // ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE,
                // ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                // ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATE_ID,
                // &zoneCluster_data.zone_state,
                // false);
                // vReport_clusters_attribute(HA_IAS_ZONE_ENDPOINT ,ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE ,ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATE_ID,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
                if((true == esp_timer_is_active(reporting_timer)))
                {
                    /*Do nothing*/
                }
                else
                {
                    esp_timer_start_periodic(reporting_timer, REPORTING_TIMER_10MIN);
                    printf("------------------periodic timer start------------------\n");
                }
                if(reporting_timer_elapsed)
                {
                    printf("------------------periodic timer elapsed------------------\n");
                    vReport_clusters_attribute(HA_TEMP_SENSOR_ENDPOINT,ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
                    vReport_clusters_attribute(HA_LIGHT_SENSOR_ENDPOINT ,ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT ,ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
                    reporting_timer_elapsed = 0;
                }
            }
#endif
               
                #if 0
                int64_t t_before_us = esp_timer_get_time();
                /* Enter sleep mode */
                esp_light_sleep_start();
                /* Get timestamp after waking up from sleep */
                int64_t t_after_us = esp_timer_get_time();
                const char* wakeup_reason;
                switch (esp_sleep_get_wakeup_cause()) {
                    case ESP_SLEEP_WAKEUP_TIMER:
                        wakeup_reason = "timer";
                        break;
                    case ESP_SLEEP_WAKEUP_GPIO:
                        wakeup_reason = "pin";
                        break;
                    case ESP_SLEEP_WAKEUP_UART:
                        wakeup_reason = "uart";
                        /* Hang-up for a while to switch and execuse the uart task
                            * Otherwise the chip may fall sleep again before running uart task */
                        vTaskDelay(1);
                        break;
                    default:
                        wakeup_reason = "other";
                        break;
                }
                printf("Returned from light sleep, reason: %s, t=%lld ms, slept for %lld ms\n",
                wakeup_reason, t_after_us / 1000, (t_after_us - t_before_us) / 1000);
                #endif
        }
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}
static void commissioning_task(void *pvParameters)
{
    uint32_t io_num;
    while(true)
    {
        if (NULL != gpio_evt_queue)
		{
            if (pdTRUE == xQueueReceive(gpio_evt_queue, &io_num, (10 / portTICK_RATE_MS)))
            {
                printf("buttone queue receive\n");
                vTaskDelay(300 / portTICK_PERIOD_MS);
                if(!gpio_get_level(io_num))
                {
                    printf("buttone pressed >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
                    led_gpio_set_level(ZB_TRUE);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    led_gpio_set_level(ZB_FALSE);
                    esp_zb_factory_reset();
                } else {
                    printf("buttone  released >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
                    //led_gpio_set_level(ZB_FALSE);
                }
            }
            else{

            }
        }
        vTaskDelay(5000/ portTICK_PERIOD_MS);
    }
}
static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_zcl_attr_t *lastStatede = 0;
    uint8_t factory_reset = 0;
    uint8_t test_attr;
    uint32_t test_attr1;
    test_attr = 0x01;
    test_attr1 = 0x10;
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    /*----------------------------------basic cluster attr list-------------------------------------------------*/
    esp_zb_basic_cluster_cfig_t BasicCluster_default_value = {
    .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    .manufacturer_name = {BASIC_CLUSTER_MANUF_NAME},
    .model_identifier = {BASIC_CLUSTER_MODEL_IDENTIFIER},
    .datecode = ESP_ZB_ZCL_BASIC_DATE_CODE_DEFAULT_VALUE, 
    .powersource = ZB_ZCL_BASIC_POWER_SOURCE_BATTERY                    
    };
    /* create cluster list and attribute list for basic cluster*/
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &BasicCluster_default_value.zcl_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, BasicCluster_default_value.manufacturer_name);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, BasicCluster_default_value.model_identifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, BasicCluster_default_value.datecode);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &BasicCluster_default_value.powersource);
    
    /*----------------------------------identify cluster attr list-------------------------------------------------*/
    esp_zb_identify_cluster_cfig_t IdentifyCluster_default_value = {
        .identify_time = 0
    };
    /* create identify cluster*/
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &IdentifyCluster_default_value.identify_time);

    /*----------------------------------time cluster attr list-------------------------------------------------*/
    int32_t utc_time = 0;
    esp_zb_time_cluster_cfig_t TimeCluster_default_value = {
       .time = 0, 
       .time_status = 0
    };
    /* create time cluster*/
    esp_zb_attribute_list_t *esp_zb_time_clustertime_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);
    esp_zb_time_cluster_add_attr(esp_zb_time_clustertime_cluster,ESP_ZB_ZCL_ATTR_TIME_TIME_ID,&TimeCluster_default_value.time);
    esp_zb_time_cluster_add_attr(esp_zb_time_clustertime_cluster,ESP_ZB_ZCL_ATTR_TIME_TIME_STATUS_ID,&TimeCluster_default_value.time_status);

    /*----------------------------------power config cluster attr list-------------------------------------------------*/
    esp_zb_power_config_cluster_cfig_t PowConfigCluster_default_value =  {
    .battery_voltage = 33,                     
    .battery_size = ZB_ZCL_POWER_CONFIG_BATTERY_SIZE_DEFAULT_VALUE,                        
    .battery_quantity = 1,                 
    .battery_rated_voltage = 33,                  
    .battery_voltage_min_threshold = 29,        
    .battery_alarm_state = 0              
    };
   /*create power configuration cluster attributes list*/
    esp_zb_attribute_list_t *esp_zb_power_configuration_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    esp_zb_power_config_cluster_add_attr(esp_zb_power_configuration_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &PowConfigCluster_default_value.battery_voltage);
    esp_zb_power_config_cluster_add_attr(esp_zb_power_configuration_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, &PowConfigCluster_default_value.battery_size);
    esp_zb_power_config_cluster_add_attr(esp_zb_power_configuration_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID, &PowConfigCluster_default_value.battery_quantity);
    esp_zb_power_config_cluster_add_attr(esp_zb_power_configuration_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_RATED_VOLTAGE_ID, &PowConfigCluster_default_value.battery_rated_voltage);
    esp_zb_power_config_cluster_add_attr(esp_zb_power_configuration_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_MIN_THRESHOLD_ID, &PowConfigCluster_default_value.battery_voltage_min_threshold);
    esp_zb_power_config_cluster_add_attr(esp_zb_power_configuration_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_STATE_ID, &PowConfigCluster_default_value.battery_alarm_state);


    // /* create cluster lists for zdo endpoint for mendatory clusters*/
    esp_zb_cluster_list_t *esp_zb_cluster_list0 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list0, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list0, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list0, esp_zb_power_configuration_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_time_cluster(esp_zb_cluster_list0,esp_zb_time_clustertime_cluster,ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    /*----------------------------------occupancy cluster attr list-------------------------------------------------*/
    esp_zb_occupancy_sensing_cluster_cfig_t occupancy_default_value = {
    .occupancy = 0x02,
    .sensor_type = 0x01,
    .occupied_to_unoccupied_delay = 5, //ESP_ZB_ZCL_OCCUPANCY_SENSING_PIR_OCC_TO_UNOCC_DELAY_MIN_VALUE,
    .unoccupied_to_occupied_delay = ESP_ZB_ZCL_OCCUPANCY_SENSING_PIR_UNOCC_TO_OCC_DELAY_DEFAULT_VALUE,
    .unoccupied_to_occupied_threshold = ESP_ZB_ZCL_OCCUPANCY_SENSING_PIR_UNOCC_TO_OCC_THRESHOLD_DEFAULT_VALUE
    };
    /* create attribute list for occupancy sensing pir sensor */
    esp_zb_attribute_list_t *esp_zb_ocuupancy_sensing_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &occupancy_default_value.occupancy);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ID, &occupancy_default_value.sensor_type);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_OCC_TO_UNOCC_DELAY_ID, &occupancy_default_value.occupied_to_unoccupied_delay);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_UNOCC_TO_OCC_DELAY_ID, &occupancy_default_value.unoccupied_to_occupied_delay);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_UNOCC_TO_OCC_THRESHOLD_ID, &occupancy_default_value.unoccupied_to_occupied_threshold);
    
   /* create cluster lists for occupancy sensing pir sensor endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_occupancy_sensing_cluster(esp_zb_cluster_list, esp_zb_ocuupancy_sensing_client_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* create attribute list for the cluster occupancy with temperature */
    esp_zb_attribute_list_t *esp_zb_ocuupancy_sensing_temperature_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_temperature_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &occupancy_default_value.occupancy);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_temperature_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ID, &occupancy_default_value.sensor_type);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_temperature_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_OCC_TO_UNOCC_DELAY_ID, &occupancy_default_value.occupied_to_unoccupied_delay);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_temperature_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_UNOCC_TO_OCC_DELAY_ID, &occupancy_default_value.unoccupied_to_occupied_delay);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_temperature_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_UNOCC_TO_OCC_THRESHOLD_ID, &occupancy_default_value.unoccupied_to_occupied_threshold);

    /* create cluster lists for occupancy with temperature logic endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list1_1 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_occupancy_sensing_cluster(esp_zb_cluster_list1_1, esp_zb_ocuupancy_sensing_temperature_client_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* create attribute list for  occupancy with illuminance(light) */
    esp_zb_attribute_list_t *esp_zb_ocuupancy_sensing_illuminance_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_illuminance_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &occupancy_default_value.occupancy);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_illuminance_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ID, &occupancy_default_value.sensor_type);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_illuminance_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_OCC_TO_UNOCC_DELAY_ID, &occupancy_default_value.occupied_to_unoccupied_delay);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_illuminance_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_UNOCC_TO_OCC_DELAY_ID, &occupancy_default_value.unoccupied_to_occupied_delay);
    esp_zb_occupancy_sensing_cluster_add_attr(esp_zb_ocuupancy_sensing_illuminance_client_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_UNOCC_TO_OCC_THRESHOLD_ID,&occupancy_default_value.unoccupied_to_occupied_threshold);

    /* create cluster lists for occupancy with light logic endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list1_2 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_occupancy_sensing_cluster(esp_zb_cluster_list1_2, esp_zb_ocuupancy_sensing_illuminance_client_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /*----------------------------------temperature cluster attr list-------------------------------------------------*/
    esp_zb_temperature_meas_cluster_cfig_t TempCluster_default_value = 
    {
        .measured_value = 0,                  
        .min_measured_value =  0,                      
        .max_measured_value = 5000 
    };
    /* create attribute lists for temperature sensor endpoint */
    esp_zb_attribute_list_t * esp_zb_temperature_meas_cluster= esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
	esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster,ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,&TempCluster_default_value.measured_value);
	esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster,ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID,&TempCluster_default_value.min_measured_value);
	esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_meas_cluster,ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID,&TempCluster_default_value.max_measured_value);
    /* create cluster lists for temperature sensor endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list2 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list2, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /*----------------------------------Illuminance cluster attr list-------------------------------------------------*/
    esp_zb_illuminance_meas_cluster_cfig_t IlluminanceCluster_default_value = {
        .measured_value = 0,                    
        .min_measured_value = 0x12A4,                        
        .max_measured_value =  0xBD43,                         
        .light_sensor_type = 0
    };
    esp_zb_attribute_list_t * esp_zb_illuminance_meas_cluster= esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT);
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster,ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,&IlluminanceCluster_default_value.measured_value);
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster,ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MIN_MEASURED_VALUE_ID ,&IlluminanceCluster_default_value.min_measured_value);
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster,ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MAX_MEASURED_VALUE_ID,&IlluminanceCluster_default_value.max_measured_value);
    esp_zb_illuminance_meas_cluster_add_attr(esp_zb_illuminance_meas_cluster,ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_LIGHT_SENSOR_TYPE_ID,&IlluminanceCluster_default_value.light_sensor_type);
    /* create cluster lists for light sensor endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list3 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_illuminance_meas_cluster(esp_zb_cluster_list3, esp_zb_illuminance_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
   
    /*----------------------------------IAS zone cluster attr list-------------------------------------------------*/
    esp_zb_get_extended_pan_id(extended_pan_id);
    esp_zb_ias_zone_cluster_cfig_t IASCluster_default_value=  {
    .zone_state =  0x00,                          
    .zone_type =   0x000D,                           
    .zone_status = 0,                                          
    .zone_id = 0                               
    };
    memcpy(&IASCluster_default_value.ias_cie_Address,&extended_pan_id,sizeof(esp_zb_ieee_addr_t));
    /* create attribute lists for IAS sensor endpoint */
    esp_zb_attribute_list_t * esp_zb_ias_zone_cluster= esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE);
	esp_zb_ias_zone_cluster_add_attr(esp_zb_ias_zone_cluster,ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATE_ID,&IASCluster_default_value.zone_state);
    esp_zb_ias_zone_cluster_add_attr(esp_zb_ias_zone_cluster,ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONETYPE_ID,&IASCluster_default_value.zone_type);
	esp_zb_ias_zone_cluster_add_attr(esp_zb_ias_zone_cluster,ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATUS_ID,&IASCluster_default_value.zone_status);
    esp_zb_ias_zone_cluster_add_attr(esp_zb_ias_zone_cluster,ESP_ZB_ZCL_ATTR_IAS_ZONE_IAS_CIE_ADDRESS_ID,&IASCluster_default_value.ias_cie_Address);
	esp_zb_ias_zone_cluster_add_attr(esp_zb_ias_zone_cluster,ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONEID_ID,&IASCluster_default_value.zone_id);
    //esp_zb_cluster_add_attr(esp_zb_ias_zone_cluster,ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE,0xfc00,0x23,0x07,&test_attr1);
    // /* create cluster lists for IAS sensor endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list4 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_ias_zone_cluster(esp_zb_cluster_list4, esp_zb_ias_zone_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
   
    /*----------------------------------OTA attr list-------------------------------------------------*/
    /** Create ota client cluster with attributes.
     *  Manufacturer code, image type and file version should match with configured values for server.
     *  If the client values do not match with configured values then it shall discard the command and
     *  no further processing shall continue.
     */
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        .ota_upgrade_file_version = OTA_UPGRADE_RUNNING_FILE_VERSION,
        .ota_upgrade_downloaded_file_ver = OTA_UPGRADE_DOWNLOADED_FILE_VERSION,
        .ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
        .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
    };
    esp_zb_attribute_list_t *esp_zb_ota_client_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    /** add client parameters to ota client cluster */
    esp_zb_zcl_ota_upgrade_client_variable_t variable_config = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = OTA_UPGRADE_HW_VERSION,
        .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,
    };
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, (void *)&variable_config);
    /* create cluster list with ota cluster */
    esp_zb_cluster_list_t *esp_zb_ota_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_ota_cluster(esp_zb_ota_cluster_list, esp_zb_ota_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    /* add created endpoint (cluster_list) to endpoint list */

    /*---------------------------- create endpoint list endpoints-------------------------------------------------*/
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list0, ZDO_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, 0x0107); //ruchita
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, HA_MOTION_SENSOR_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID,  0x0107);
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list2, HA_TEMP_SENSOR_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, 0x0302);
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list3, HA_LIGHT_SENSOR_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, 0x0106);
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list4, HA_IAS_ZONE_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, 0x0402);
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list1_1, HA_MOTION_SENSOR_WITH_TEMP_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, 0x0107);
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list1_2, HA_MOTION_SENSOR_WITH_LIGHT_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, 0x0107);
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_ota_cluster_list, ESP_OTA_CLIENT_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_TEST_DEVICE_ID);

    /* register all endpoints */
    esp_zb_device_register(esp_zb_ep_list);
    uint8_t key[16] = {0xc1, 0x01, 0x09, 0x56, 0xbe, 0x74, 0x7a, 0xb9, 0xf1, 0x50, 0xed, 0x5f, 0x82, 0x8b, 0x2c, 0xc0};
    esp_zb_secur_TC_standard_preconfigure_key_set(&key);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    esp_zb_set_tx_power(20);
    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_main_loop_iteration();
}
void app_main(void)
{
    esp_err_t ret = ESP_FAIL;
    QueueHandle_t queue = xQueueCreate(1, sizeof(5));
    int8_t tx_power = 0;
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    const esp_timer_create_args_t factory_reset_timer_args = {
            .callback = &factory_reset_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "periodic_factoryreset"
    };
    ret = esp_timer_create(&factory_reset_timer_args, &factory_reset_timer);
    esp_timer_start_periodic(factory_reset_timer, FACTORY_RESET_TIMER_10MIN);
    printf("factory reset timer start\n");
    xTaskCreate(update_attribute, "update_attr", 6144, queue, 4, NULL);
    xTaskCreate(commissioning_task, "commissioning_switch", 2048, queue, 3, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    
}
