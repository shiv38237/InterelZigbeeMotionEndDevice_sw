/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: Ruchita Shekhada
 *      Date & Time:
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
 * \file     NVS_sensorConfig.c
 * \brief
 *
 */

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include "esp_err.h"
#include "rom/gpio.h"
#include <esp_system.h>
#include "esp_log.h"            // for log_write
#include "nvs_flash.h"
#include "nvs.h"
#include "NVS_sensorConfig.h"
#include "STHS34PF80.h"
#include "ltr_329Optical.h"
#include "pyq1548.h"
/************************************************************************
 * Define Macros
 ************************************************************************/
#define TAG  "[NVS_sensorConfig]:"

#define STORAGE_NAMESPACE       	    "nvs"	/*!< Storage Name Key */
#define LTR329_SENSOR_CONFIG_KEY        "ltrconfigkey"
#define PYQ1548_SENSOR_CONFIG_KEY       "pyqconfigkey"
#define STHS34PF80_SENSOR_CONFIG_KEY    "sthsconfigkey"
#define INVALID_NVS_HANDLER             (0U)	/*!< Invalid NVS Handler */
#define CONFIGURE_BYTES                 (56)	/*!< Invalid NVS Handler */
/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/



/************************************************************************
 * Define Private Variables
 ************************************************************************/

/************************************************************************
 * Define Global Variables
 ************************************************************************/
nvs_handle_t  nvsHandler = INVALID_NVS_HANDLER;					/*!< NVS Handler */
/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/
/**
 * @brief: API to Read Configuration data of LTR329 sensor from NVS
 */ 
uint8_t u8Ltr329ConfigGet(LTR329_config_details_t *LTRDataRead)
{
    uint8_t err = NVS_ERR_NONE;
    if ((NULL != LTRDataRead) && (INVALID_NVS_HANDLER != nvsHandler))
    {
        size_t required_size = 0;
        err = nvs_get_blob(nvsHandler, LTR329_SENSOR_CONFIG_KEY, NULL, &required_size);
        if (NVS_ERR_NONE == err)
        {
            if (required_size > 0)
            {
                size_t DBConfigSize = sizeof(LTR329_config_details_t);
                err = nvs_get_blob(nvsHandler, LTR329_SENSOR_CONFIG_KEY, LTRDataRead, &DBConfigSize);
                if (NVS_ERR_NONE != err)
                {
                    ESP_LOGE(TAG,"Failed to get ltr329 last blob data from nvs, Error = %d",err);
                }
            }
        }
    }

    return err;
}
/**
 * @brief: API to Write LTR329 sensor data into NVS
 */
uint8_t u8Ltr329ConfigSet(LTR329_config_details_t LTRDataWrite)
{
	uint8_t err = NVS_ERR_NONE;
    if (INVALID_NVS_HANDLER != nvsHandler)
    {		
        err = nvs_set_blob(nvsHandler, LTR329_SENSOR_CONFIG_KEY, &LTRDataWrite, sizeof(LTR329_config_details_t));
        if (NVS_ERR_NONE != err)
        {
            ESP_LOGE(TAG,"Failed to set ltr329 last blob data from nvs, Error = %d",err);
        }
        else
        {
            err = nvs_commit(nvsHandler);
        }
    }
    return err;
}
/**
 * @brief: API to Read Configuration data of Sths34pf80 sensor from NVS
 */ 
uint8_t u8Sths34pf80ConfigGet(sths_config_details_t *Sths34pf80DataRead)
{
    uint8_t err = NVS_ERR_NONE;
    if ((NULL != Sths34pf80DataRead) && (INVALID_NVS_HANDLER != nvsHandler))
    {
        size_t required_size = 0;
        err = nvs_get_blob(nvsHandler, STHS34PF80_SENSOR_CONFIG_KEY, NULL, &required_size);
        if (NVS_ERR_NONE == err)
        {
            if (required_size > 0)
            {
                size_t DBConfigSize = sizeof(sths_config_details_t);
                err = nvs_get_blob(nvsHandler, STHS34PF80_SENSOR_CONFIG_KEY, Sths34pf80DataRead, &DBConfigSize);
                if (NVS_ERR_NONE != err)
                {
                    ESP_LOGE(TAG,"Failed to get sths34pf80 last blob data from nvs, Error = %d",err);
                }
            }
        }
    }

    return err;
}
/**
 * @brief: API to Write Sths34pf80 sensor data into NVS
 */
uint8_t u8Sths34pf80ConfigSet(sths_config_details_t Sths34pf80DataWrite)
{
	uint8_t err = NVS_ERR_NONE;
    if (INVALID_NVS_HANDLER != nvsHandler)
    {		
        err = nvs_set_blob(nvsHandler, STHS34PF80_SENSOR_CONFIG_KEY, &Sths34pf80DataWrite, sizeof(sths_config_details_t));
        if (NVS_ERR_NONE != err)
        {
            ESP_LOGE(TAG,"Failed to set sths340f80 last blob data from nvs, Error = %d",err);
        }
        else
        {
            err = nvs_commit(nvsHandler);
        }
    }
    return err;
}
/**
 * @brief: API to Read Configuration data of PYQ1548 sensor from NVS
 */ 
uint8_t u8Pyq1548ConfigGet(PYQ1548_config_details_t *PYQDataRead)
{
    uint8_t err = NVS_ERR_NONE;
    if ((NULL != PYQDataRead) && (INVALID_NVS_HANDLER != nvsHandler))
    {
        size_t required_size = 0;
        err = nvs_get_blob(nvsHandler, PYQ1548_SENSOR_CONFIG_KEY, NULL, &required_size);
        if (NVS_ERR_NONE == err)
        {
            if (required_size > 0)
            {
                size_t DBConfigSize = sizeof(PYQ1548_config_details_t);
                err = nvs_get_blob(nvsHandler, PYQ1548_SENSOR_CONFIG_KEY, PYQDataRead, &DBConfigSize);
                if (NVS_ERR_NONE != err)
                {
                    ESP_LOGE(TAG,"Failed to get PYQ1548 last blob data from nvs, Error = %d",err);
                }
            }
        }
    }

    return err;
}
/**
 * @brief: API to Write PYQ1548 sensor data into NVS
 */
uint8_t u8Pyq1548ConfigSet(PYQ1548_config_details_t PyqDataWrite)
{
	uint8_t err = NVS_ERR_NONE;
    if (INVALID_NVS_HANDLER != nvsHandler)
    {		
        err = nvs_set_blob(nvsHandler, PYQ1548_SENSOR_CONFIG_KEY, &PyqDataWrite, sizeof(PYQ1548_config_details_t));
        if (NVS_ERR_NONE != err)
        {
            ESP_LOGE(TAG,"Failed to set PYQ1548 last blob data from nvs, Error = %d",err);
        }
        else
        {
            err = nvs_commit(nvsHandler);
        }
    }
    return err;
}
/**
 * @brief: API to restore data from nvs storage
 * 		   and fill respective structure
 */
uint8_t MS_RestoreNVSData(void)
{
    uint8_t err = NVS_ERR_NONE;
    err = u8Ltr329ConfigGet(&LTR_configData);
    err = u8Sths34pf80ConfigGet(&sths_configData);
    err = u8Pyq1548ConfigGet(&pyq_configData);
    if (NVS_ERR_NONE != err)
    {
        ESP_LOGE(TAG,"Failed to read configured data from nvs, return Status: %d", err);
        /*write default data in nvs*/
        err = u8Ltr329ConfigSet(Default_Ltr329ConfigData);
        err = u8Sths34pf80ConfigSet(Default_sths34pf80ConfigData);
        err = u8Pyq1548ConfigSet(Default_pyq1548ConfigData);
        if (NVS_ERR_NONE != err)
        {
            ESP_LOGE(TAG,"Failed to  write default data , return Status: %d", err);
        }
        else
        {
            ESP_LOGI(TAG,"Written default ltr329 data to nvs Succesfully");
            err = u8Ltr329ConfigGet(&LTR_configData);
            err = u8Sths34pf80ConfigGet(&sths_configData);
            err = u8Pyq1548ConfigGet(&pyq_configData);
            if(NVS_ERR_NONE != err)
            {
                ESP_LOGE(TAG,"Failed to read default  sensor configured data from nvs, return Status: %d", err);
            }
            else
            {
                ESP_LOGI(TAG,"default lux gain setting %d",LTR_configData.lux_operating_mode_gain_setting);
                ESP_LOGI(TAG,"default lux als rate %d",LTR_configData.lux_als_measurement_rate);
                ESP_LOGI(TAG,"default sths34pf sensitivity_level %d",sths_configData.sths34pf80_sensitivity_level);
                ESP_LOGI(TAG,"default sths34pf ODR %d",sths_configData.sths34pf80_ODR);
                ESP_LOGI(TAG,"default sths34pf Amb shock hyst %d",sths_configData.sths34pf80_ambient_temp_shock_hyst);
                ESP_LOGI(TAG,"default pyq configuration %08X",(int)pyq_configData.PYQconf_data);
            }
        }
    }
    else
    {
        ESP_LOGI(TAG,"lux gain setting %d",LTR_configData.lux_operating_mode_gain_setting);
        ESP_LOGI(TAG,"lux als rate %d",LTR_configData.lux_als_measurement_rate);
        ESP_LOGI(TAG,"sths34pf sensitivity_level %d",sths_configData.sths34pf80_sensitivity_level);
        ESP_LOGI(TAG,"sths34pf ODR %d",sths_configData.sths34pf80_ODR);
        ESP_LOGI(TAG,"sths34pf Amb shock hyst %d",sths_configData.sths34pf80_ambient_temp_shock_hyst);
        ESP_LOGI(TAG,"pyq configuration  %08X ",(int)pyq_configData.PYQconf_data);
    }
    return err;
}
/**
 * @brief: API to initialize database
 */
uint8_t MS_vDatabaseInit(void)
{
    uint8_t err = NVS_ERR_NONE;
    // err = nvs_flash_init();      //Initialization Of NVS
    // if (ESP_ERR_NVS_NO_FREE_PAGES == err || ESP_ERR_NVS_NEW_VERSION_FOUND == err)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());    // NVS partition was truncated and needs to be erased
    //     err = nvs_flash_init();         // Retry nvs_flash_init
    //     ESP_LOGE(TAG, "NSV init Failed");
    // }
    // ESP_ERROR_CHECK(err);
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvsHandler);  //open common NVS Storage
    if (NVS_ERR_NONE != err)
    {
        ESP_LOGE(TAG, "NVS Open Storage fail.");
    }
    else
    {
        ESP_LOGI(TAG, "NVS Open Storage Successfully");
    }
    return err;
}

