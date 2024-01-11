/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: Ruchita Shekhada
 *      Date & Time:2023-11-20 && 9:40:00
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
 * \file     Interel_Zigbee_Motion_sw.h
 *
 */
/************************************************************************
 * Include Header Files
 ************************************************************************/
#include "esp_zigbee_core.h"
/************************************************************************
 * Define Macros
 ************************************************************************/
#define PYQ_SUPPLY_PIN                      GPIO_NUM_3 
#define STH_SUPPLY_PIN                      GPIO_NUM_1
#define LUX_SUPPLY_PIN                      GPIO_NUM_21
#define OPEN_COLLECTOR_PIN                  GPIO_NUM_12
/* endpoint identifiers*/
#define ZDO_ENDPOINT                            0x01
#define HA_MOTION_SENSOR_ENDPOINT               0x22
#define HA_TEMP_SENSOR_ENDPOINT                 0x26
#define HA_LIGHT_SENSOR_ENDPOINT                0x27
#define HA_IAS_ZONE_ENDPOINT                    0x23
#define HA_MOTION_SENSOR_WITH_TEMP_ENDPOINT     0x28
#define HA_MOTION_SENSOR_WITH_LIGHT_ENDPOINT    0x29

#define MODIFY_BIT(regValue,bitPosition,valuetoset)\
do { \
	uint8_t mask = 1 << bitPosition;\
    regValue = ((regValue & ~mask) | (valuetoset << bitPosition));\
} while(false)
#define CHECK_BIT(regValue,bitPosition)     (((regValue)>>(bitPosition)) & 0001)
#define TIME_SECOND_TO_MICROSECOND          1000000
/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false    /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask use in the example */

#define ESP_ZB_ZED_CONFIG()                                         \
{                                                               \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
    .nwk_cfg.zed_cfg = {                                        \
        .ed_timeout = ED_AGING_TIMEOUT,                         \
        .keep_alive = ED_KEEP_ALIVE,                            \
    },                                                          \
}

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                       \
{                                                           \
    .radio_mode = RADIO_MODE_NATIVE,                        \
}

#define ESP_ZB_DEFAULT_HOST_CONFIG()                        \
{                                                           \
    .host_connection_mode = HOST_CONNECTION_MODE_NONE,      \
}
/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/


/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/
void factory_reset_timer_callback(void *arg);
void reporting_timer_callback(void *arg);
/**
 * @name   pyq_mot_timer_callback
 * @brief: Call back to reset pyq occupancy flag
 *
 * @param[in] void 
 * @param[out] void
 */
void pyq_mot_timer_callback(void *arg);
/**
 * @name   St_mot_timer_callback
 * @brief: Call back to reset sths34pf80 motion occupancy flag
 *
 * @param[in] void 
 * @param[out] void
 */
void St_mot_timer_callback(void *arg);
/**
 * @name   St_pres_timer_callback
 * @brief: Call back to reset sths34pf80 presense occupancy flag
 *
 * @param[in] void 
 * @param[out] void
 */
void St_pres_timer_callback(void *arg);
/**
 * @name   timers_init
 * @brief: timer to keep motion/presense flag for defined time period
 *         after detection
 * @param[in] void
 * @param[out] esp_err_t
 */
esp_err_t timers_init(void);
/**
 * @name   gpioInit
 * @brief: initialize gpio
 * @param[in] void
 * @param[out] esp_err_t
 */
esp_err_t gpioInit(void);
esp_err_t example_register_timer_wakeup(void);
