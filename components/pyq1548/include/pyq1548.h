/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: 
 *      Date & Time:2023-08-21
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
 * \file     pyq1548.h
 * \brief
 *
 */

#ifndef _PYQ1548_H_
#define _PYQ1548_H_

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include "driver/gpio.h"
#include "esp_system.h"

/************************************************************************
 * Define Macros
 ************************************************************************/
#define SERIAL_IN_GPIO_PIN                     22 // Change this to the GPIO pin you are using
#define DLINK_GPIO_PIN                         2 // Change this to the GPIO pin you are using
#define MOTION_INDICATOR__GPIO                 14
#define DEF_PYQ_MODE_CONFIGURATION             0x0010
#define DEF_PYQ_UPPER_THRESOLD                 10
#define DEF_PYQ_LOWER_THRESOLD                -10
/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/
typedef struct PYQ1548_config_details
{
        unsigned long PYQconf_data;
        int16_t pyq_lower_thresold;
        int16_t pyq_upper_thresold;
}__attribute__((packed)) PYQ1548_config_details_t;

/************************************************************************
 * Define Private Variables
 ************************************************************************/

/************************************************************************
 * Define Global Variables
 ************************************************************************/
extern PYQ1548_config_details_t pyq_configData;
extern PYQ1548_config_details_t Default_pyq1548ConfigData;

/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/
int i32ReadLowPowerPyro(void) ;
void vWritePyqRegval(unsigned long regval);
void vPyqGPIO_init(void);
void vPyqMotionDetectionEnable(void);
void vPyqMotionDetectionDisable(void);

/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/

#endif
