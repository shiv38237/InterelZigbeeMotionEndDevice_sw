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
 * \file    button_control.h
 * \brief
 *
 */

#ifndef _BUTTON_CONTROL_H_
#define _BUTTON_CONTROL_H_

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/************************************************************************
 * Define Macros
 ************************************************************************/
#define COMMISSIONING_SWITCH_PIN        GPIO_NUM_0
#define GPIO_INPUT_PIN_SEL              (1ULL<<COMMISSIONING_SWITCH_PIN)
#define ESP_INTR_FLAG_DEFAULT           0
/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/

/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/


/************************************************************************
 * Define Private Variables
 ************************************************************************/

/************************************************************************
 * Define Global Variables
 ************************************************************************/
extern QueueHandle_t gpio_evt_queue;
/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/
void Button_init (void);

#endif
