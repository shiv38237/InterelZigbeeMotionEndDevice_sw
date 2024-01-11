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
 * \file    led_control.h
 * \brief
 *
 */

#ifndef _LED_CONTROL_H_
#define _LED_CONTROL_H_

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/************************************************************************
 * Define Macros
 ************************************************************************/
#define LED_PIN                            GPIO_NUM_14  
/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/


/************************************************************************
 * Define Private Variables
 ************************************************************************/

/************************************************************************
 * Define Global Variables
 ************************************************************************/


/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/
void led_gpio_set_level(int gpio_level);
void LED_init(void);
#endif
