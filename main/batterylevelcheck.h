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
 * \file    batterylevelcheck.h
 * \brief
 *
 */

#ifndef _BATTERYLEVELCHECK_H_
#define _BATTERYLEVELCHECK_H_

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/************************************************************************
 * Define Macros
 ************************************************************************/

#define VOLTAGE_MAX 3300 //8200
#define VOLTAGE_MIN 2900 //7000

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_4
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11
#define ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED 0
#define ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED_NEW 1
#define BATTERY_MAX_ADC             3280//VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MAX))
#define BATTERY_MIN_ADC             2910//VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MIN))
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
void voltage_calculate_init(void);
int get_battery_level(void);
/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/

#endif
