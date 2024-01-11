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
 * \file    NVS_sensorConfig.h
 * \brief
 *
 */

#ifndef _NVS_SENSORCONFIG_H_
#define _NVS_SENSORCONFIG_H_

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/************************************************************************
 * Define Macros
 ************************************************************************/
#define NVS_ERR_NONE                           0

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
/**
 * @brief: API to restore data from nvs storage
 * 		   and fill respective structure
 */
uint8_t MS_RestoreNVSData(void);
/**
 * @brief: API to initialize database
 */
uint8_t MS_vDatabaseInit(void);
#endif
