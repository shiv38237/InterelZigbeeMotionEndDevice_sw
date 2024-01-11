/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: Ruchita Shekhada
 *      Date & Time:2023-09-28 && 9:40:00
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
 * \file     ltr329.h
 * \brief    Ambient Light Sensor - LTR-329ALS Header File  
 *
 */

#ifndef _LTR329_H_
#define _LTR329_H_

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include "i2c_service.h"
/************************************************************************
 * Define Macros
 ************************************************************************/
#define SENSOR_I2C_ADDR_LTR329              0x29 /*!< Light sensor address */
/*!******************************************************************
 * \defgroup LTR329_REG LTR329 Registers
 * \brief Address of LTR329 registers
 *
 *  @{
 *******************************************************************/
#define LTR329_ALS_CONTR                   0x80 /*!< ALS operation mode control SW reset */
#define LTR329_ALS_MEAS_RATE               0x85 /*!< ALS measurement rate in active mode */
#define LTR329_PART_ID                     0x86 /*!< Part Number ID and Revision ID */
#define LTR329_MANUFAC_ID                  0x87 /*!< Manufacturer ID */
#define LTR329_ALS_DATA_CH1_0              0x88 /*!< ALS measurement CH1 data, lower byte */
#define LTR329_ALS_DATA_CH1_1              0x89 /*!< ALS measurement CH1 data, upper byte */
#define LTR329_ALS_DATA_CH0_0              0x8A /*!< ALS measurement CH0 data, lower byte */
#define LTR329_ALS_DATA_CH0_1              0x8B /*!< ALS measurement CH0 data, upper byte */
#define LTR329_ALS_STATUS                  0x8C /*!< ALS new data status */
/**@}*/
/*!******************************************************************
 * \defgroup LTR329_ERR_CODES LTR329 Error Codes
 * \brief Error code definitions for LTR329
 *
 *  @{
 *******************************************************************/
#define LTR329_ERR_NONE                    0x00 /*!< No error */
#define LTR329_ERR_I2C                     0x40 /*!< Error on I2C communication */
#define LTR329_ERR_ID                      0x41 /*!< Bad ID value */
#define LTR329_ERR_GAIN                    0x42 /*!< Gain has an invalid value */
#define LTR329_ERR_MEASURE_TIMEOUT         0x43 /*!< Error, measurement is too long */
/**@}*/
#define LTR329_SOFTWARE_RESET              0x02
#define LTR329_DATA_READY                  0x04
#define LTR329_STAND_BY_MODE               0x00

#define RIGHT_SHIFT_BY(regValue,bitToShift,finalValue )\
do { \
    finalValue = regValue >> bitToShift;\
} while(false)


#define DEF_LUX_ALS_MEASUREMENT_RATE           0x3
#define DEF_LUX_OPERATING_MODE_GAIN_SETTING    0x0
#define DEF_LUX_LEVEL_UPPER_THRESOLD           6100
#define DEF_LUX_LEVEL_LOWER_THRESOLD           0
/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/

/*!******************************************************************
 * \enum ltr329_gain_e
 * \brief Setting for light sensor gain.
 *******************************************************************/
typedef enum {
    LTR329_GAIN_1X       = 0b000, /*!< Gain x1 -> range: 1 lux to 64k lux */
    LTR329_GAIN_2X       = 0b001, /*!< Gain x2 -> range: 0.5 lux to 32k lux */
    LTR329_GAIN_4X       = 0b010, /*!< Gain x4 -> range: 0.25 lux to 16k lux */
    LTR329_GAIN_8X       = 0b011, /*!< Gain x8 -> range: 0.125 lux to 8k lux */
    LTR329_GAIN_INVALID0 = 0b100, /*!< Invalid gain value */
    LTR329_GAIN_INVALID1 = 0b101, /*!< Invalid gain value */
    LTR329_GAIN_48X      = 0b110, /*!< Gain x48 -> range: 0.02 lux to 1.3k lux */
    LTR329_GAIN_96X      = 0b111, /*!< Gain x96 -> range: 0.01 lux to 600 lux */
    LTR329_GAIN_LAST
} ltr329_gain_e;

typedef struct LTR329_config_details
{
        uint8_t lux_als_measurement_rate;
        uint8_t lux_operating_mode_gain_setting;
        int16_t lux_level_lower_thresold;
        int16_t lux_level_upper_thresold;
}__attribute__((packed)) LTR329_config_details_t;
/************************************************************************
 * Define Private Variables
 ************************************************************************/

/************************************************************************
 * Define Global Variables
 ************************************************************************/
typedef int LTR_err_t;
extern LTR329_config_details_t LTR_configData;
extern LTR329_config_details_t Default_Ltr329ConfigData;
/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/


/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/
/**
 * @name   LTR329_init
 * @brief :This Function Configure LTR329 sensor
 *
 * @param[in] m_rate
 * @param[in] gm_setting
 * @param[out] LTR_err_t Return LTR329 Error
 */
LTR_err_t i32LTR329DefaultInit(uint8_t m_rate , uint8_t gm_setting);
/**
 * @name   LTR329_set_standby_mode
 * @brief :This Function Put sensor in stand-by mode.
 *
 * @param[in] void
 * @param[out] LTR_err_t Return LTR329 Error
 */
LTR_err_t LTR329_set_standby_mode(void);
/**
 * @name: LTR329_set_op_mode_Gain
 * @brief : Put sensor in active or standby mode & configure sensor gain.
 * 
 * @param[in] oModenGain 
 * @param[out] LTR_err_t Return LTR329 Error
 */
LTR_err_t LTR329_set_op_mode_Gain(uint8_t oModenGain);
/**
 * @name :i32GetLTR329ChannelData
 * @brief :  Get a light measurement.<BR>
 * Values must be divided by the configured gain to convert it in lux.
 * 
 * @param[out]  LTR_ch01_Data 
 * @param[out] LTR_err_t Return LTR329 Error
 */ 
LTR_err_t i32GetLTR329ChannelData(uint32_t* LTR_ch01_Data);
/**
 * @name : Read_Data_From_LTR_329
 * @brief: API to read  LTR 329 reg value 
 *
 * @param[in] reg   reg address to read 
 * @param[in] count	size of read byte 
 *
 * @param[out] pdata 	  Return reg stored data
 * @param[out] LTR_err_t 	Return LTR329 Error
 */
LTR_err_t Read_Data_From_LTR_329( uint8_t reg, uint8_t *pdata, uint8_t count );
/**
 * @name :  Write_Data_To_LTR_329
 * @brief: API to write LTR 329 reg value 
 *
 * @param[in] reg   reg address to write 
 * @param[in] pdata	reg data to write in register addr
 * @param[in] count	size of write byte
 *
 * @param[out] LTR_err_t 	Return LTR329 Error
 */
LTR_err_t Write_Data_To_LTR_329( uint8_t reg, uint8_t *pdata, uint8_t count );
/**
 * @name :  Read_LTR329_ManufacturerId
 * @brief : Read manufacture id of LTR 329
 * 
 * @param[in] void	
 *
 * @param[out] LTR_err_t 	Return LTR329 Error
 */
LTR_err_t Read_LTR329_ManufacturerId(void);
#endif
