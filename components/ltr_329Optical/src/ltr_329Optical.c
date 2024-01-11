/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: Ruchita Shekhada
 *      Date & Time:2023-09-28 && 9:35:00
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
 * \file        ltr329.c
 * \brief       Ambient Light Sensor - LTR-329ALS Source File
 */

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include "ltr_329Optical.h"         
#include "i2c_service.h"
/************************************************************************
 * Define Macros
 ************************************************************************/
#define LTR329_ID                       0xA0
#define LTR329_MANUF_ID                 0x05
#define LTR329_MAX_RETRY                3 /* Number of measurement retry */
/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/
/**
 *  default value of LTR329 sensors
 */
LTR329_config_details_t Default_Ltr329ConfigData = {
    
    .lux_als_measurement_rate   = DEF_LUX_ALS_MEASUREMENT_RATE,
    .lux_operating_mode_gain_setting   = DEF_LUX_OPERATING_MODE_GAIN_SETTING,
    .lux_level_lower_thresold = DEF_LUX_LEVEL_LOWER_THRESOLD,
    .lux_level_upper_thresold = DEF_LUX_LEVEL_UPPER_THRESOLD,
};
/************************************************************************
 * Define Private Variables
 ************************************************************************/

/************************************************************************
 * Define Global Variables
 ************************************************************************/
LTR329_config_details_t LTR_configData;
/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/
/**
 * @brief : Read manufacture id of LTR 329
 */
LTR_err_t Read_LTR329_ManufacturerId(void)
{
    LTR_err_t err;
       /* Check PART ID */
    uint8_t Part_number_id;
    err = Read_Data_From_LTR_329(LTR329_PART_ID, &Part_number_id, sizeof(Part_number_id));
    if (LTR329_ERR_NONE != err)
    { 
        err =  LTR329_ERR_I2C;
    }
    if (LTR329_ID != Part_number_id)
    {
        err = LTR329_ERR_ID;
    }
    return err;
}
/**
 * @brief : Read contents of a LTR 329 register 
 */
LTR_err_t Read_Data_From_LTR_329( uint8_t reg, uint8_t *pdata, uint8_t count )
{
    LTR_err_t err;
	err = i2c_master_read_slave_reg( I2C_MASTER_NUM, SENSOR_I2C_ADDR_LTR329,  reg, pdata, count );
    if(LTR329_ERR_NONE != err) 
    {
        err = LTR329_ERR_I2C;
    }  
    return err;
}
/**
 * @brief :  Write value to specified LTR 329 register 
 */
LTR_err_t Write_Data_To_LTR_329( uint8_t reg, uint8_t *pdata, uint8_t count )
{
    LTR_err_t err;
	err = i2c_master_write_slave_reg( I2C_MASTER_NUM, SENSOR_I2C_ADDR_LTR329,  reg, pdata, count );
    if(LTR329_ERR_NONE != err) 
    {
        err = LTR329_ERR_I2C;
    }  
    return err;
}
/**
 * @brief : Put sensor in active or standby mode & configure sensor gain..
 */
LTR_err_t LTR329_set_op_mode_Gain(uint8_t oModenGain)
{
    LTR_err_t err;
    uint8_t LTR_gain;
    RIGHT_SHIFT_BY(oModenGain,2 ,LTR_gain);
    if ( (LTR_gain > LTR329_GAIN_LAST) || (LTR_gain == LTR329_GAIN_INVALID0) || (LTR_gain == LTR329_GAIN_INVALID1) )
    {
        err = LTR329_ERR_GAIN;
    }
    else
    {
        /* Set sensor in active mode */
        err = Write_Data_To_LTR_329(LTR329_ALS_CONTR, &oModenGain, sizeof(oModenGain));
        if (LTR329_ERR_NONE != err)
        {
            err = LTR329_ERR_I2C;
        }
    }
    return err;
}
/**
 * @brief :This Function Configure LTR329 sensor
 */
LTR_err_t i32LTR329DefaultInit(uint8_t m_rate , uint8_t gm_setting)
{
    LTR_err_t err;
    /* Reset sensor */ 
    uint8_t software_reset_data = LTR329_SOFTWARE_RESET;            /* SW reset */
    err = Write_Data_To_LTR_329(LTR329_ALS_CONTR, &software_reset_data, sizeof(software_reset_data));
    if (LTR329_ERR_NONE != err)
    { 
        err = LTR329_ERR_I2C;
    }
    /* Configure sensor measurement rate & integration time */
    err = Write_Data_To_LTR_329(LTR329_ALS_MEAS_RATE, &m_rate, sizeof(m_rate));
    if (LTR329_ERR_NONE != err)
    { 
        err = LTR329_ERR_I2C;
    }
     /* Active light sensor */
    err = LTR329_set_op_mode_Gain(gm_setting);
    if (LTR329_ERR_NONE != err)
    {
        err = LTR329_ERR_I2C;
    }
    return err;
}
/**
 * @brief :This Function Put sensor in stand-by mode.
 */
LTR_err_t LTR329_set_standby_mode(void)
{
    LTR_err_t err;
    uint8_t set_stand_by_mode = LTR329_STAND_BY_MODE;
    /* Set sensor in stand-by mode */
    err = Write_Data_To_LTR_329(LTR329_ALS_CONTR, &set_stand_by_mode, sizeof(set_stand_by_mode));
    if (LTR329_ERR_NONE != err)
    {
        err =  LTR329_ERR_I2C;
    }
    return err;
}
/**
 * @brief :  Get a light measurement.<BR>
 * Values must be divided by the configured gain to convert it in lux.
 */
LTR_err_t i32GetLTR329ChannelData(uint32_t* LTR_ch01_Data)
{
    LTR_err_t err;
    uint8_t iter;
    /* Wait measurement is completed */
    for ( iter = 0 ; iter < LTR329_MAX_RETRY ; iter++)
    {
        uint8_t als_data_status;
        err = Read_Data_From_LTR_329(LTR329_ALS_STATUS, &als_data_status, 1);
        if (ESP_OK != err)
        {
            return LTR329_ERR_I2C;
        }
        if ( (als_data_status & LTR329_DATA_READY) == LTR329_DATA_READY )
        {
            /* Data is ready. Loop end. */
            break;
        }
        else
        {
            /* Wait */
        }
    }
    if (iter == LTR329_MAX_RETRY)
    {
        err =  LTR329_ERR_MEASURE_TIMEOUT;
    }
    else
    {
        /* Get measurement data */
        err = Read_Data_From_LTR_329(LTR329_ALS_DATA_CH1_0, (uint8_t *)LTR_ch01_Data, sizeof(uint32_t));
        if (ESP_OK != err)
        {
            err = LTR329_ERR_I2C;
        }
        else
        {
            /*Do Nothing*/
        }
        
    }
    return err;
}
