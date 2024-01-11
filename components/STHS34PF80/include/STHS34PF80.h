/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author:  Ruchita Shekhada
 *      Date & Time:2023-09-20 && 9:40:00
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
 * \file     sths34pf80.h
 * \brief
 *
 */

#ifndef _STHS34PF80_H_
#define _STHS34PF80_H_

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include "i2c_service.h"
/************************************************************************
 * Define Macros
 ************************************************************************/
#define STHS34PF80_I2C_ADDR		              0x5A
#define WHO_AM_I_REG			                  0x0F

#define STHS34PF80_DEVICE_ID                0xD3
#define STHS34PF80_FUNC_STATUS              0x25
#define STHS34PF80_SENS_DATA                0x1D
#define STHS34PF80_LPF1                     0x0C
#define STHS34PF80_LPF2                     0x0D  
#define STHS34PF80_AVG_TRIM                 0x10

#define STHS34PF80_TOBJECT_L                0x26
#define STHS34PF80_TOBJECT_H                0x27
#define STHS34PF80_TAMBIENT_L               0x28
#define STHS34PF80_TAMBIENT_H               0x29
#define STHS34PF80_TOBJ_COMP_L              0x38
#define STHS34PF80_TOBJ_COMP_H              0x39
#define STHS34PF80_TPRESENCE_L              0x3A
#define STHS34PF80_TPRESENCE_H              0x3B
#define STHS34PF80_TMOTION_L                0x3C
#define STHS34PF80_TMOTION_H                0x3D
#define STHS34PF80_TAMB_SHOCK_L             0x3E
#define STHS34PF80_TAMB_SHOCK_H             0x3F
#define STHS34PF80_STATUS                   0x23
#define STHS34PF80_CTRL0                    0x17
#define STHS34PF80_CTRL1                    0x20
#define STHS34PF80_CTRL2                    0x21
#define STHS34PF80_CTRL3                    0x22
#define STHS34PF80_STATUS                   0x23

#define NORMAL_MODE                         0x07  // default gain mode
#define WIDE_MODE                           0x00
#define POWER_DOWN                          0
// Embedded functions page register mapping
#define STHS34PF80_FUNC_CFG_ADDR            0x08
#define STHS34PF80_FUNC_CFG_DATA            0x09
#define STHS34PF80_FUNC_PAGE_RW             0x11

#define STHS34PF80_PRESENCE_THS_L           0x20
#define STHS34PF80_PRESENCE_THS_H           0x21
#define STHS34PF80_MOTION_THS_L             0x22
#define STHS34PF80_MOTION_THS_H             0x23
#define STHS34PF80_TAMB_SHOCK_THS_L         0x24
#define STHS34PF80_TAMB_SHOCK_THS_H         0x25
#define STHS34PF80_HYST_MOTION              0x26
#define STHS34PF80_HYST_PRESENCE            0x27
#define STHS34PF80_ALGO_CONFIG              0x28
#define STHS34PF80_HYST_TAMB_SHOCK          0x29
#define STHS34PF80_RESET_ALGO               0x2A

#define EN_DIS_ACCESS_EMBEDDED_FUNC_REG     0x10
#define EN_WRITE_EMBEDDED_FUNC_REG          0x40
#define DIS_WRITE_EMBEDDED_FUNC_REG         0x00
#define EN_READ_EMBEDDED_FUNC_REG           0x20
#define DIS_READ_EMBEDDED_FUNC_REG          0x00
#define SET_STATUS_DRDY_BIT                 0x04
#define SET_CTRL2_BOOT_BIT                  0x80
#define EMBEDDED_ALGO_CONFIGURATION         0x0c
#define CTRL3_INTERRUPT_CONFIGURATION       0x3a
#define STH_PRESENSE_DETECT_BIT             0x04
#define STH_MOTION_DETECT_BIT               0x02
#define STH_TAMB_SHOCK_DETECT_BIT           0x01
#define BITS_PER_U16                        (16U)
#define BITS_PER_U32                        (32U)
#define BITS_PER_U16                        (16U)
#define BITS_PER_U8                         (8U)
#define U32_LOWEST_U16_MASK                 (0x0000FFFFL)
#define U32_HIGHEST_U16_MASK                (0xFFFF0000L)
#define U16_LOW_U8_MASK                     (0x00FFU)
#define U16_HIGH_U8_MASK                    (0xFF00U)
#define U16_UPPER_U8( x )                   ((uint8_t) ((x) >> BITS_PER_U8))
#define U16_LOWER_U8( x )                   ((uint8_t) (((x) & U16_LOW_U8_MASK)))
#define U32_HIGHEST_U16( x )                ((uint16_t)(((x) & U32_HIGHEST_U16_MASK) >> (BITS_PER_U16)))
#define U32_LOWEST_U16( x )                 ((uint16_t)( ( ( x ) & U32_LOWEST_U16_MASK ) ))

#define STHS_ERR_NONE                       0x00 /*!< No error */
#define STHS_ERR_I2C                        0x50 /*!< Error on I2C communication */
#define STHS_ERR_ID                         0x51 /*!< Bad ID value */

#define STHS34PF80_INT_PIN                   GPIO_NUM_5   // 10 interrupt pin definitions 
#define DEF_STHS_LPF_CUTOFF_FREQUENCY_1        0x04
#define DEF_STHS_LPF_CUTOFF_FREQUENCY_2        0x22
#define DEF_STHS_AVERAGING_OF_TEMP_DATA        0x03
#define DEF_STHS_ODR                           0x06
#define DEF_STHS_PRESENCE_THRESOLD             40
#define DEF_STHS_MOTION_THRESOLD               40
#define DEF_STHS_AMBIENT_TEMP_SHOCK_THRESOLD   20
#define DEF_STHS_PRESENCE_HYST                 5
#define DEF_STHS_MOTION_HYST                   5
#define DEF_STHS_AMBIENT_TEMP_SHOCK_HYST       10
#define DEF_STHS_SENSITIVITY_LEVEL             2000
#define DEF_AMBIENT_TEMP_UPPER_THRESOLD        50    //degree celsious
#define DEF_AMBIENT_TEMP_LOWER_THRESOLD        20
/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/
// Averaging for ambient temperature
typedef enum {
  STHS34PF80_AVGT_8                = 0x00, // default
  STHS34PF80_AVGT_4                = 0x01,  
  STHS34PF80_AVGT_2                = 0x02,
  STHS34PF80_AVGT_1                = 0x03
} AVGT;
// Averaging for object temperature
typedef enum {
  STHS34PF80_AVGTMOS_2             = 0x00,  
  STHS34PF80_AVGTMOS_8             = 0x01,  
  STHS34PF80_AVGTMOS_32            = 0x02,
  STHS34PF80_AVGTMOS_128           = 0x03,  // default
  STHS34PF80_AVGTMOS_256           = 0x04,  
  STHS34PF80_AVGTMOS_512           = 0x05,  
  STHS34PF80_AVGTMOS_1024          = 0x06,
  STHS34PF80_AVGTMOS_2048          = 0x07
} AVGTMOS;
// Low-pass filter options
typedef enum  {
  STHS34PF80_ODR_09           	   = 0x00, // low pass filter = ODR divided by 9, etc.
  STHS34PF80_ODR_20           	   = 0x01,
  STHS34PF80_ODR_50           	   = 0x02,
  STHS34PF80_ODR_100          	   = 0x03,
  STHS34PF80_ODR_200          	   = 0x04,
  STHS34PF80_ODR_400          	   = 0x05,
  STHS34PF80_ODR_800          	   = 0x06
} LPF;
// ODR (data sample rate)
typedef enum {
  STHS34PF80_PWRDOWN               = 0x00,  
  STHS34PF80_ODR_0_25              = 0x01,  
  STHS34PF80_ODR_0_50              = 0x02,
  STHS34PF80_ODR_1                 = 0x03,  
  STHS34PF80_ODR_2                 = 0x04,  
  STHS34PF80_ODR_4                 = 0x05,  
  STHS34PF80_ODR_8                 = 0x06,
  STHS34PF80_ODR_15                = 0x07,
  STHS34PF80_ODR_30                = 0x08
} ODR;

typedef struct sths_config_details
{
  int16_t sths34pf80_sensitivity_level;
  uint8_t sths34pf80_LPF_cutoff_frequency_1;
  uint8_t sths34pf80_LPF_cutoff_frequency_2;
  uint8_t sths34pf80_averaging_of_temp_data;
  uint16_t sths34pf80_presence_thresold;
  uint16_t sths34pf80_motion_thresold;
  uint16_t sths34pf80_ambient_temp_shock_thresold;
  uint8_t sths34pf80_presence_hyst;
  uint8_t sths34pf80_motion_hyst;
  uint8_t sths34pf80_ambient_temp_shock_hyst;
  uint8_t sths34pf80_ODR;
  int16_t ambient_temp_lower_thresold;
  int16_t ambient_temp_upper_thresold;
}__attribute__((packed)) sths_config_details_t;

typedef struct STHS34PF80_Data {
  int16_t i16Presence;
  int16_t i16Motion; 
  int16_t i16AmbTemp;
  int16_t i16ObjTemp;
  int16_t i16CObjTemp;
  int16_t i16AmbShock;
  uint8_t u8MotionFlag;
  uint8_t u8PresenceFlag;  
}stsensor_data_t;
/************************************************************************
 * Define Private Variables
 ************************************************************************/

/************************************************************************
 * Define Global Variables
 ************************************************************************/
typedef int sths_err_t;
extern sths_config_details_t sths_configData;
extern sths_config_details_t Default_sths34pf80ConfigData;
/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/
/**
 * @name : rd_shthspf80_reg
 * @brief: API to read  shthspf80 reg value 
 *
 * @param[in] reg   reg address to read 
 * @param[in] count	size of read byte 
 *
 * @param[out] pdata 	  Return reg stored data
 * @param[out] sths_err_t 	Return sths error
 */
sths_err_t rd_shthspf80_reg( uint8_t reg, uint8_t *pdata, uint8_t count );
/**
 * @name :  wr_shthspf80_reg
 * @brief: API to write shthspf80 reg value 
 *
 * @param[in] reg   reg address to write 
 * @param[in] pdata	reg data to write in register addr
 * @param[in] count	size of write byte
 *
 * @param[out] sths_err_t 	Return sths error
 */
sths_err_t wr_shthspf80_reg( uint8_t reg, uint8_t *pdata, uint8_t count );
/**
 * @name :  i8Shthspf80SensorInit
 * @brief :This function configure all the register of sthspf80 sensor
 *
 * @param[in]  sths_config_details_t
 * @param[out] sths_err_t 	Return sths error
 */
sths_err_t i8Shthspf80SensorInit(sths_config_details_t sths_configurationData);
/**
 * @name :  shthspf80_reset
 * @brief :This function reset sthspf80 sensor
 *
 * @param[in]  void
 * @param[out] sths_err_t 	Return sths error
 */
sths_err_t shthspf80_reset(void);
/**
 * @name :  shthspf80_powerDown_Mode
 * @brief : This fuction put sths34pf80 sensor in to power down mode
 *
 * @param[in]  void
 * @param[out] sths_err_t 	Return sths error
 * 
 */
sths_err_t shthspf80_powerDown_Mode(void) ;
/**
 * @name :  sthspf80_readSenseData
 * @brief : This fuction give the sensitivity data of  sths34pf80 sensor
 *
 * @param[in]  void
 * @param[out] int8_t sensitivity_data 
 * @param[out] sths_err_t 	Return sths error
 * 
 */
sths_err_t sthspf80_readSenseData(int8_t *sensitivity_data);
/**
 * @name :  sthspf80_writeSenseData
 * @brief : This fuction write new  the sensitivity data in register
 *
 * @param[in]  void
 * @param[in] int16_t sensitivity_data 
 * @param[out] sths_err_t 	Return sths error
 * 
 */
sths_err_t sthspf80_writeSenseData(int16_t sensitivity_data);
/**
 * @name : sthspf80_setLowpassFilters
 * @brief : This function set LPF 1 and LPF 2 register ,cutoff frequency
 * 
 * @param[in] lpf_1 
 * @param[in] lpf_2 
 * @param[out] sths_err_t 	Return sths error
 */
sths_err_t sthspf80_setLowpassFilters(uint8_t lpf_1, uint8_t lpf_2);
/**
 * @name : shthspf80_config
 * @brief : This function set number of average sample for ambient and object temperature
 * 
 * @param[in] set_avgt_avgtmos 
 * @param[in] gain 
 * @param[out] sths_err_t Return sths error
 */
sths_err_t shthspf80_config(uint8_t set_avgt_avgtmos, uint8_t gain);
/**
 * @name : shthspf80_configAlgo
 * @brief : This function fill the thresold and hysterisis value in register
 * 
 * @param[in] presence_ths 
 * @param[in]  motion_ths 
 * @param[in]  tamb_shock_ths 
 * @param[in]  presence_hyst 
 * @param[in]  motion_hyst 
 * @param[in]  tamb_shock_hyst 
 * @param[out] sths_err_t  Return sths error
 */
sths_err_t shthspf80_configAlgo(uint16_t presence_ths, uint16_t motion_ths, uint16_t tamb_shock_ths, uint8_t presence_hyst, uint8_t motion_hyst, uint8_t tamb_shock_hyst);
/**
 * @name : shthspf80_readAlgoConfig
 * @brief : This function read all the thresold and hysterisis value from the register 
 * 
 * @param[out] dest 
 * @param[out] sths_err_t  Return sths error
 */
sths_err_t shthspf80_readAlgoConfig(uint16_t *dest);
/**
 * @name : shthspf80_resetAlgo
 * @brief : This function reset the algorithem ,use when change odr,
 *          filter value or multiple embedded function register
 * 
 * @param[out] sths_err_t Return sths error
 */
sths_err_t shthspf80_resetAlgo(void);
/**
 * @name : shthspf80_powerUp
 * @brief : This function power up and enter in to continuoes mode 
 *          with configure odr
 * 
 * @param[in] odr 
 * @param[out] sths_err_t Return sths error
 */
sths_err_t shthspf80_powerUp(uint8_t odr);
/**
 * @name : sthspf80_getFuncStatus
 * @brief Get the Status for presence ,motion , ambient temp shock detection
 * 
 * @param[out] flag_detect 
 * @param[out] sths_err_t Return sths error
 */
sths_err_t sthspf80_getFuncStatus(uint8_t *flag_detect);
/**
 * @name : sthspf80_getDataReadyStatus
 * @brief :This function give state if new set of data is available or not
 * 
 * @param[out] data_ready 
 * @param[out] sths_err_t Return sths error
 */
sths_err_t sthspf80_getDataReadyStatus(uint8_t *data_ready);
/**
 * @name : shthspf80_embedded_function_reg
 * @brief : This function will fill the embedded reg value
 * 
 * @param[in] reg_address 
 * @param[in] register_value
 * @param[in] size
 * @param[out] sths_err_t Return sths error
 */
sths_err_t shthspf80_embedded_function_reg(uint8_t reg_address ,uint16_t register_value,uint8_t size);
/**
 * @name : readPresence
 * @brief : This function return presense raw data
 * 
 * @param[out] int16_t presense data
 */
int16_t readPresence(void);
/**
 * @name : readObjTemp
 * @brief :This function return object temp data
 * 
 * @param[out] int16_t presense data
 */
int16_t readObjTemp(void);
/**
 * @name : readMotion
 * @brief : This function return motion raw data
 * 
 * @param[out] int16_t presense data
 */
int16_t readMotion(void);
/**
 * @name : readAmbShock
 * @brief : This function return ambient shock raw data
 * 
 * @param[out] int16_t presense data
 */
int16_t readAmbShock(void);
/**
 * @name : readAmbTemp
 * @brief : This function return ambient temp data
 * 
 * @param[out] int16_t presense data
 */
int16_t readAmbTemp(void);
/**
 * @name : twos_complement_to_decimal16_conversion
 * @brief : This function return converted decimal value of 2's
 *          complement value
 *  
 * @param[in]  u8val 
 * @param[out] int8_t dec value
 */
uint8_t twos_complement_to_decimal8_conversion(int8_t u8val);
sths_err_t i8Shthspf80GpioInterruptInit();
sths_err_t u8GetMotionPresenceValue(stsensor_data_t *pstsensordata);
uint16_t twos_complement_to_decimal_conversion(int16_t val);
#endif