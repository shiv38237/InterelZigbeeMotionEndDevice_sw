/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *		@author: Jaimit Shah
 *
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
 * \file     i2c_service.h
 * \brief   
 */

#ifndef I2C_SERVICE_H
#define I2C_SERVICE_H

/**
 * \headerfile ""
 */
/************************************************************************
 * Include Header Files
 ************************************************************************/
#include "esp_attr.h"
#include "esp_system.h"
#include "driver/i2c.h"
/************************************************************************
 * Define Macros
 ************************************************************************/
#define FREQ_100K 10000U            /*!< Static uint32_t i2c_frequency = 100000; */
#define I2C_PORT I2C_NUM_0           /*!< Static i2c_port_t i2c_port = I2C_NUM_0; */
#define I2C_MASTER_TX_BUF_DISABLE 0U /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0U /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE   /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ     /*!< I2C master read */
#define ACK_CHECK_EN 0x1U            /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS 0x0U           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0U                 /*!<I2C ack value */
#define NACK_VAL 0x1U                /*!< I2C nack value */
#define I2C_SCL GPIO_NUM_13                   /*!< SCL pin number (IC connected) */
#define I2C_SDA GPIO_NUM_15                  /*!< SDA pin number (IC connected) */
#define I2C_TIMEOUT 1048575U
#define I2C_MASTER_NUM  I2C_PORT//I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 40000U//CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define STH_LUX_SCL_PIN                     GPIO_NUM_7     /*!< gpio number for I2C master clock */
#define STH_LUX_SDA_PIN                     GPIO_NUM_6     /*!< gpio number for I2C master data  */
#define STH_INT_PIN                         GPIO_NUM_5 //reg remain
#define STH_CS_PIN                          GPIO_NUM_13
#define portTICK_RATE_MS                    1U
/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/

/************************************************************************
 * Define Private Variables
 ************************************************************************/

/************************************************************************
 * Define Global Variables
 ************************************************************************/
// SemaphoreHandle_t i2cMutexlock;      /*!< Mutex lock for access global data */
/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/

/**
 * @brief 
 * 
 * @return esp_err_t 
 */
esp_err_t i2c_master_init(void);
/**
 * @brief: API to read slave reg value 
 *
 * @param[in] i2c_num   i2c_port
 * @param[in] i2c_addr	i2c_slave_Addr
 * @param[in] i2c_reg	reg address to read data of that register
 * @param[in] size	    size of read data
 *
 * @param[out] esp_err_t 	Return ESP_OK or ESP_FAIL
 * @param[out] data_rd 	    data of reg address 
 */
esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size);
/**
 * @brief: API to write  slave reg value 
 *
 * @param[in] i2c_num   i2c_port
 * @param[in] i2c_addr	i2c_slave_Addr
 * @param[in] i2c_reg	reg address to write 
 * @param[in] size	    size of write data
 * @param[in] data_wr	value to write in reg address
 *
 * @param[out] esp_err_t 	Return ESP_OK or ESP_FAIL
 */
esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size);
/****************************************************************************
 * END OF FILE
 ****************************************************************************/
#endif
