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
 * \file     pyq1548.c
 * \brief
 *
 */

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include "pyq1548.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
/************************************************************************
 * Define Macros
 ************************************************************************/

/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/
/**
 *  default value of pyq sensor
 */
PYQ1548_config_details_t Default_pyq1548ConfigData = {
    .PYQconf_data = DEF_PYQ_MODE_CONFIGURATION,
    .pyq_lower_thresold = DEF_PYQ_LOWER_THRESOLD,
    .pyq_upper_thresold = DEF_PYQ_UPPER_THRESOLD,
};
/************************************************************************
 * Define Private Variables
 ************************************************************************/

/************************************************************************
 * Define Global Variables
 ************************************************************************/
int PIRval = 0; // PIR signal
unsigned long statcfg = 0; // status and configuration register
PYQ1548_config_details_t pyq_configData;
/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/
void vWritePyqRegval(unsigned long regval)
{
    int i;
    unsigned char nextbit;
    unsigned long regmask = 0x1000000;
    gpio_set_level(DLINK_GPIO_PIN, 0);
    gpio_set_level(SERIAL_IN_GPIO_PIN, 0);
    esp_rom_delay_us(5);
    for(i=0;i < 25;i++){
        nextbit = (regval&regmask)!=0;
        regmask >>= 1;
        gpio_set_level(SERIAL_IN_GPIO_PIN, 0);
        gpio_set_level(SERIAL_IN_GPIO_PIN, 1);
        gpio_set_level(SERIAL_IN_GPIO_PIN, nextbit);
        esp_rom_delay_us(100);
    }
    gpio_set_level(SERIAL_IN_GPIO_PIN, 0);
    esp_rom_delay_us(650);
}

int i32ReadLowPowerPyro(void) 
{
    int i;
    unsigned int uibitmask;
    unsigned long ulbitmask;
    gpio_set_level(DLINK_GPIO_PIN, 1);
    esp_rom_delay_us(140);
    uibitmask = 0x4000; // Set BitPos
    PIRval = 0;
    ulbitmask = 0x1000000; // Set BitPos
    statcfg = 0;
    for (i=0; i < 40; i++)
    {
        gpio_set_level(DLINK_GPIO_PIN, 0);// Set DL = Low, duration must be > 200 ns (tL)
        gpio_set_level(DLINK_GPIO_PIN, 1);// Set DL = High, duration must be > 200 ns (tH) 
        gpio_set_direction(DLINK_GPIO_PIN, GPIO_MODE_INPUT); // Configure DL as Input
        esp_rom_delay_us(8); // Wait for stable low signal
        if(i< 15) {
            if (gpio_get_level(DLINK_GPIO_PIN)) // If DL High set masked bit in PIRVal
                PIRval |= uibitmask;
            uibitmask>>=1; 
        } else {
            if (gpio_get_level(DLINK_GPIO_PIN))  // If DL High set masked bit
                statcfg |= ulbitmask;
            ulbitmask>>=1; 
        }
        gpio_set_level(DLINK_GPIO_PIN, gpio_get_level(DLINK_GPIO_PIN));
        gpio_set_direction(DLINK_GPIO_PIN, GPIO_MODE_OUTPUT);
        esp_rom_delay_us(4);
    } 
    PIRval &= 0x3FFF; // clear unused bit
    if (!(statcfg & 0x60)){
        if(PIRval & 0x2000) 
            PIRval -= 0x4000;
    }
    printf("PIR = [ %6d ] - [%08X] config = %08X \n",PIRval,PIRval,(int)statcfg);
    return PIRval;
}
void vPyqMotionDetectionEnable()
{
    //gpio_set_level(MOTION_INDICATOR__GPIO, 1);
}

void vPyqMotionDetectionDisable()
{
    //gpio_set_level(MOTION_INDICATOR__GPIO, 0);
}

void vPyqGPIO_init()
{
    // Initialize the GPIO subsystem
    gpio_config_t io_conf;

    // Configure the SerialIn GPIO pin
    io_conf.pin_bit_mask = (1ULL << SERIAL_IN_GPIO_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Configure the Dlink GPIO pin
    io_conf.pin_bit_mask = (1ULL << DLINK_GPIO_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}