/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: 
 *      Date & Time:  Date & Time:2023-010-06 && 2:59:00
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
 * \file     led_control.c
 * \brief
 *
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_control.h"

void LED_init(void)
{
    uint8_t err;
    gpio_config_t io_conf = {0U};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = 1ULL << LED_PIN;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pull_up_en = 0U;
    io_conf.pull_down_en = 0U;
    err = gpio_config(&io_conf);
    if (0 != err)
    {
        ESP_LOGE(" ", "Failed to init led as Output, input return Status: %d", err);
    }
}

void led_gpio_set_level(int gpio_level)
{
    gpio_set_level(LED_PIN,gpio_level);
}


