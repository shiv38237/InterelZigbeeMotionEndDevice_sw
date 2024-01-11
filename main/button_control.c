/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *		@author: Ruchita Shekhada
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
/** \file button_control.c
 * \brief
 *
 **/
/************************************************************************
 * INCLUDES
 ************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "button_control.h"
/****************************************************************************
 * MACROS
 ***************************************************************************/

/****************************************************************************
 * TYPES
 ****************************************************************************/

/****************************************************************************
 * GLOBAL VARIABLES
 ****************************************************************************/
QueueHandle_t gpio_evt_queue;
/****************************************************************************
 * PRIVATE FUNCTIONS
 ****************************************************************************/

/****************************************************************************
 * FUNCTIONS
 ****************************************************************************/

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
 /**
 * @brief: Initializes interrupt handler of button gpio
 *
 * @param[in] data     void     
 *
 * @param[out] void
 */

void Button_init (void)
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
	uint8_t error = gpio_config(&io_conf);//configure GPIO with the given settings
	if(error != 0){
	printf("error configuring BUTTON gpio\n");
	}
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	if (gpio_evt_queue == NULL)
	{
		ESP_LOGE(" ", "Failed to create gpio event queue \n");
		vQueueDelete(gpio_evt_queue);
	}
    else
    {
    //create a queue to handle gpio event from isr
		gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	    gpio_isr_handler_add(COMMISSIONING_SWITCH_PIN, gpio_isr_handler, (void*) COMMISSIONING_SWITCH_PIN);
    }
}



/****************************************************************************
 * END OF FILE
 ****************************************************************************/