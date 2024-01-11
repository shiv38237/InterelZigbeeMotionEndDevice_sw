/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: Ruchita Shekhada
 *      Date & Time:2023-09-20 && 9:36:00
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
 * \file     sths34pf80.c
 * \brief
 *
 */

/************************************************************************
 * Include Header Files
 ************************************************************************/
#include <stdio.h>
#include <string.h>
#include "sths34pf80.h"
#include "i2c_service.h"
/************************************************************************
 * Define Macros
 ************************************************************************/

/************************************************************************
 * Define Enumeration/Structure/Unions
 ************************************************************************/
/**
 *  default value of sths34pf80 sensor
 */
sths_config_details_t Default_sths34pf80ConfigData = {

    .sths34pf80_sensitivity_level   = DEF_STHS_SENSITIVITY_LEVEL,
    .sths34pf80_LPF_cutoff_frequency_1   = DEF_STHS_LPF_CUTOFF_FREQUENCY_1,
    .sths34pf80_LPF_cutoff_frequency_2   = DEF_STHS_LPF_CUTOFF_FREQUENCY_2,
    .sths34pf80_averaging_of_temp_data   = DEF_STHS_AVERAGING_OF_TEMP_DATA,
    .sths34pf80_ODR   = DEF_STHS_ODR,
    .sths34pf80_presence_thresold   = DEF_STHS_PRESENCE_THRESOLD,
    .sths34pf80_motion_thresold   = DEF_STHS_MOTION_THRESOLD,
    .sths34pf80_ambient_temp_shock_thresold   = DEF_STHS_AMBIENT_TEMP_SHOCK_THRESOLD,
    .sths34pf80_presence_hyst   = DEF_STHS_PRESENCE_HYST,
    .sths34pf80_motion_hyst   = DEF_STHS_MOTION_HYST,
    .sths34pf80_ambient_temp_shock_hyst   = DEF_STHS_AMBIENT_TEMP_SHOCK_HYST,
    .ambient_temp_lower_thresold = DEF_AMBIENT_TEMP_LOWER_THRESOLD,
    .ambient_temp_upper_thresold = DEF_AMBIENT_TEMP_UPPER_THRESOLD,
};

/************************************************************************
 * Define Private Variables
 ************************************************************************/

/************************************************************************
 * Define Global Variables
 ************************************************************************/
sths_config_details_t sths_configData = { 0 }; /*structure to configure all the sths34pf80 reg */
volatile bool STHS34PF80_DataReady_flag = false;
/************************************************************************
 * Define Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Define Global Function Prototypes
 ************************************************************************/
/**
 * @brief : This function return object temp data
 */
int16_t readObjTemp(void)
{
    uint8_t rawData[2]; // 16-bit register data stored here
    rd_shthspf80_reg(STHS34PF80_TOBJECT_L, &rawData[0], sizeof(rawData));   // Added Slight Change in regular expression
    return (int16_t)(((int16_t)rawData[1]) << BITS_PER_U8 | rawData[0]);
}
/**
 * @brief : This function return presense raw data
 */
int16_t readPresence(void)
{
    uint8_t rawData[2];// 16-bit register data stored here
    rd_shthspf80_reg(STHS34PF80_TPRESENCE_L, &rawData[0], sizeof(rawData));
    return (int16_t)(((int16_t)rawData[1]) << BITS_PER_U8 | rawData[0]);
}
/**
 * @brief : This function return motion raw data
 */
int16_t readMotion(void)
{
    uint8_t rawData[2]; // 16-bit register data stored here
    rd_shthspf80_reg(STHS34PF80_TMOTION_L, &rawData[0], sizeof(rawData));
    return (int16_t)(((int16_t)rawData[1]) << BITS_PER_U8 | rawData[0]);
}
/**
 * @brief : This function return ambient shock raw data
 */
int16_t readAmbShock(void)
{
    uint8_t rawData[2]; // 16-bit register data stored here
    rd_shthspf80_reg(STHS34PF80_TAMB_SHOCK_L, &rawData[0], sizeof(rawData));
    return (int16_t)(((int16_t)rawData[1]) << BITS_PER_U8 | rawData[0]);
}
/**
 * @brief : This function return ambient temp data
 */
int16_t readAmbTemp(void)
{
    uint8_t rawData[2]; // 16-bit register data stored here
    rd_shthspf80_reg(STHS34PF80_TAMBIENT_L, &rawData[0], sizeof(rawData));
    return (int16_t)(((int16_t)rawData[1]) << BITS_PER_U8 | rawData[0]);
}
/**
 *  @brief : This function will fill the embedded reg value
 */
sths_err_t shthspf80_embedded_function_reg(uint8_t reg_address ,uint16_t register_value  , uint8_t size)
{
    sths_err_t err;
    uint8_t lower_val , upper_val , Regval;
    uint8_t en_dis_access_efr, wr_en_dis_func_reg ,wr_en_pulsed_latched_mode;
    uint8_t Reg_addr[3] = {0};   
    lower_val = U16_LOWER_U8(register_value); 
    Reg_addr[0] = reg_address;
    if(sizeof(uint16_t) == size)
    {
        upper_val = U16_UPPER_U8(register_value);
        Reg_addr[1] = reg_address + 0x01;
    }
    Reg_addr[2] = STHS34PF80_ALGO_CONFIG; 
    err = rd_shthspf80_reg(STHS34PF80_CTRL2, &Regval, sizeof(Regval));
    if(STHS_ERR_NONE != err)
    {
        
    }else{
        en_dis_access_efr = Regval | EN_DIS_ACCESS_EMBEDDED_FUNC_REG; 
        err = wr_shthspf80_reg(STHS34PF80_CTRL2, &en_dis_access_efr, sizeof(en_dis_access_efr));    // enable access to embedded function register
        if(STHS_ERR_NONE != err)
        {
            
        }else{
            wr_en_dis_func_reg = EN_WRITE_EMBEDDED_FUNC_REG;
            err = wr_shthspf80_reg(STHS34PF80_FUNC_PAGE_RW, &wr_en_dis_func_reg, sizeof(wr_en_dis_func_reg)); // enable write to embedded function register
            if(ESP_OK != err)
            {
                
            }else{
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &Reg_addr[0], sizeof(Reg_addr[0]));   // set embedded register address to PRESENCE_THS_L
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &lower_val, sizeof(lower_val));       // write lower byte
                if(sizeof(uint16_t)  == size)
                {
                    err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &Reg_addr[1], sizeof(Reg_addr[1]));   // set embedded register address to PRESENCE_THS_H
                    err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &upper_val, sizeof(upper_val));           // write higher byte 
                }
                if(STHS_ERR_NONE != err)
                {
                    
                }else{
                    // set algorithm interrupt to pulsed mode (bit 3 = 1) or latch mode (bit 3 = 0)
                    wr_en_pulsed_latched_mode = 0x0c;
                    err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &Reg_addr[2], sizeof(Reg_addr[2])); // set embedded register address to ALGO_CONFIG
                    err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &wr_en_pulsed_latched_mode, sizeof(wr_en_pulsed_latched_mode)); 
                    if(ESP_OK != err)
                    {
                        
                    }else{
                        wr_en_dis_func_reg = DIS_WRITE_EMBEDDED_FUNC_REG;
                        err = wr_shthspf80_reg(STHS34PF80_FUNC_PAGE_RW, &wr_en_dis_func_reg, sizeof(wr_en_dis_func_reg)); // disable write to embedded function register
                        if(STHS_ERR_NONE != err)
                        {
                            
                        }else{
                            Regval = 0x00;   //clear var value before read
                            err = rd_shthspf80_reg(STHS34PF80_CTRL2, &Regval, sizeof(Regval));
                            if(STHS_ERR_NONE != err)
                            {
                                
                            }else{
                                en_dis_access_efr = Regval & ~(EN_DIS_ACCESS_EMBEDDED_FUNC_REG);
                                err = wr_shthspf80_reg(STHS34PF80_CTRL2, &en_dis_access_efr, sizeof(en_dis_access_efr));
                            }
                        }
                    }
                }
            }
        }
    }
    return err;
}
/**
 * @brief : This function give state if new set of data is available or not
 * 
 */
sths_err_t sthspf80_getDataReadyStatus(uint8_t *data_ready)
{
    sths_err_t err;
    uint8_t dr_val;
    err = rd_shthspf80_reg(STHS34PF80_STATUS, &dr_val, sizeof(dr_val));
    if(STHS_ERR_NONE != err)
    {
        
    }
    else
    {
        *data_ready =  dr_val;
    }
    return err;
}
/**
 * @brief Get the Status for presence ,motion , ambient temp shock detection
 */
sths_err_t sthspf80_getFuncStatus(uint8_t *flag_detect)
{
    sths_err_t err;
    uint8_t flag_reg;
    err = rd_shthspf80_reg(STHS34PF80_FUNC_STATUS, &flag_reg, sizeof(flag_reg));
    if(STHS_ERR_NONE != err)
    {
        
    }
    else
    {
        *flag_detect =  flag_reg;
    }
    return err;
}
/**
 * @brief : This function power up and enter in to continuoes mode 
 *          with configure odr
 */
sths_err_t shthspf80_powerUp(uint8_t odr)
{
    sths_err_t err;
    uint8_t Regval , pweup_val;
    err = rd_shthspf80_reg(STHS34PF80_CTRL1, &Regval, sizeof(Regval));
    if(STHS_ERR_NONE != err)
    {
       
    }
    else
    {
        pweup_val = Regval | odr;
        err = wr_shthspf80_reg(STHS34PF80_CTRL1, &pweup_val, sizeof(pweup_val)); //  set bdu = 1 (bit 4 == 1) and odr
    }
    return err;
}
/**
 * @brief : This function reset the algorithem ,use when change odr,
 *          filter value or multiple embedded function register
 */
sths_err_t shthspf80_resetAlgo(void)
{
    sths_err_t err;
    uint8_t Regval;
    uint8_t reset_algo_buf[8] = {0};
    err = rd_shthspf80_reg(STHS34PF80_CTRL2, &Regval, sizeof(Regval));
    if(STHS_ERR_NONE != err)
    {
    
    }else{
        reset_algo_buf[0] = Regval | EN_DIS_ACCESS_EMBEDDED_FUNC_REG;
        err = wr_shthspf80_reg(STHS34PF80_CTRL2, &reset_algo_buf[0], sizeof(reset_algo_buf[0]));                   // enable access to embedded function register
        if(STHS_ERR_NONE != err)
        {
    
        }else{
            reset_algo_buf[1] = EN_WRITE_EMBEDDED_FUNC_REG;
            err = wr_shthspf80_reg(STHS34PF80_FUNC_PAGE_RW, &reset_algo_buf[1],sizeof(reset_algo_buf[1]));                   // enable write to embedded function register
            if(STHS_ERR_NONE != err)
            {
           
            }else{
                reset_algo_buf[2] = STHS34PF80_RESET_ALGO;
                reset_algo_buf[3] = 0x01;
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reset_algo_buf[2], sizeof(reset_algo_buf[2])); // set embedded register address to RESET_ALGO
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &reset_algo_buf[3], sizeof(reset_algo_buf[3]));                  // set ALGO_ENABLE_RESET bit
                if(STHS_ERR_NONE != err)
                {
           
                }else{
                    reset_algo_buf[4] = DIS_WRITE_EMBEDDED_FUNC_REG;
                    err = wr_shthspf80_reg(STHS34PF80_FUNC_PAGE_RW, &reset_algo_buf[4], sizeof(reset_algo_buf[4]));                   // disable write to embedded function register
                    if(STHS_ERR_NONE != err)
                    {
                    
                    }else{
                        Regval = 0x00; //clear var before read
                        err = rd_shthspf80_reg(STHS34PF80_CTRL2, &Regval, sizeof(Regval));
                        if(STHS_ERR_NONE != err)
                        {
                   
                        }else{
                            reset_algo_buf[5]  = Regval & ~(EN_DIS_ACCESS_EMBEDDED_FUNC_REG);
                            err = wr_shthspf80_reg(STHS34PF80_CTRL2, &reset_algo_buf[5] , sizeof(reset_algo_buf[5]));
                        }
                    }
                }
            }
        }     
    }
    return err;
}
/**
 * @brief : This function read all the thresold and hysterisis value from the register 
 */
sths_err_t shthspf80_readAlgoConfig(uint16_t *dest)
{
    sths_err_t err;
    uint8_t en_dis_access_efr, rd_en_dis_func_reg ,Regval;
    uint8_t rawData[2] = {0, 0}; // register data stored here
    uint8_t reg_val[10] = {0};

    reg_val[0] = STHS34PF80_PRESENCE_THS_L;
    reg_val[1] = STHS34PF80_PRESENCE_THS_H;
    reg_val[2] = STHS34PF80_MOTION_THS_L;
    reg_val[3] = STHS34PF80_MOTION_THS_H;
    reg_val[4] = STHS34PF80_TAMB_SHOCK_THS_L;
    reg_val[5] = STHS34PF80_TAMB_SHOCK_THS_H;
    reg_val[6] = STHS34PF80_HYST_MOTION;
    reg_val[7] = STHS34PF80_HYST_PRESENCE; 
    reg_val[8] = STHS34PF80_ALGO_CONFIG;
    reg_val[9] = STHS34PF80_HYST_TAMB_SHOCK;

    err = rd_shthspf80_reg(STHS34PF80_CTRL2, &Regval, sizeof(Regval));
    if(STHS_ERR_NONE != err)
    {

    }else{
        en_dis_access_efr = Regval | EN_DIS_ACCESS_EMBEDDED_FUNC_REG;  
        err = wr_shthspf80_reg(STHS34PF80_CTRL2, &en_dis_access_efr, sizeof(en_dis_access_efr));    // enable access to embedded function register
        if(STHS_ERR_NONE != err)
        {
        
        }else{
            rd_en_dis_func_reg = EN_READ_EMBEDDED_FUNC_REG;
            err = wr_shthspf80_reg(STHS34PF80_FUNC_PAGE_RW, &rd_en_dis_func_reg, sizeof(rd_en_dis_func_reg)); // enable write to embedded function register
            if(STHS_ERR_NONE != err)
            {
            
            }else{ 
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[0], sizeof(reg_val[0])); // set embedded register address to PRESENCE_THS_L
                err = rd_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &rawData[0], sizeof(rawData[0]));
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[1], sizeof(reg_val[1])); // set embedded register address to PRESENCE_THS_H
                err = rd_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &rawData[1], sizeof(rawData[1]));
                if(STHS_ERR_NONE != err)
                {
                    
                }else{
                    dest[0] = (uint16_t)(((uint16_t)rawData[1]) << BITS_PER_U8 | rawData[0]);
                    err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[2], sizeof(reg_val[2])); // set embedded register address to MOTION_THS_L
                    err = rd_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &rawData[0], sizeof(rawData[0]));
                    err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[3], sizeof(reg_val[3])); // set embedded register address to MOTION_THS_H
                    err = rd_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &rawData[1], sizeof(rawData[1]));
                    if(STHS_ERR_NONE != err)
                    {
                        
                    }else{
                        dest[1] = (uint16_t)(((uint16_t)rawData[1]) << BITS_PER_U8 | rawData[0]);
                        err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[4],sizeof(reg_val[4])); // set embedded register address to TAMB_SHOCK_THS_L
                        err = rd_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &rawData[0],sizeof(rawData[0]));
                        err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[5],sizeof(reg_val[5])); // set embedded register address to TAMB_SHOCK_THS_H
                        err = rd_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &rawData[1],sizeof(rawData[1]));
                        if(STHS_ERR_NONE != err)
                        {
                            
                        }else{
                            dest[2] = (uint16_t)(((uint16_t)rawData[1]) << BITS_PER_U8 | rawData[0]);
                            err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR,  &reg_val[6], sizeof(reg_val[6])); // set embedded register address to HYST_MOTION
                            err = rd_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA,  &rawData[0], sizeof(rawData[0]));
                            err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR,  &reg_val[7], sizeof(reg_val[7])); // set embedded register address to HYST_PRESENCE
                            err = rd_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA,  &rawData[1], sizeof(rawData[1]));
                            if(STHS_ERR_NONE != err)
                            {
                                
                            }else{
                                dest[3] = (uint16_t)(((uint16_t)rawData[1]) << BITS_PER_U8 | rawData[0]);
                                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[8], sizeof(reg_val[8])); // set embedded register address to ALGO_CONFIG
                                err = rd_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &rawData[0], sizeof(reg_val[0]));
                                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[9], sizeof(reg_val[9])); // set embedded register address to HYST_TAMB_SHOCK
                                err = rd_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &rawData[1], sizeof(reg_val[1]));
                                if(STHS_ERR_NONE != err)
                                {
                                
                                }else{
                                    dest[4] = (uint16_t)(((uint16_t)rawData[1]) << BITS_PER_U8 | rawData[0]);
                                    rd_en_dis_func_reg = DIS_READ_EMBEDDED_FUNC_REG;
                                    err = wr_shthspf80_reg(STHS34PF80_FUNC_PAGE_RW, &rd_en_dis_func_reg, sizeof(rd_en_dis_func_reg)); // disable write to embedded function register
                                    if(STHS_ERR_NONE != err)
                                    {
                                        
                                    }else{
                                        Regval = 0x00;
                                        err = rd_shthspf80_reg(STHS34PF80_CTRL2, &Regval, sizeof(Regval));
                                        if(STHS_ERR_NONE != err)
                                        {
                                            
                                        }else{
                                            en_dis_access_efr = Regval & ~(EN_DIS_ACCESS_EMBEDDED_FUNC_REG);
                                            err = wr_shthspf80_reg(STHS34PF80_CTRL2, &en_dis_access_efr, sizeof(en_dis_access_efr));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return err;
}

/**
 *  @brief : This function fill the thresold and hysterisis value in register
 */
sths_err_t shthspf80_configAlgo(uint16_t presence_ths, uint16_t motion_ths, uint16_t tamb_shock_ths, uint8_t presence_hyst, uint8_t motion_hyst, uint8_t tamb_shock_hyst)
{
    sths_err_t err;
    uint8_t en_dis_access_efr, wr_en_dis_func_reg, wr_low_pres_ths, wr_high_pres_ths, wr_lsb_mot_ths, wr_msb_mot_ths;
    uint8_t Regval , wr_lsb_tam_sh_ths, wr_msb_tam_sh_ths, wr_en_pulsed_latched_mode;
    uint8_t reg_val[10] = {0};   

    wr_low_pres_ths = U16_LOWER_U8(presence_ths);
    wr_high_pres_ths = U16_UPPER_U8(presence_ths);
    wr_lsb_mot_ths = U16_LOWER_U8(motion_ths);
    wr_msb_mot_ths = U16_UPPER_U8(motion_ths); 
    wr_lsb_tam_sh_ths = U16_LOWER_U8(tamb_shock_ths);
    wr_msb_tam_sh_ths = U16_UPPER_U8(tamb_shock_ths);

    reg_val[0] = STHS34PF80_PRESENCE_THS_L;
    reg_val[1] = STHS34PF80_PRESENCE_THS_H;
    reg_val[2] = STHS34PF80_MOTION_THS_L;
    reg_val[3] = STHS34PF80_MOTION_THS_H;
    reg_val[4] = STHS34PF80_TAMB_SHOCK_THS_L;
    reg_val[5] = STHS34PF80_TAMB_SHOCK_THS_H;
    reg_val[6] = STHS34PF80_HYST_PRESENCE;
    reg_val[7] = STHS34PF80_HYST_MOTION;
    reg_val[8] = STHS34PF80_HYST_TAMB_SHOCK;
    reg_val[9] = STHS34PF80_ALGO_CONFIG; 

    err = rd_shthspf80_reg(STHS34PF80_CTRL2, &Regval, sizeof(Regval));
    if(STHS_ERR_NONE != err)
    {
        
    }else{
        en_dis_access_efr = Regval | EN_DIS_ACCESS_EMBEDDED_FUNC_REG; 
        err = wr_shthspf80_reg(STHS34PF80_CTRL2, &en_dis_access_efr, sizeof(en_dis_access_efr));    // enable access to embedded function register
        if(STHS_ERR_NONE != err)
        {
            
        }else{
            wr_en_dis_func_reg = EN_WRITE_EMBEDDED_FUNC_REG;
            err = wr_shthspf80_reg(STHS34PF80_FUNC_PAGE_RW, &wr_en_dis_func_reg, sizeof(wr_en_dis_func_reg)); // enable write to embedded function register
            if(STHS_ERR_NONE != err)
            {
                
            }else{
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[0], sizeof(reg_val[0]));   // set embedded register address to PRESENCE_THS_L
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &wr_low_pres_ths, sizeof(wr_low_pres_ths));       // write lower byte
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[1], sizeof(reg_val[1]));   // set embedded register address to PRESENCE_THS_H
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &wr_high_pres_ths, sizeof(wr_high_pres_ths));           // write higher byte
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[2], sizeof(reg_val[2]));     // set embedded register address to MOTION_THS_L
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &wr_lsb_mot_ths, sizeof(wr_lsb_mot_ths));         // write lower byte
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[3], sizeof(reg_val[3]));     // set embedded register address to MOTION_THS_H
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &wr_msb_mot_ths, sizeof(wr_msb_mot_ths));             // write higher byte
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[4], sizeof(reg_val[4])); // set embedded register address to TAMB_SHOCK_THS_L
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &wr_lsb_tam_sh_ths, sizeof(wr_lsb_tam_sh_ths));     // write lower byte
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[5], sizeof(reg_val[5])); // set embedded register address to TAMB_SHOCK_THS_H
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &wr_msb_tam_sh_ths, sizeof(wr_msb_tam_sh_ths));         // write higher byte
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[6], sizeof(reg_val[6]));   // set embedded register address to HYST_PRESNCE
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &presence_hyst, sizeof(presence_hyst));              // write byte
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[7], sizeof(reg_val[7]));     // set embedded register address to HYST_MOTION
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &motion_hyst, sizeof(motion_hyst));                // write byte
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[8], sizeof(reg_val[8])); // set embedded register address to HYST_TAMB_SHOCK
                err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &tamb_shock_hyst, sizeof(tamb_shock_hyst));            // write byte 
                if(STHS_ERR_NONE != err)
                {
                    
                }else{
                    // set algorithm interrupt to pulsed mode (bit 3 = 1) or latch mode (bit 3 = 0)
                    wr_en_pulsed_latched_mode = EMBEDDED_ALGO_CONFIGURATION;
                    err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_ADDR, &reg_val[9], sizeof(reg_val[9])); // set embedded register address to ALGO_CONFIG
                    err = wr_shthspf80_reg(STHS34PF80_FUNC_CFG_DATA, &wr_en_pulsed_latched_mode, sizeof(wr_en_pulsed_latched_mode)); 
                    if(STHS_ERR_NONE != err)
                    {
                        
                    }else{
                        wr_en_dis_func_reg = DIS_WRITE_EMBEDDED_FUNC_REG;
                        err = wr_shthspf80_reg(STHS34PF80_FUNC_PAGE_RW, &wr_en_dis_func_reg, sizeof(wr_en_dis_func_reg)); // disable write to embedded function register
                        if(STHS_ERR_NONE != err)
                        {
                            
                        }else{
                            Regval = 0x00;
                            err = rd_shthspf80_reg(STHS34PF80_CTRL2, &Regval, sizeof(Regval));
                            if(STHS_ERR_NONE != err)
                            {
                                
                            }else{
                                en_dis_access_efr = Regval & ~(EN_DIS_ACCESS_EMBEDDED_FUNC_REG);
                                err = wr_shthspf80_reg(STHS34PF80_CTRL2, &en_dis_access_efr, sizeof(en_dis_access_efr));
                            }
                        }
                    }
                }
            }
        }
    }
    return err;
}
/**
 * @brief : This function set number of average sample for ambient and object temperature
 */
sths_err_t shthspf80_config(uint8_t set_avgt_avgtmos, uint8_t gain)
{
    sths_err_t err;
    uint8_t Regval, set_temp_gain, set_cfg_dry_val;
    err = wr_shthspf80_reg(STHS34PF80_AVG_TRIM, &set_avgt_avgtmos, sizeof(set_avgt_avgtmos));
    if(STHS_ERR_NONE != err)
    {
        
    }else{
        err = rd_shthspf80_reg(STHS34PF80_CTRL0, &(Regval), sizeof(Regval));
        if(STHS_ERR_NONE != err)
        {
            
        }else{
            set_temp_gain = Regval | gain << 4;
            err = wr_shthspf80_reg(STHS34PF80_CTRL0, &set_temp_gain, sizeof(set_temp_gain)); 
            if(STHS_ERR_NONE != err)
            {
                
            }else{
                // Configure interrupt behavior
                // select active HIGH (bit 7 == 0), push-pull (bit 6 == 0), INT_MSK[5:3]= 0x00(default) 
                //latched mode (bit 2 == 1)  IEN[1:0] : 01 dataready
                set_cfg_dry_val = CTRL3_INTERRUPT_CONFIGURATION;
                err = wr_shthspf80_reg(STHS34PF80_CTRL3, &set_cfg_dry_val, sizeof(set_cfg_dry_val)); 
            }
        }
    }
    return err;
}
/**
 * @brief : This function set LPF 1 and LPF 2 register ,cutoff frequency
 */
sths_err_t sthspf80_setLowpassFilters(uint8_t lpf_1, uint8_t lpf_2)
{
    sths_err_t err;
    err = wr_shthspf80_reg(STHS34PF80_LPF1, &lpf_1, sizeof(lpf_1));
    if(STHS_ERR_NONE != err)
    {

    }else{ 
        err = wr_shthspf80_reg(STHS34PF80_LPF2, &lpf_2, sizeof(lpf_2));
    }
    return err;
}
/**
 * @brief : This fuction write new  the sensitivity data in register
*/
sths_err_t sthspf80_writeSenseData(int16_t sensitivity_data)
{
    sths_err_t err;
    uint8_t Regval;
    int8_t c_data = (int8_t)(sensitivity_data - 2048)/16;
    Regval = twos_complement_to_decimal8_conversion(c_data);
    err = wr_shthspf80_reg(STHS34PF80_SENS_DATA, &Regval, sizeof(Regval));
    return err;
}
/**
 * @brief :This fuction give the sensitivity data of  sths34pf80 sensor
 */
sths_err_t sthspf80_readSenseData(int8_t *sensitivity_data)
{
    sths_err_t err;
    uint8_t Regval = 0x00;
    err = rd_shthspf80_reg(STHS34PF80_SENS_DATA, &(Regval), sizeof(Regval));
    if(STHS_ERR_NONE != err)
    {
    
    }
    else
    {
        *sensitivity_data =  Regval;
    }
    return err;
} 
/**
 * @brief : This fuction put sths34pf80 sensor in to power down mode
 */
sths_err_t shthspf80_powerDown_Mode(void) 
{
    sths_err_t err;
    uint8_t Regval,odr_data;
    uint8_t Drdy_stat = 0XFF;
    err = rd_shthspf80_reg(STHS34PF80_FUNC_STATUS, &Regval, sizeof(Regval));                   // Reset STATUS register to 0
    if(STHS_ERR_NONE != err)   
    {
       
    }else{
        err = rd_shthspf80_reg(STHS34PF80_STATUS, &Drdy_stat, sizeof(Drdy_stat));
        if(STHS_ERR_NONE != err)   
        {
            
        }else{
            while( (Drdy_stat & SET_STATUS_DRDY_BIT) )
            {
                err = rd_shthspf80_reg(STHS34PF80_STATUS, &Drdy_stat, sizeof(Drdy_stat));
            } // wait for STATUS DR bit --> 0
            Regval = 0x00;
            err = rd_shthspf80_reg(STHS34PF80_CTRL1,  &Regval, sizeof(Regval)); 
            if(STHS_ERR_NONE != err)   
            {
                
            }else{
                odr_data = Regval & ~(0x07); 
                err =  wr_shthspf80_reg(STHS34PF80_CTRL1, &odr_data, sizeof(odr_data) ); 
                if(STHS_ERR_NONE != err)   
                {
                    
                }else{ 
                    Regval = 0x00;
                    err = rd_shthspf80_reg(STHS34PF80_FUNC_STATUS, &(Regval), sizeof(Regval)); // reset DR bit of STATUS to 0       
                }     
            }     
        }     
    } 
    return err;    
}
/**
 * @brief :This function reset sthspf80 sensor
 * 
 */
sths_err_t shthspf80_reset(void)
{
    sths_err_t err;
    uint8_t Regval,sensor_reboot;
    err = rd_shthspf80_reg(STHS34PF80_CTRL2, &Regval, sizeof(Regval));
    if(STHS_ERR_NONE != err)
    {
        
    }else{
        sensor_reboot = Regval | SET_CTRL2_BOOT_BIT;                              // set bit 7 to force device reset, wait 2.5 ms for reboot
        err = wr_shthspf80_reg(STHS34PF80_CTRL2, &sensor_reboot, sizeof(sensor_reboot)); 
    } 
    return err; 
}
/**
 * @brief : This function return converted decimal value of 2's
 *          complement value
 */
uint8_t twos_complement_to_decimal8_conversion(int8_t u8val)
{
    uint8_t sign_mask = 0x80;
    // if positive
    if ( (u8val & sign_mask) == 0 ) {
        return u8val;
    //  if negative
    } else {
        // invert all bits, add one, and make negative
        return -(~u8val + 1);
    }
}
/* Read contents of a STHS34PF80 register
---------------------------------------------------------------------------*/
sths_err_t rd_shthspf80_reg( uint8_t reg, uint8_t *pdata, uint8_t count )
{
    sths_err_t err;
	err = i2c_master_read_slave_reg( I2C_MASTER_NUM, STHS34PF80_I2C_ADDR,  reg, pdata, count);
    if(STHS_ERR_NONE != err) 
    {
        err = STHS_ERR_I2C;
    }  
    return err;
}
/* Write value to specified STHS34PF80 register
---------------------------------------------------------------------------*/
sths_err_t wr_shthspf80_reg( uint8_t reg, uint8_t *pdata, uint8_t count )
{
    sths_err_t err;
	err = i2c_master_write_slave_reg( I2C_MASTER_NUM, STHS34PF80_I2C_ADDR,  reg, pdata, count );
    if(STHS_ERR_NONE != err) 
    {
        err = STHS_ERR_I2C;
    }  
    return err;
}
/**
 * @brief :This function configure all the register of sthspf80 sensor
 */
sths_err_t i8Shthspf80SensorInit(sths_config_details_t sths_configurationData)
{
    sths_err_t err;
    uint8_t sth_dev_id;
    int8_t temp_sens;
    int16_t ObjSense = 0;
    uint16_t sths_algoConfig[5] = {0, 0, 0, 0, 0};
    i8Shthspf80GpioInterruptInit();
    err = rd_shthspf80_reg(WHO_AM_I_REG, &(sth_dev_id), sizeof(sth_dev_id));
    if(STHS_ERR_NONE == err)
    {
        if (sth_dev_id == STHS34PF80_DEVICE_ID)
        {
            err = shthspf80_reset();            // reset all registers to default, sensor enters power down after reset, so next line is redundant
            if(STHS_ERR_NONE != err)   
            {
            
            }else{
                err = shthspf80_powerDown_Mode();        // power down to configure embedded algorithms or wait for one-shot command
                if(STHS_ERR_NONE != err)   
                {
                    
                }else{
                    // err = sthspf80_writeSenseData(sths_configurationData.sths34pf80_sensitivity_level);
                    // if(STHS_ERR_NONE != err)
                    // {
                    //     printf("Failed to write sens data during configuration , return Status: %d \n", err);  
                    // }else{
                        err = sthspf80_readSenseData(&temp_sens);
                        if(STHS_ERR_NONE != err)
                        {
                            
                        }else{
                            // retreive ObjTemp sensitivity
                            ObjSense = (temp_sens * 16) + 2048;
                            printf("Object Sense Data (LSB/oC) = %d\n", ObjSense);
                            err = sthspf80_setLowpassFilters(sths_configurationData.sths34pf80_LPF_cutoff_frequency_1, 
                                                                sths_configurationData.sths34pf80_LPF_cutoff_frequency_2);
                            if(STHS_ERR_NONE != err)
                            {
                                
                            }else{
                                err = shthspf80_config(sths_configurationData.sths34pf80_averaging_of_temp_data, NORMAL_MODE);
                                if(STHS_ERR_NONE != err)
                                {
                                
                                }else{
                                    err = shthspf80_configAlgo(sths_configurationData.sths34pf80_presence_thresold, sths_configurationData.sths34pf80_motion_thresold, 
                                                                sths_configurationData.sths34pf80_ambient_temp_shock_thresold, sths_configurationData.sths34pf80_presence_hyst, 
                                                                sths_configurationData.sths34pf80_motion_hyst, sths_configurationData.sths34pf80_ambient_temp_shock_hyst);
                                    if(STHS_ERR_NONE != err)
                                    {
                                        
                                    }
                                    else
                                    {
                                        err = shthspf80_readAlgoConfig(sths_algoConfig);                             // check that algorithm is configured properly
                                        if(STHS_ERR_NONE != err)
                                        {
                                            
                                        }else{ 
                                            printf("Presence threshold %d \n",sths_algoConfig[0]);
                                            printf("Presence hysteresis %d \n",sths_algoConfig[3] >> 8);
                                            err = shthspf80_resetAlgo(); 
                                            if(STHS_ERR_NONE != err)
                                            {
                                                
                                            }else{
                                                err = shthspf80_powerUp(sths_configurationData.sths34pf80_ODR);                                       // power up and run continuously at odr
                                                if(STHS_ERR_NONE != err)
                                                {
                                                
                                                }else{
                                                   printf("Starting continuous mode!\n");
                                                }
                                            }     
                                        }
                                    }
                                }
                            }
                        }
                    //}
                }     
            }     
        }
        else
        {
            printf("STSH34PF80 ID:0x%X !!!! (NOT correct; should be 0xD3)", sth_dev_id);
            err = STHS_ERR_ID;
        }
    }
    else
    {
        err = STHS_ERR_I2C;
    }
    return err;
}
static void IRAM_ATTR myinthandler(void* arg)
{
    STHS34PF80_DataReady_flag = true;
}
sths_err_t i8Shthspf80GpioInterruptInit()
{
    esp_err_t err;
    gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL << STHS34PF80_INT_PIN);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
	err = gpio_config(&io_conf);//configure GPIO with the given settings
    if (ESP_OK != err) {
        printf("Failed to configure GPIO Input, return Status: %d", err);
    }
    else {
    }
    gpio_install_isr_service(0);
    gpio_isr_handler_add(STHS34PF80_INT_PIN, myinthandler, (void *)STHS34PF80_INT_PIN);
    return err;    
}
uint16_t twos_complement_to_decimal_conversion(int16_t val)
{
    uint16_t sign_mask = 0x8000;

    // if positive
    if ( (val & sign_mask) == 0 ) {
        return val;
    //  if negative
    } else {
        // invert all bits, add one, and make negative
        return -(~val + 1);
    }
}
sths_err_t u8GetMotionPresenceValue(stsensor_data_t *pstsensordata)
{
    uint8_t i8GetDrDyStatus = 0,i8PresMotTambStatusFlag = 0, u8readstatus = 0;
    uint8_t err;
    //int16_t i16Presence = 0, i16Motion = 0, i16AmbTemp = 0, i16ObjTemp = 0, i16CObjTemp = 0, i16AmbShock = 0;
    err = sthspf80_getDataReadyStatus(&i8GetDrDyStatus);
    if (i8GetDrDyStatus & 0x04) {
            pstsensordata->i16Presence = readPresence();
            pstsensordata->i16Motion = readMotion();
            pstsensordata->i16Motion = readMotion();
            pstsensordata->i16AmbTemp = readAmbTemp();
            pstsensordata->i16ObjTemp = readObjTemp();
            pstsensordata->i16CObjTemp = twos_complement_to_decimal_conversion(pstsensordata->i16ObjTemp);
            pstsensordata->i16AmbShock = readAmbShock();
            printf("\nSTHS34PF80: ObjTemp: [%4d] AmbTemp:[%4d] Presence:[%4d] Motion:[%4d] AmbShock:[%4d]\n", pstsensordata->i16CObjTemp,pstsensordata->i16AmbTemp,pstsensordata->i16Presence,pstsensordata->i16Motion,pstsensordata->i16AmbShock);
            printf("Ambient Temperature = [%.2f C] Object Temperature = [%.2f C] Converted Temperature = [%.2f C]\n", (float)pstsensordata->i16AmbTemp / 100.0f,(float)pstsensordata->i16ObjTemp / 2000.0f,(float)pstsensordata->i16CObjTemp / 2000.0f);
            err  = sthspf80_getFuncStatus(&i8PresMotTambStatusFlag);
            if (i8PresMotTambStatusFlag & 0x04) {
               // printf("Presence detected !\n");
                pstsensordata->u8PresenceFlag = true;
            } else {
                pstsensordata->u8PresenceFlag = false;
            }
            if (i8PresMotTambStatusFlag & 0x02) {
               // printf("Motion detected !\n");
                pstsensordata->u8MotionFlag = true; 
            } else {
                pstsensordata->u8MotionFlag = false;
            }
            if (i8PresMotTambStatusFlag & 0x01) {
               // printf("T Shock detected !\n");
            }
            u8readstatus = 0x01;
    } else {
        u8readstatus = 0x00;
    }
    return u8readstatus;   
}
