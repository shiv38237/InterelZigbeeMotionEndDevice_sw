#include <stdint.h>

typedef struct esp_zb_basic_cluster_cfig_s {
    uint8_t  zcl_version;                       
    char     manufacturer_name[32];
    char     model_identifier[32];
    char     datecode[32];
    uint8_t  powersource;                      
} esp_zb_basic_cluster_cfig_t;

typedef struct esp_zb_identify_cluster_cfig_s {
    uint16_t  identify_time;                   
} esp_zb_identify_cluster_cfig_t;

typedef struct esp_zb_ias_zone_cluster_cfig_s {
    uint8_t  zone_state;                             
    uint16_t  zone_type;                             
    uint8_t   zone_status;                           
    esp_zb_ieee_addr_t ias_cie_Address;                 
    uint8_t   zone_id;                               
} esp_zb_ias_zone_cluster_cfig_t;

typedef struct esp_zb_power_config_cluster_cfig_s {
    uint8_t     battery_voltage;                       
    uint8_t     battery_size;                        
    uint8_t     battery_quantity;                   
    uint8_t     battery_rated_voltage;                   
    uint8_t     battery_voltage_min_threshold;        
    uint32_t    battery_alarm_state;                 
} esp_zb_power_config_cluster_cfig_t;

typedef struct esp_zb_poll_control_cluster_cfig_s {
    uint32_t     check_in_interval;                       
    uint32_t     long_poll_interval;                          
    uint16_t     short_poll_interval;                    
    uint16_t     fast_poll_timeout;                  
} esp_zb_poll_control_cluster_cfig_t;

typedef struct esp_zb_ota_cluster_cfig_s {
    esp_zb_ieee_addr_t upgrade_server_id;
    uint32_t file_offset; 
    uint32_t current_file_version;            
    uint16_t current_zigbee_stack_version;           
    uint32_t downloaded_file_version;             
    uint16_t downloaded_zigbee_stack_version;             
    uint8_t  image_upgrade_status;                 
    uint16_t manufacturer_id;     
    uint16_t image_type_id;           
    uint16_t minimum_block_request_delay;           
} esp_zb_ota_cluster_cfig_t;

typedef struct esp_zb_time_cluster_cfig_s {
    uint32_t time;                              
    uint8_t time_status;                       
} esp_zb_time_cluster_cfig_t;

typedef struct esp_zb_binary_input_cluster_cfig_s {
    char     description[6];
    bool     presentvalue;  
    bool     outofservice;  
    uint8_t  reliabillity;                         
    uint8_t  statusflag;                             
} esp_zb_binary_input_cluster_cfig_t;

typedef struct esp_zb_occupancy_sensing_cluster_cfig_s {
    uint8_t occupancy;                         
    uint8_t sensor_type;                       
    uint16_t occupied_to_unoccupied_delay;                       
    uint16_t unoccupied_to_occupied_delay;                      
    uint8_t unoccupied_to_occupied_threshold;                 
} esp_zb_occupancy_sensing_cluster_cfig_t;

typedef struct esp_zb_temperature_meas_cluster_cfig_s {
    int16_t measured_value;                     
    int16_t min_measured_value;                          
    int16_t max_measured_value;                         
} esp_zb_temperature_meas_cluster_cfig_t;

typedef struct esp_zb_illuminance_meas_cluster_cfig_s {
    uint16_t measured_value;                    
    uint16_t min_measured_value;                        
    uint16_t max_measured_value;                         
    uint8_t light_sensor_type;                         
} esp_zb_illuminance_meas_cluster_cfig_t;

