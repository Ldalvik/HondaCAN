#pragma once

#include <cstdint>

// CAN FRAME IDs from a Honda Accord 2018 .dbc
// Most of these should work across generations- especially basic LX trim data. Confirmed working with 9th gen.
#define KINEMATICS_ID 148
#define BRAKE_HOLD_ID 232
#define STEERING_CONTROL_ID 228
#define BOSCH_SUPPLEMENTAL_1_ID 229
#define GAS_PEDAL_2_ID 304 // Seems to be platform-agnostic
#define GAS_PEDAL_ID 316 // Should exist on Nidec
#define ENGINE_DATA_ID 344
#define POWERTRAIN_DATA_ID 380
#define GEARBOX_15T_ID 401
#define GEARBOX_ID 419
#define VSA_STATUS_ID 420
#define STEER_MOTOR_TORQUE_ID 427
#define STEER_STATUS_ID 399
#define WHEEL_SPEEDS_ID 464
#define EPB_STATUS_ID 450
#define VEHICLE_DYNAMICS_ID 490
#define ACC_CONTROL_ID 479
#define ROUGH_WHEEL_SPEED_ID 597
#define LEFT_LANE_LINE_1_ID 576
#define LEFT_LANE_LINE_2_ID 577
#define RIGHT_LANE_LINE_1_ID 579
#define RIGHT_LANE_LINE_2_ID 580
#define ADJACENT_LEFT_LANE_LINE_1_ID 582
#define ADJACENT_LEFT_LANE_LINE_2_ID 583
#define ADJACENT_RIGHT_LANE_LINE_1_ID 585
#define ADJACENT_RIGHT_LANE_LINE_2_ID 586
#define XXX_16_ID 545
#define SCM_BUTTONS_ID 662
#define SCM_FEEDBACK_ID 806
#define CAMERA_MESSAGES_ID 862
#define RADAR_HUD_ID 927
#define SEATBELT_STATUS_ID 773
#define CAR_SPEED_ID 777
#define ACC_HUD_ID 780
#define CRUISE_ID 804
#define STALK_STATUS_ID 884
#define STALK_STATUS_2_ID 891
#define DOORS_STATUS_ID 1029
#define LKAS_HUD_A_ID 13274
#define LKAS_HUD_B_ID 13275

// Enum for GEARBOX_15T.GEAR_SHIFTER (1.5T)
typedef enum {
    GEAR_L = 32,
    GEAR_S = 16,
    GEAR_D = 8,
    GEAR_N = 4,
    GEAR_R = 2,
    GEAR_P = 1
} GEAR_SHIFTER_15T_t;

// // Enum for GEARBOX_15T.GEAR (1.5T)
// typedef enum {
//     GEAR_L_15T = 7,
//     GEAR_S_15T = 10,
//     GEAR_D_15T = 4,
//     GEAR_N_15T = 3,
//     GEAR_R_15T = 2,
//     GEAR_P_15T = 1
// } GEAR_15T_t;

// // Enum for GEARBOX.GEAR_SHIFTER
// typedef enum {
//     GEAR_SHIFTER_S = 2,
//     GEAR_SHIFTER_D = 32,
//     GEAR_SHIFTER_N = 16,
//     GEAR_SHIFTER_R = 8,
//     GEAR_SHIFTER_P = 4
// } GEAR_SHIFTER_t;

// Enum for GEARBOX.GEAR
// typedef enum {
//     GEAR_S = 26,
//     GEAR_D = 20,
//     GEAR_N = 19,
//     GEAR_R = 18,
//     GEAR_P = 17
// } GEAR_t;

// Enum for XXX_16.ECON_MODE
typedef enum {
    ECON_ON_2_OFF = 0, 
    ECON_ON_2_ON = 3
} ECON_ON_2_t;

// Enum for SCM_BUTTONS.CRUISE_BUTTONS
typedef enum {
    CRUISE_BUTTON_NONE = 0, 
    CRUISE_BUTTON_MAIN = 1,      
    CRUISE_BUTTON_CANCEL = 2, 
    CRUISE_BUTTON_DECEL_SET = 3, 
    CRUISE_BUTTON_ACCEL_RES = 4, 
    CRUISE_BUTTON_TBD_5 = 5,  
    CRUISE_BUTTON_TBD_6 = 6,   
    CRUISE_BUTTON_TBD_7 = 7
} CRUISE_BUTTONS_t;

// Enum for SCM_BUTTONS.CRUISE_SETTING
typedef enum {
    CRUISE_SETTING_NONE = 0, 
    CRUISE_SETTING_LKAS_BUTTON = 1, 
    CRUISE_SETTING_TBD = 2,
    CRUISE_SETTING_DISTANCE_ADJ = 3 
} CRUISE_SETTING_t;

// Enum for SCM_FEEDBACK.CBMS_BUTTON
typedef enum {
    CMBS_BUTTON_RELEASED = 0,
    CMBS_BUTTON_PRESSED = 3  
} CMBS_BUTTON_t;

// Enum for EPB_STATUS.EPB_STATE
typedef enum {
    EPB_STATE_OFF = 0,
    EPB_STATE_ENGAGING = 1,
    EPB_STATE_DISENGAGING = 2,
    EPB_STATE_ON = 3
} EPB_STATE_t;

// Enum for LKAS_HUD.BEEP
typedef enum {
    BEEP_NO_BEEP = 0,         
    BEEP_REPEATED_BEEP = 1,   
    BEEP_TRIPLE_BEEP = 2,     
    BEEP_SINGLE_BEEP = 3      
} BEEP_t;

// Enum for STEER_STATUS.STEER_STATUS
typedef enum {
    STEER_STATUS_NORMAL = 0,           
    STEER_STATUS_NO_TORQUE_ALERT_1 = 2,
    STEER_STATUS_LOW_SPEED_LOCKOUT = 3,
    STEER_STATUS_NO_TORQUE_ALERT_2 = 4,
    STEER_STATUS_FAULT_1 = 5,          
    STEER_STATUS_TMP_FAULT = 6         
} STEER_STATUS_t;

// ID: 148
// Module: XXX
typedef struct {
    float LAT_ACCEL;                 // 7|10@0+ (-0.035,17.92) [-20|20] "m/s2" EON
    float LONG_ACCEL;                // 25|10@0+ (-0.035,17.92) [-20|20] "m/s2" EON
    uint8_t CHECKSUM;                // 59|4@0+ (1,0) [0|3] EON
    uint8_t COUNTER;                 // 61|2@0+ (1,0) [0|3] EON
} KINEMATICS;

// ID: 228
typedef struct {
    uint8_t STEER_TORQUE_REQUEST;    // 23|1@0+ (1,0) [0|1] EPS
    uint8_t SET_ME_X00;              // 22|7@0+ (1,0) [0|127] EPS
    uint8_t SET_ME_X00_2;            // 31|8@0+ (1,0) [0|0] EPS
    int16_t STEER_TORQUE;            // 7|16@0- (1,0) [-4096|4096] EPS
    uint8_t STEER_DOWN_TO_ZERO;      // 38|1@0+ (1,0) [0|1] EPS
    uint8_t COUNTER;                 // 37|2@0+ (1,0) [0|3] EPS
    uint8_t CHECKSUM;                // 35|4@0+ (1,0) [0|15] EPS
} STEERING_CONTROL;

// ID: 229
typedef struct {
    uint8_t SET_ME_X04;
    uint8_t SET_ME_X00;
    uint8_t SET_ME_X80;
    uint8_t SET_ME_X10;
    uint8_t COUNTER;   
    uint8_t CHECKSUM;  
} BOSCH_SUPPLEMENTAL_1;

// ID: 232
typedef struct {
    uint16_t XMISSION_SPEED;       
    uint16_t COMPUTER_BRAKE;       
    uint8_t COMPUTER_BRAKE_REQUEST;
    uint8_t COUNTER;               
    uint8_t CHECKSUM;              
} BRAKE_HOLD;

// ID: 304
typedef struct {
    int16_t ENGINE_TORQUE_ESTIMATE;
    int16_t ENGINE_TORQUE_REQUEST; 
    uint8_t CAR_GAS;               
    uint8_t COUNTER;               
    uint8_t CHECKSUM;              
} GAS_PEDAL_2;

// ID: 316
typedef struct {
    uint8_t CAR_GAS; 
    uint8_t COUNTER; 
    uint8_t CHECKSUM;
} GAS_PEDAL;

// ID: 330
typedef struct {
    float STEER_ANGLE;            
    float STEER_ANGLE_RATE;       
    uint8_t STEER_SENSOR_STATUS_1;
    uint8_t STEER_SENSOR_STATUS_2;
    uint8_t STEER_SENSOR_STATUS_3;
    float STEER_WHEEL_ANGLE;      
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} STEERING_SENSORS;

// ID: 344
typedef struct {
    float XMISSION_SPEED; 
    uint16_t ENGINE_RPM;  
    float XMISSION_SPEED2;
    uint8_t ODOMETER;     
    uint8_t COUNTER;      
    uint8_t CHECKSUM;     
} ENGINE_DATA;

// ID: 380 (PCM)
typedef struct {
    uint8_t PEDAL_GAS;                // 7|8@0+ (1,0) [0|255] EON
    uint16_t ENGINE_RPM;              // 23|16@0+ (1,0) [0|15000] "rpm" EON
    uint8_t GAS_PRESSED;              // 39|1@0+ (1,0) [0|1] EON
    uint8_t ACC_STATUS;               // 38|1@0+ (1,0) [0|1] EON
    uint8_t BOH_17C;                  // 37|5@0+ (1,0) [0|1] EON
    uint8_t BRAKE_SWITCH;             // 32|1@0+ (1,0) [0|1] EON
    uint8_t BOH2_17C;                 // 47|10@0+ (1,0) [0|1] EON
    uint8_t BRAKE_PRESSED;            // 53|1@0+ (1,0) [0|1] EON
    uint8_t BOH3_17C;                 // 52|5@0+ (1,0) [0|1] EON
    uint8_t COUNTER;                  // 61|2@0+ (1,0) [0|3] EON
    uint8_t CHECKSUM;                 // 59|4@0+ (1,0) [0|15] EON
} POWERTRAIN_DATA;

// ID: 399
typedef struct {
    int16_t STEER_TORQUE_SENSOR;
    int16_t STEER_ANGLE_RATE;   
    uint8_t STEER_STATUS;
    uint8_t STEER_CONTROL_ACTIVE;
    uint8_t STEER_CONFIG_INDEX; 
    uint8_t COUNTER;            
    uint8_t CHECKSUM;           
} STEER_STATUS;

// ID: 401
typedef struct {
    uint8_t GEAR_SHIFTER;
    uint8_t BOH;                    
    uint8_t GEAR2;               
    uint8_t GEAR;                
    uint8_t ZEROS_BOH;              
    uint8_t COUNTER;                
    uint8_t CHECKSUM;               
} GEARBOX_15T;

// ID: 419
typedef struct {
    uint8_t GEAR_SHIFTER;
    uint8_t GEAR;                
    uint8_t COUNTER;            
    uint8_t CHECKSUM;           
} GEARBOX;

// ID: 420
typedef struct {
    float USER_BRAKE;          
    uint8_t COMPUTER_BRAKING;  
    uint8_t ESP_DISABLED;      
    uint8_t BRAKE_HOLD_RELATED;
    uint8_t BRAKE_HOLD_ACTIVE; 
    uint8_t BRAKE_HOLD_ENABLED;
    uint8_t COUNTER;           
    uint8_t CHECKSUM;          
} VSA_STATUS;

// ID: 427
typedef struct {
    uint8_t CONFIG_VALID;   
    uint16_t MOTOR_TORQUE;  
    uint8_t OUTPUT_DISABLED;
    uint8_t COUNTER;        
    uint8_t CHECKSUM;       
} STEER_MOTOR_TORQUE;

// ID: 432
typedef struct {
    uint8_t WHEELS_MOVING;
    uint8_t BRAKE_ERROR_1;
    uint8_t BRAKE_ERROR_2;
    uint8_t COUNTER;      
    uint8_t CHECKSUM;     
} STANDSTILL;

// ID: 446
typedef struct {
    uint8_t BRAKE_PRESSED;
    uint8_t COUNTER;      
    uint8_t CHECKSUM;     
} BRAKE_MODULE;

// ID: 450
typedef struct {
    uint8_t EPB_ACTIVE;   
    uint8_t EPB_STATE;
    uint8_t COUNTER;      
    uint8_t CHECKSUM;     
} EPB_STATUS;

// ID: 464
typedef struct {
    float WHEEL_SPEED_FL;
    float WHEEL_SPEED_FR;
    float WHEEL_SPEED_RL;
    float WHEEL_SPEED_RR;
    uint8_t CHECKSUM;    
} WHEEL_SPEEDS;

// ID: 479
typedef struct {
    uint8_t SET_TO_0;          
    uint8_t CONTROL_ON;        
    uint16_t GAS_COMMAND;      
    float ACCEL_COMMAND;       
    uint8_t BRAKE_LIGHTS;      
    uint8_t BRAKE_REQUEST;     
    uint8_t STANDSTILL;        
    uint8_t STANDSTILL_RELEASE;
    uint8_t AEB_STATUS;        
    uint8_t AEB_BRAKING;       
    uint8_t AEB_PREPARE;       
    uint8_t COUNTER;           
    uint8_t CHECKSUM;          
} ACC_CONTROL;

// ID: 490
typedef struct {
    float LAT_ACCEL;              
    float LONG_ACCEL;             
    uint8_t COUNTER;                
    uint8_t CHECKSUM;               
} VEHICLE_DYNAMICS;

// ID: 495
typedef struct {
    uint8_t SET_TO_75;         
    uint8_t SET_TO_30;         
    uint8_t ZEROS_BOH;         
    uint16_t ZEROS_BOH2;       
    uint8_t SET_TO_FF;         
    uint16_t SET_TO_3;         
    uint8_t CONTROL_ON;        
    uint8_t CHECKSUM;          
    uint8_t COUNTER;           
} ACC_CONTROL_ON;

// ID: 506
typedef struct {
    uint8_t CHIME;               
    uint8_t CHECKSUM;            
    uint8_t COUNTER;             
} LEGACY_BRAKE_COMMAND;

// ID: 545
typedef struct {
    uint8_t ECON_ON;       
    uint8_t DRIVE_MODE;
    uint8_t COUNTER;       
    uint8_t CHECKSUM;      
} XXX_16;

// ID: 576
typedef struct {
    uint8_t LINE_DISTANCE_VISIBLE;
    float LINE_PROBABILITY;       
    float LINE_OFFSET;            
    float LINE_ANGLE;             
    uint8_t FRAME_INDEX;          
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} LEFT_LANE_LINE_1;

// ID: 577
typedef struct {
    int8_t LINE_FAR_EDGE_POSITION;
    uint8_t LINE_SOLID;           
    uint8_t LINE_DASHED;          
    float LINE_CURVATURE;         
    uint8_t LINE_PARAMETER;       
    uint8_t FRAME_INDEX;          
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} LEFT_LANE_LINE_2;

// ID: 579
typedef struct {
    uint8_t LINE_DISTANCE_VISIBLE;
    float LINE_PROBABILITY;       
    float LINE_OFFSET;            
    float LINE_ANGLE;             
    uint8_t FRAME_INDEX;          
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} RIGHT_LANE_LINE_1;

// ID: 580
typedef struct {
    int8_t LINE_FAR_EDGE_POSITION;
    uint8_t LINE_SOLID;           
    uint8_t LINE_DASHED;          
    float LINE_CURVATURE;         
    uint8_t LINE_PARAMETER;       
    uint8_t FRAME_INDEX;          
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} RIGHT_LANE_LINE_2;

// ID: 582
typedef struct {
    uint8_t LINE_DISTANCE_VISIBLE;
    float LINE_PROBABILITY;       
    float LINE_OFFSET;            
    float LINE_ANGLE;             
    uint8_t FRAME_INDEX;          
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} ADJACENT_LEFT_LANE_LINE_1;

// ID: 583
typedef struct {
    int8_t LINE_FAR_EDGE_POSITION;
    uint8_t LINE_SOLID;           
    uint8_t LINE_DASHED;          
    float LINE_CURVATURE;         
    uint8_t LINE_PARAMETER;       
    uint8_t FRAME_INDEX;          
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} ADJACENT_LEFT_LANE_LINE_2;

// ID: 585
typedef struct {
    uint8_t LINE_DISTANCE_VISIBLE;
    float LINE_PROBABILITY;       
    float LINE_OFFSET;            
    float LINE_ANGLE;             
    uint8_t FRAME_INDEX;          
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} ADJACENT_RIGHT_LANE_LINE_1;

// ID: 586
typedef struct {
    int8_t LINE_FAR_EDGE_POSITION;
    uint8_t LINE_SOLID;           
    uint8_t LINE_DASHED;          
    float LINE_CURVATURE;         
    uint8_t LINE_PARAMETER;       
    uint8_t FRAME_INDEX;          
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} ADJACENT_RIGHT_LANE_LINE_2;

// ID: 597
typedef struct {
    uint8_t WHEEL_SPEED_FL; 
    uint8_t WHEEL_SPEED_FR; 
    uint8_t WHEEL_SPEED_RL; 
    uint8_t WHEEL_SPEED_RR; 
    uint8_t SET_TO_X55;     
    uint8_t SET_TO_X55_2;   
    uint8_t LONG_COUNTER;   
    uint8_t CHECKSUM;       
    uint8_t COUNTER;        
} ROUGH_WHEEL_SPEED;

// ID: 662
typedef struct {
    uint8_t CRUISE_BUTTONS;
    uint8_t CRUISE_SETTING;
    uint8_t COUNTER;                
    uint8_t CHECKSUM;               
} SCM_BUTTONS;

// ID: 773
typedef struct {
    uint8_t SEATBELT_DRIVER_LAMP;     
    uint8_t SEATBELT_PASS_UNLATCHED;  
    uint8_t SEATBELT_PASS_LATCHED;    
    uint8_t SEATBELT_DRIVER_UNLATCHED;
    uint8_t SEATBELT_DRIVER_LATCHED;  
    uint8_t PASS_AIRBAG_OFF;          
    uint8_t PASS_AIRBAG_ON;           
    uint8_t COUNTER;                  
    uint8_t CHECKSUM;                 
} SEATBELT_STATUS;

// ID: 777
typedef struct {
    uint8_t ROUGH_CAR_SPEED;  
    float CAR_SPEED;          
    float ROUGH_CAR_SPEED_3;  
    uint8_t ROUGH_CAR_SPEED_2;
    uint8_t LOCK_STATUS;      
    uint8_t COUNTER;          
    uint8_t CHECKSUM;         
    uint8_t IMPERIAL_UNIT;    
} CAR_SPEED;

// ID: 780
typedef struct {
    uint16_t PCM_SPEED;          
    uint8_t PCM_GAS;             
    uint8_t CRUISE_SPEED;        
    uint8_t DTC_MODE;            
    uint8_t BOH;                 
    uint8_t ACC_PROBLEM;         
    uint8_t FCM_OFF;             
    uint8_t FCM_OFF_2;           
    uint8_t FCM_PROBLEM;         
    uint8_t RADAR_OBSTRUCTED;    
    uint8_t ENABLE_MINI_CAR;     
    uint8_t HUD_DISTANCE;        
    uint8_t HUD_LEAD;            
    uint8_t BOH_3;               
    uint8_t BOH_4;               
    uint8_t BOH_5;               
    uint8_t CRUISE_CONTROL_LABEL;
    uint8_t SET_ME_X01_2;        
    uint8_t IMPERIAL_UNIT;       
    uint8_t ACC_ON;              
    uint8_t CHIME;               
    uint8_t SET_ME_X01;          
    uint8_t ICONS;               
    uint8_t COUNTER;             
    uint8_t CHECKSUM;            
} ACC_HUD;

// ID: 804
typedef struct {
    uint8_t HUD_SPEED_KPH;      
    uint8_t HUD_SPEED_MPH;      
    uint16_t TRIP_FUEL_CONSUMED;
    uint8_t CRUISE_SPEED_PCM;   
    uint8_t BOH2;               
    uint8_t BOH3;               
    uint8_t COUNTER;            
    uint8_t CHECKSUM;           
} CRUISE;

// ID: 806
typedef struct {
    uint8_t DRIVERS_DOOR_OPEN;
    uint8_t MAIN_ON;          
    uint8_t RIGHT_BLINKER;    
    uint8_t LEFT_BLINKER;     
    uint8_t CMBS_STATES;
    uint8_t COUNTER;          
    uint8_t CHECKSUM;         
} SCM_FEEDBACK;

// ID: 829
typedef struct {
    uint8_t CAM_TEMP_HIGH;          
    uint8_t SET_ME_X41;             
    uint8_t BOH;                    
    uint8_t DASHED_LANES;           
    uint8_t DTC;                    
    uint8_t LKAS_PROBLEM;           
    uint8_t LKAS_OFF;               
    uint8_t SOLID_LANES;            
    uint8_t LDW_RIGHT;              
    uint8_t STEERING_REQUIRED;      
    uint8_t BOH_2;                  
    uint8_t LDW_PROBLEM;            
    uint8_t BEEP;                  
    uint8_t LDW_ON;                 
    uint8_t LDW_OFF;                
    uint8_t CLEAN_WINDSHIELD;       
    uint8_t SET_ME_X48;             
    uint8_t COUNTER;                
    uint8_t CHECKSUM;               
} LKAS_HUD;

// ID: 862
typedef struct {
    uint8_t ZEROS_BOH;            
    uint8_t AUTO_HIGHBEAMS_ACTIVE;
    uint8_t HIGHBEAMS_ON;         
    uint8_t ZEROS_BOH_2;          
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} CAMERA_MESSAGES;

// ID: 884
typedef struct {
    uint8_t DASHBOARD_ALERT;  
    uint8_t AUTO_HEADLIGHTS;  
    uint8_t HIGH_BEAM_HOLD;   
    uint8_t HIGH_BEAM_FLASH;  
    uint8_t HEADLIGHTS_ON;    
    uint8_t WIPER_SWITCH;     
    uint8_t COUNTER;          
    uint8_t CHECKSUM;         
} STALK_STATUS;

// ID: 891
typedef struct {
    uint8_t WIPERS;       
    uint8_t LOW_BEAMS;    
    uint8_t HIGH_BEAMS;   
    uint8_t PARK_LIGHTS;  
    uint8_t COUNTER;      
    uint8_t CHECKSUM;     
} STALK_STATUS_2;

// ID: 892
typedef struct {
    float CRUISE_SPEED_OFFSET;  
    uint8_t CHECKSUM;           
    uint8_t COUNTER;            
} CRUISE_PARAMS;

// ID: 927
typedef struct {
    uint8_t ZEROS_BOH;              
    uint8_t CMBS_OFF;               
    uint8_t RESUME_INSTRUCTION;     
    uint8_t SET_TO_1;               
    uint8_t ZEROS_BOH2;             
    uint8_t APPLY_BRAKES_FOR_CANC;  
    uint8_t ACC_ALERTS;             
    uint8_t SET_TO_0;               
    uint8_t HUD_LEAD;               
    uint8_t SET_TO_64;              
    uint8_t LEAD_DISTANCE;          
    uint8_t ZEROS_BOH3;             
    uint8_t ZEROS_BOH4;             
    uint8_t COUNTER;                
    uint8_t CHECKSUM;               
} RADAR_HUD;

// ID: 1029
typedef struct {
    uint8_t DOOR_OPEN_FL;       
    uint8_t DOOR_OPEN_FR;       
    uint8_t DOOR_OPEN_RL;       
    uint8_t DOOR_OPEN_RR;       
    uint8_t TRUNK_OPEN;         
    uint8_t COUNTER;            
    uint8_t CHECKSUM;           
} DOORS_STATUS;

// ID: 1302
typedef struct {
    uint32_t ODOMETER;            
    uint8_t COUNTER;              
    uint8_t CHECKSUM;             
} ODOMETER;

//ID: 13274
typedef struct {
    uint8_t CAM_TEMP_HIGH;           
    uint8_t SET_ME_X41;              
    uint8_t BOH;                     
    uint8_t DASHED_LANES;            
    uint8_t DTC;                     
    uint8_t LKAS_PROBLEM;            
    uint8_t LKAS_OFF;                
    uint8_t SOLID_LANES;             
    uint8_t LDW_RIGHT;               
    uint8_t STEERING_REQUIRED;       
    uint8_t BOH_2;                   
    uint8_t LDW_PROBLEM;             
    uint8_t BEEP;                    
    uint8_t SET_ME_X01;              
    uint8_t LDW_ON;                  
    uint8_t LDW_OFF;                 
    uint8_t CLEAN_WINDSHIELD;        
    uint8_t COUNTER;                 
    uint8_t CHECKSUM;                
} LKAS_HUD_A;

// ID: 13275
typedef struct {
    uint8_t CAM_TEMP_HIGH;          
    uint8_t SET_ME_X41;             
    uint8_t BOH;                    
    uint8_t DASHED_LANES;           
    uint8_t DTC;                    
    uint8_t LKAS_PROBLEM;           
    uint8_t LKAS_OFF;               
    uint8_t SOLID_LANES;            
    uint8_t LDW_RIGHT;              
    uint8_t STEERING_REQUIRED;      
    uint8_t BOH_2;                  
    uint8_t LDW_PROBLEM;            
    uint8_t BEEP;                   
    uint8_t SET_ME_X01;             
    uint8_t LDW_ON;                 
    uint8_t LDW_OFF;                
    uint8_t CLEAN_WINDSHIELD;       
    uint8_t COUNTER;                
    uint8_t CHECKSUM;               
} LKAS_HUD_B;
