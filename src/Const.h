#pragma once
#include <cstdint>

// CAN FRAME IDs from a Honda Accord 2018 .dbc
// Most of these should work across generations- especially basic LX trim data. Confirmed working with 9th gen.
// #define KINEMATICS_ID 148
// #define BRAKE_HOLD_ID 232
// #define STEERING_CONTROL_ID 228
// #define BOSCH_SUPPLEMENTAL_1_ID 229
// #define GAS_PEDAL_2_ID 304 // Seems to be platform-agnostic
// #define GAS_PEDAL_ID 316 // Should exist on Nidec
// #define ENGINE_DATA_ID 344
// #define POWERTRAIN_DATA_ID 380
// #define GEARBOX_ID 401
// #define VSA_STATUS_ID 420
// #define STEER_MOTOR_TORQUE_ID 427
// #define STEER_STATUS_ID 399
// #define WHEEL_SPEEDS_ID 464
// #define EPB_STATUS_ID 450
// #define VEHICLE_DYNAMICS_ID 490
// #define ACC_CONTROL_ID 479
// #define ROUGH_WHEEL_SPEED_ID 597
// #define LEFT_LANE_LINE_1_ID 576
// #define LEFT_LANE_LINE_2_ID 577
// #define RIGHT_LANE_LINE_1_ID 579
// #define RIGHT_LANE_LINE_2_ID 580
// #define ADJACENT_LEFT_LANE_LINE_1_ID 582
// #define ADJACENT_LEFT_LANE_LINE_2_ID 583
// #define ADJACENT_RIGHT_LANE_LINE_1_ID 585
// #define ADJACENT_RIGHT_LANE_LINE_2_ID 586
// #define XXX_16_ID 545
// #define SCM_BUTTONS_ID 662
// #define SCM_FEEDBACK_ID 806
// #define CAMERA_MESSAGES_ID 862
// #define RADAR_HUD_ID 927
// #define SEATBELT_STATUS_ID 773
// #define CAR_SPEED_ID 777
// #define ACC_HUD_ID 780
// #define CRUISE_ID 804
// #define STALK_STATUS_ID 884
// #define STALK_STATUS_2_ID 891
// #define DOORS_STATUS_ID 1029
// #define LKAS_HUD_A_ID 13274
// #define LKAS_HUD_B_ID 13275

#define STEERING_SENSORS_ID 342
#define ENGINE_DATA_2_ID 344
#define WHEEL_SPEEDS_ID 464
#define VSA_STATUS_ID 420
#define POWERTRAIN_DATA_ID 380
#define VEHICLE_DYNAMICS_ID 490
#define GEARBOX_ID 422
#define STANDSTILL_ID 432
#define DRIVE_MODES_ID 545
#define ENGINE_DATA_3_ID 804
#define CAR_SPEED_ID 777


typedef enum {
    GEAR_L = 32,
    GEAR_S = 16,
    GEAR_D = 8,
    GEAR_N = 4,
    GEAR_R = 2,
    GEAR_P = 1
} GEAR_SHIFTER_t;

typedef enum {
    ECON_OFF = 0, 
    ECON_ON = 3
} ECON_t;

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

typedef struct {
    float LAT_ACCEL;                 // 7|10@0+  (-0.035,17.92) [-20|20] "m/s2" EON
    float LONG_ACCEL;                // 25|10@0+ (-0.035,17.92) [-20|20] "m/s2" EON
} KINEMATICS;

typedef struct {
    int16_t ENGINE_TORQUE_ESTIMATE; // 7|16@0- (1,0) [-1000|1000] "Nm" EON
    int16_t ENGINE_TORQUE_REQUEST;  // 23|16@0- (1,0) [-1000|1000] "Nm" EON
    uint8_t CAR_GAS;                // 39|8@0+ (1,0) [0|255] "" EON
} ENGINE_DATA_1;

typedef struct {
    float XMISSION_SPEED;           // 7|16@0+ (0.01,0) [0|250] "kph" EON
    float XMISSION_SPEED_SPEEDOMETER;          // 39|16@0+ (0.01,0) [0|250] "kph" EON
    uint8_t CURRENT_TRIP_DISTANCE;          // 39|16@0+ (0.01,0) [0|250] "kph" EON
} ENGINE_DATA_2;

typedef struct {
    uint8_t ENGINE_TEMP; 
    uint8_t INTAKE_TEMP; 
    float TRIP_FUEL_CONSUMED;
} ENGINE_DATA_3;

typedef struct {
    uint8_t PEDAL_GAS;              // 7|8@0+ (1,0) [0|255] EON
    uint16_t ENGINE_RPM;            // 23|16@0+ (1,0) [0|15000] "rpm" EON
    uint8_t GAS_PRESSED;            // 39|1@0+ (1,0) [0|1] EON
    uint8_t BRAKE_SWITCH;           // 32|1@0+ (1,0) [0|1] EON
    uint8_t BRAKE_PRESSED;          // 53|1@0+ (1,0) [0|1] EON
} POWERTRAIN_DATA;

typedef struct {
    uint8_t GEAR_SHIFTER;           // 5|6@0+ (1,0) [0|63] "" EON
} GEARBOX;  

typedef struct {
    uint16_t USER_BRAKE;          
    //uint8_t COMPUTER_BRAKING;  
    uint8_t ESP_DISABLED;  // VSA button
} VSA_STATUS;

typedef struct {
    uint8_t WHEELS_MOVING;
 } STANDSTILL;

// ID: 450
typedef struct {
    uint8_t EPB_ACTIVE;   
    uint8_t EPB_STATE;
} EPB_STATUS;

// ID: 464
typedef struct {
    float WHEEL_SPEED_FL;
    float WHEEL_SPEED_FR;
    float WHEEL_SPEED_RL;
    float WHEEL_SPEED_RR;
} WHEEL_SPEEDS;

typedef struct {
    float LAT_ACCEL;              
    float LONG_ACCEL;                          
} VEHICLE_DYNAMICS;

typedef struct {
    uint8_t ECON_ON;       
} DRIVE_MODES;

typedef struct {
    uint8_t WHEEL_SPEED_FL; 
    uint8_t WHEEL_SPEED_FR; 
    uint8_t WHEEL_SPEED_RL; 
    uint8_t WHEEL_SPEED_RR; 
} ROUGH_WHEEL_SPEED;

typedef struct {
    uint8_t CRUISE_BUTTONS;
    uint8_t CRUISE_SETTING;              
} SCM_BUTTONS;

typedef struct {
    uint8_t SEATBELT_DRIVER_LAMP;     
    uint8_t SEATBELT_PASS_UNLATCHED;  
    uint8_t SEATBELT_PASS_LATCHED;    
    uint8_t SEATBELT_DRIVER_UNLATCHED;
    uint8_t SEATBELT_DRIVER_LATCHED;  
    uint8_t PASS_AIRBAG_OFF;          
    uint8_t PASS_AIRBAG_ON;               
} SEATBELT_STATUS;

typedef struct {
    float ROUGH_CAR_SPEED_3;  
    //uint8_t LOCK_STATUS;        
    //uint8_t IMPERIAL_UNIT;    
} CAR_SPEED;

typedef struct {
    uint8_t DASHBOARD_ALERT;  
    uint8_t AUTO_HEADLIGHTS;  
    uint8_t HIGH_BEAM_HOLD;   
    uint8_t HIGH_BEAM_FLASH;  
    uint8_t HEADLIGHTS_ON;    
    uint8_t WIPER_SWITCH;      
} STALK_STATUS;

typedef struct {
    uint8_t WIPERS;       
    uint8_t LOW_BEAMS;    
    uint8_t HIGH_BEAMS;   
    uint8_t PARK_LIGHTS;   
} STALK_STATUS_2;

typedef struct {
    uint8_t DOOR_OPEN_FL;       
    uint8_t DOOR_OPEN_FR;       
    uint8_t DOOR_OPEN_RL;       
    uint8_t DOOR_OPEN_RR;       
    uint8_t TRUNK_OPEN;          
} DOORS_STATUS;
