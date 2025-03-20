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
    GEAR_SHIFTER_L_15T = 32,
    GEAR_SHIFTER_S_15T = 16,
    GEAR_SHIFTER_D_15T = 8,
    GEAR_SHIFTER_N_15T = 4,
    GEAR_SHIFTER_R_15T = 2,
    GEAR_SHIFTER_P_15T = 1
} GEAR_SHIFTER_15T_t;

// Enum for GEARBOX_15T.GEAR (1.5T)
typedef enum {
    GEAR_L_15T = 7,
    GEAR_S_15T = 10,
    GEAR_D_15T = 4,
    GEAR_N_15T = 3,
    GEAR_R_15T = 2,
    GEAR_P_15T = 1
} GEAR_15T_t;

// Enum for GEARBOX.GEAR_SHIFTER
typedef enum {
    GEAR_SHIFTER_S = 2,
    GEAR_SHIFTER_D = 32,
    GEAR_SHIFTER_N = 16,
    GEAR_SHIFTER_R = 8,
    GEAR_SHIFTER_P = 4
} GEAR_SHIFTER_t;

// Enum for GEARBOX.GEAR
typedef enum {
    GEAR_S = 26,
    GEAR_D = 20,
    GEAR_N = 19,
    GEAR_R = 18,
    GEAR_P = 17
} GEAR_t;

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
typedef struct {
    float LAT_ACCEL;                 // (7|10) [-20|20] "m/s2"
    float LONG_ACCEL;                // (25|10) [-20|20] "m/s2"
    uint8_t CHECKSUM;                // (59|4) [0|3] ""
    uint8_t COUNTER;                 // (61|2) [0|3] ""
} KINEMATICS;

// ID: 228
typedef struct {
    uint8_t STEER_TORQUE_REQUEST;    // (23|1) [0|1] ""
    uint8_t SET_ME_X00;              // (22|7) [0|127] ""
    uint8_t SET_ME_X00_2;            // (31|8) [0|0] ""
    int16_t STEER_TORQUE;            // (7|16) [-4096|4096] ""
    uint8_t STEER_DOWN_TO_ZERO;      // (38|1) [0|1] ""
    uint8_t COUNTER;                 // (37|2) [0|3] ""
    uint8_t CHECKSUM;                // (35|4) [0|15] ""
} STEERING_CONTROL;

// ID: 229
typedef struct {
    uint8_t SET_ME_X04;             // (0|8) [0|255] ""
    uint8_t SET_ME_X00;             // (8|8) [0|255] ""
    uint8_t SET_ME_X80;             // (16|8) [0|255] ""
    uint8_t SET_ME_X10;             // (24|8) [0|255] ""
    uint8_t COUNTER;                // (61|2) [0|3] ""
    uint8_t CHECKSUM;               // (59|4) [0|15] ""
} BOSCH_SUPPLEMENTAL_1;

// ID: 232
typedef struct {
    uint16_t XMISSION_SPEED;        // (7|14) [1|0] ""
    uint16_t COMPUTER_BRAKE;        // (39|16) [0|0] ""
    uint8_t COMPUTER_BRAKE_REQUEST; // (29|1) [0|0] ""
    uint8_t COUNTER;                // (53|2) [0|3] ""
    uint8_t CHECKSUM;               // (51|4) [0|15] ""
} BRAKE_HOLD;

// ID: 304
typedef struct {
    int16_t ENGINE_TORQUE_ESTIMATE;   // (7|16) [-1000|1000] "Nm"
    int16_t ENGINE_TORQUE_REQUEST;    // (23|16) [-1000|1000] "Nm"
    uint8_t CAR_GAS;                  // (39|8) [0|255] ""
    uint8_t COUNTER;                  // (61|2) [0|3] ""
    uint8_t CHECKSUM;                 // (59|4) [0|15] ""
} GAS_PEDAL_2;

// ID: 316
typedef struct {
    uint8_t CAR_GAS;                  // (39|8) [0|255] ""
    uint8_t COUNTER;                  // (61|2) [0|3] ""
    uint8_t CHECKSUM;                 // (59|4) [0|15] ""
} GAS_PEDAL;

// ID: 330
typedef struct {
    float STEER_ANGLE;             // (7|16) [-500|500] "deg"
    float STEER_ANGLE_RATE;        // (23|16) [-3000|3000] "deg/s"
    uint8_t STEER_SENSOR_STATUS_1; // (34|1) [0|1] ""
    uint8_t STEER_SENSOR_STATUS_2; // (33|1) [0|1] ""
    uint8_t STEER_SENSOR_STATUS_3; // (32|1) [0|1] ""
    float STEER_WHEEL_ANGLE;       // (47|16) [-500|500] "deg"
    uint8_t COUNTER;               // (61|2) [0|3] ""
    uint8_t CHECKSUM;              // (59|4) [0|15] ""
} STEERING_SENSORS;

// ID: 344
typedef struct {
    float XMISSION_SPEED;             // (7|16) [0|250] "kph"
    uint16_t ENGINE_RPM;              // (23|16) [0|15000] "rpm"
    float XMISSION_SPEED2;            // (39|16) [0|250] "kph"
    uint8_t ODOMETER;                 // (55|8) [0|2550] "m"
    uint8_t COUNTER;                  // (61|2) [0|3] ""
    uint8_t CHECKSUM;                 // (59|4) [0|15] ""
} ENGINE_DATA;

// ID: 380
typedef struct {
    uint8_t PEDAL_GAS;                // (7|8) [0|255] ""
    uint16_t ENGINE_RPM;              // (23|16) [0|15000] "rpm"
    uint8_t GAS_PRESSED;              // (39|1) [0|1] ""
    uint8_t ACC_STATUS;               // (38|1) [0|1] ""
    uint8_t BOH_17C;                  // (37|5) [0|1] ""
    uint8_t BRAKE_SWITCH;             // (32|1) [0|1] ""
    uint8_t BOH2_17C;                 // (47|10) [0|1] ""
    uint8_t BRAKE_PRESSED;            // (53|1) [0|1] ""
    uint8_t BOH3_17C;                 // (52|5) [0|1] ""
    uint8_t COUNTER;                  // (61|2) [0|3] ""
    uint8_t CHECKSUM;                 // (59|4) [0|15] ""
} POWERTRAIN_DATA;

// ID: 399
typedef struct {
    int16_t STEER_TORQUE_SENSOR;    // (7|16) [-31000|31000] "tbd"
    int16_t STEER_ANGLE_RATE;       // (23|16) [-31000|31000] "deg/s"
    STEER_STATUS_t STEER_STATUS;    // (39|4) [0|15] ""
    uint8_t STEER_CONTROL_ACTIVE;   // (35|1) [0|1] ""
    uint8_t STEER_CONFIG_INDEX;     // (43|4) [0|15] ""
    uint8_t COUNTER;                // (53|2) [0|3] ""
    uint8_t CHECKSUM;               // (51|4) [0|15] ""
} STEER_STATUS;

// ID: 401
typedef struct {
    GEAR_SHIFTER_15T_t GEAR_SHIFTER; // (5|6) [0|63] ""
    uint8_t BOH;                     // (45|6) [0|63] ""
    GEAR_15T_t GEAR2;                // (31|8) [0|1] ""
    GEAR_15T_t GEAR;                 // (39|8) [0|255] ""
    uint8_t ZEROS_BOH;               // (47|2) [0|3] ""
    uint8_t COUNTER;                 // (61|2) [0|3] ""
    uint8_t CHECKSUM;                // (59|4) [0|15] ""
} GEARBOX_15T;

// ID: 419
typedef struct {
    GEAR_SHIFTER_t GEAR_SHIFTER; // (24|8) [0|255] ""
    GEAR_t GEAR;                 // (32|8) [0|255] ""
    uint8_t COUNTER;             // (61|2) [0|3] ""
    uint8_t CHECKSUM;            // (59|4) [0|15] ""
} GEARBOX;

// ID: 420
typedef struct {
    float USER_BRAKE;                // (7|16) [0|1000] ""
    uint8_t COMPUTER_BRAKING;        // (23|1) [0|1] ""
    uint8_t ESP_DISABLED;            // (28|1) [0|1] ""
    uint8_t BRAKE_HOLD_RELATED;      // (52|1) [0|1] "On when Brake Hold engaged"
    uint8_t BRAKE_HOLD_ACTIVE;       // (46|1) [0|1] ""
    uint8_t BRAKE_HOLD_ENABLED;      // (45|1) [0|1] ""
    uint8_t COUNTER;                 // (61|2) [0|3] ""
    uint8_t CHECKSUM;                // (59|4) [0|15] ""
} VSA_STATUS;

// ID: 427
typedef struct {
    uint8_t CONFIG_VALID;            // (7|1) [0|1] ""
    uint16_t MOTOR_TORQUE;           // (1|10) [0|256] ""
    uint8_t OUTPUT_DISABLED;         // (22|1) [0|1] ""
    uint8_t COUNTER;                 // (21|2) [0|3] ""
    uint8_t CHECKSUM;                // (19|4) [0|15] ""
} STEER_MOTOR_TORQUE;

// ID: 432
typedef struct {
    uint8_t WHEELS_MOVING;        // (12|1) [0|1] ""
    uint8_t BRAKE_ERROR_1;        // (11|1) [0|1] ""
    uint8_t BRAKE_ERROR_2;        // (9|1) [0|1] ""
    uint8_t COUNTER;              // (53|2) [0|3] ""
    uint8_t CHECKSUM;             // (51|4) [0|15] ""
} STANDSTILL;

// ID: 446
typedef struct {
    uint8_t BRAKE_PRESSED;        // (4|1) [0|1] ""
    uint8_t COUNTER;              // (21|2) [0|3] ""
    uint8_t CHECKSUM;             // (19|4) [0|15] ""
} BRAKE_MODULE;

// ID: 450
typedef struct {
    uint8_t EPB_ACTIVE;             // (3|1) [0|1] ""
    EPB_STATE_t EPB_STATE;          // (29|2) [0|3] ""
    uint8_t COUNTER;                // (61|2) [0|3] ""
    uint8_t CHECKSUM;               // (59|4) [0|15] ""
} EPB_STATUS;

// ID: 464
typedef struct {
    float WHEEL_SPEED_FL;            // (7|15) [0|250] "kph"
    float WHEEL_SPEED_FR;            // (8|15) [0|250] "kph"
    float WHEEL_SPEED_RL;            // (25|15) [0|250] "kph"
    float WHEEL_SPEED_RR;            // (42|15) [0|250] "kph"
    uint8_t CHECKSUM;                // (59|4) [0|3] ""
} WHEEL_SPEEDS;

// ID: 479
typedef struct {
    uint8_t SET_TO_0;               // (20|5) [0|1] ""
    uint8_t CONTROL_ON;             // (23|3) [0|5] "Set to 5 when car is being controlled"
    uint16_t GAS_COMMAND;           // (7|16) [0|0] ""
    float ACCEL_COMMAND;            // (31|11) [0.01,0] "m/s2"
    uint8_t BRAKE_LIGHTS;           // (62|1) [0|1] ""
    uint8_t BRAKE_REQUEST;          // (34|1) [0|1] ""
    uint8_t STANDSTILL;             // (35|1) [0|1] ""
    uint8_t STANDSTILL_RELEASE;     // (36|1) [0|1] ""
    uint8_t AEB_STATUS;             // (33|1) [0|1] "Set for the duration of AEB event"
    uint8_t AEB_BRAKING;            // (47|1) [0|1] "Set when braking is commanded during AEB event"
    uint8_t AEB_PREPARE;            // (43|1) [0|1] "Set 1s before AEB"
    uint8_t COUNTER;                // (61|2) [0|3] ""
    uint8_t CHECKSUM;               // (59|4) [0|15] ""
} ACC_CONTROL;

// ID: 490
typedef struct {
    float LAT_ACCEL;                 // (7|16) [-20|20] "m/s2"
    float LONG_ACCEL;                // (23|16) [-20|20] "Wheel speed derivative, noisy and zero snapping. m/s2"
    uint8_t COUNTER;                 // (61|2) [0|3] ""
    uint8_t CHECKSUM;                // (59|4) [0|3] ""
} VEHICLE_DYNAMICS;

// ID: 495
typedef struct {
    uint8_t SET_TO_75;              // (31|8) [0|255] ""
    uint8_t SET_TO_30;              // (39|8) [0|255] ""
    uint8_t ZEROS_BOH;              // (23|8) [0|255] ""
    uint16_t ZEROS_BOH2;            // (47|16) [0|255] ""
    uint8_t SET_TO_FF;              // (15|8) [0|255] ""
    uint16_t SET_TO_3;              // (6|7) [0|4095] ""
    uint8_t CONTROL_ON;             // (7|1) [0|1] ""
    uint8_t CHECKSUM;               // (59|4) [0|15] ""
    uint8_t COUNTER;                // (61|2) [0|3] ""
} ACC_CONTROL_ON;

// ID: 506
typedef struct {
    uint8_t CHIME;                // (40|8) [0|255] ""
    uint8_t CHECKSUM;             // (59|4) [0|15] ""
    uint8_t COUNTER;              // (61|2) [0|3] ""
} LEGACY_BRAKE_COMMAND;

// ID: 545
typedef struct {
    uint8_t ECON_ON;                // (23|1) [0|1] ""
    ECON_ON_2_t DRIVE_MODE;         // (37|2) [0|3] "" 
    uint8_t COUNTER;                // (45|2) [0|3] ""
    uint8_t CHECKSUM;               // (43|4) [0|15] ""
} XXX_16;

// ID: 576
typedef struct {
    uint8_t LINE_DISTANCE_VISIBLE; // (39|9) [0|1] ""
    float LINE_PROBABILITY;        // (46|6) [0.015625,0] ""
    float LINE_OFFSET;             // (23|12) [0.004,-8.192] "Meters"
    float LINE_ANGLE;              // (7|12) [0.0005,-1.024] ""
    uint8_t FRAME_INDEX;           // (8|4) [0|15] ""
    uint8_t COUNTER;               // (61|2) [0|1] ""
    uint8_t CHECKSUM;              // (59|4) [0|1] ""
} LEFT_LANE_LINE_1;

// ID: 577
typedef struct {
    int8_t LINE_FAR_EDGE_POSITION;  // (55|8) [0|1] ""
    uint8_t LINE_SOLID;             // (13|1) [0|1] ""
    uint8_t LINE_DASHED;            // (14|1) [0|1] ""
    float LINE_CURVATURE;           // (23|12) [0.00001,-0.02048] ""
    uint8_t LINE_PARAMETER;         // (39|12) [0|1] ""
    uint8_t FRAME_INDEX;            // (7|4) [0|15] ""
    uint8_t COUNTER;                // (61|2) [0|1] ""
    uint8_t CHECKSUM;               // (59|4) [0|1] ""
} LEFT_LANE_LINE_2;

// ID: 579
typedef struct {
    uint8_t LINE_DISTANCE_VISIBLE;  // (39|9) [0|1] ""
    float LINE_PROBABILITY;         // (46|6) [0.015625,0] ""
    float LINE_OFFSET;              // (23|12) [0.004,-8.192] "Meters"
    float LINE_ANGLE;               // (7|12) [0.0005,-1.024] ""
    uint8_t FRAME_INDEX;            // (8|4) [0|15] ""
    uint8_t COUNTER;                // (61|2) [0|1] ""
    uint8_t CHECKSUM;               // (59|4) [0|1] ""
} RIGHT_LANE_LINE_1;

// ID: 580
typedef struct {
    int8_t LINE_FAR_EDGE_POSITION; // (55|8) [0|1] ""
    uint8_t LINE_SOLID;            // (13|1) [0|1] ""
    uint8_t LINE_DASHED;           // (14|1) [0|1] ""
    float LINE_CURVATURE;          // (23|12) [0.00001,-0.02048] ""
    uint8_t LINE_PARAMETER;        // (39|12) [0|1] ""
    uint8_t FRAME_INDEX;           // (7|4) [0|15] ""
    uint8_t COUNTER;               // (61|2) [0|1] ""
    uint8_t CHECKSUM;              // (59|4) [0|1] ""
} RIGHT_LANE_LINE_2;

// ID: 582
typedef struct {
    uint8_t LINE_DISTANCE_VISIBLE; // (39|9) [0|1] ""
    float LINE_PROBABILITY;        // (46|6) [0.015625,0] ""
    float LINE_OFFSET;             // (23|12) [0.004,-8.192] "Meters"
    float LINE_ANGLE;              // (7|12) [0.0005,-1.024] ""
    uint8_t FRAME_INDEX;           // (8|4) [0|15] ""
    uint8_t COUNTER;               // (61|2) [0|1] ""
    uint8_t CHECKSUM;              // (59|4) [0|1] ""
} ADJACENT_LEFT_LANE_LINE_1;

// ID: 583
typedef struct {
    int8_t LINE_FAR_EDGE_POSITION; // (55|8) [0|1] ""
    uint8_t LINE_SOLID;            // (13|1) [0|1] ""
    uint8_t LINE_DASHED;           // (14|1) [0|1] ""
    float LINE_CURVATURE;          // (23|12) [0.00001,-0.02048] ""
    uint8_t LINE_PARAMETER;        // (39|12) [0|1] ""
    uint8_t FRAME_INDEX;           // (7|4) [0|15] ""
    uint8_t COUNTER;               // (61|2) [0|1] ""
    uint8_t CHECKSUM;              // (59|4) [0|1] ""
} ADJACENT_LEFT_LANE_LINE_2;

// ID: 585
typedef struct {
    uint8_t LINE_DISTANCE_VISIBLE; // (39|9) [0|1] ""
    float LINE_PROBABILITY;        // (46|6) [0.015625,0] ""
    float LINE_OFFSET;             // (23|12) [0.004,-8.192] "Meters"
    float LINE_ANGLE;              // (7|12) [0.0005,-1.024] ""
    uint8_t FRAME_INDEX;           // (8|4) [0|15] ""
    uint8_t COUNTER;               // (61|2) [0|1] ""
    uint8_t CHECKSUM;              // (59|4) [0|1] ""
} ADJACENT_RIGHT_LANE_LINE_1;

// ID: 586
typedef struct {
    int8_t LINE_FAR_EDGE_POSITION; // (55|8) [0|1] ""
    uint8_t LINE_SOLID;            // (13|1) [0|1] ""
    uint8_t LINE_DASHED;           // (14|1) [0|1] ""
    float LINE_CURVATURE;          // (23|12) [0.00001,-0.02048] ""
    uint8_t LINE_PARAMETER;        // (39|12) [0|1] ""
    uint8_t FRAME_INDEX;           // (7|4) [0|15] ""
    uint8_t COUNTER;               // (61|2) [0|1] ""
    uint8_t CHECKSUM;              // (59|4) [0|1] ""
} ADJACENT_RIGHT_LANE_LINE_2;

// ID: 597
typedef struct {
    uint8_t WHEEL_SPEED_FL;         // (7|8) [0|255] "mph"
    uint8_t WHEEL_SPEED_FR;         // (15|8) [0|255] "mph"
    uint8_t WHEEL_SPEED_RL;         // (23|8) [0|255] "mph"
    uint8_t WHEEL_SPEED_RR;         // (31|8) [0|255] "mph"
    uint8_t SET_TO_X55;             // (39|8) [0|255] ""
    uint8_t SET_TO_X55_2;           // (47|8) [0|255] ""
    uint8_t LONG_COUNTER;           // (55|8) [0|255] ""
    uint8_t CHECKSUM;               // (59|4) [0|15] ""
    uint8_t COUNTER;                // (61|2) [0|3] ""
} ROUGH_WHEEL_SPEED;

// ID: 662
typedef struct {
    CRUISE_BUTTONS_t CRUISE_BUTTONS; // (7|3) [0|7] ""
    CRUISE_SETTING_t CRUISE_SETTING; // (3|2) [0|3] ""
    uint8_t COUNTER;                 // (29|2) [0|3] ""
    uint8_t CHECKSUM;                // (27|4) [0|15] ""
} SCM_BUTTONS;

// ID: 773
typedef struct {
    uint8_t SEATBELT_DRIVER_LAMP;      // (7|1) [0|1] ""
    uint8_t SEATBELT_PASS_UNLATCHED;   // (10|1) [0|1] ""
    uint8_t SEATBELT_PASS_LATCHED;     // (11|1) [0|1] ""
    uint8_t SEATBELT_DRIVER_UNLATCHED; // (12|1) [0|1] ""
    uint8_t SEATBELT_DRIVER_LATCHED;   // (13|1) [0|1] ""
    uint8_t PASS_AIRBAG_OFF;           // (14|1) [0|1] "Might just be indicator light"
    uint8_t PASS_AIRBAG_ON;            // (15|1) [0|1] "Might just be indicator light"
    uint8_t COUNTER;                   // (53|2) [0|3] ""
    uint8_t CHECKSUM;                  // (51|4) [0|3] ""
} SEATBELT_STATUS;

// ID: 777
typedef struct {
    uint8_t ROUGH_CAR_SPEED;        // (23|8) [0|255] "mph"
    float CAR_SPEED;                // (7|16) [0|65535] "kph"
    float ROUGH_CAR_SPEED_3;        // (39|16) [0|65535] "kph"
    uint8_t ROUGH_CAR_SPEED_2;      // (31|8) [0|255] "mph"
    uint8_t LOCK_STATUS;            // (55|2) [0|255] ""
    uint8_t COUNTER;                // (61|2) [0|3] ""
    uint8_t CHECKSUM;               // (59|4) [0|15] ""
    uint8_t IMPERIAL_UNIT;          // (63|1) [0|1] ""
} CAR_SPEED;

// ID: 780
typedef struct {
    uint16_t PCM_SPEED;           // (7|16) [0|250] "Used by Nidec. kph"
    uint8_t PCM_GAS;              // (23|8) [0|127] "Used by Nidec"
    uint8_t CRUISE_SPEED;         // (31|8) [0|255] "255 = no speed. kph"
    uint8_t DTC_MODE;             // (39|1) [0|1] ""
    uint8_t BOH;                  // (38|1) [0|1] ""
    uint8_t ACC_PROBLEM;          // (37|1) [0|1] ""
    uint8_t FCM_OFF;              // (35|1) [0|1] ""
    uint8_t FCM_OFF_2;            // (36|1) [0|1] ""
    uint8_t FCM_PROBLEM;          // (34|1) [0|1] ""
    uint8_t RADAR_OBSTRUCTED;     // (33|1) [0|1] ""
    uint8_t ENABLE_MINI_CAR;      // (32|1) [0|1] ""
    uint8_t HUD_DISTANCE;         // (47|2) [0|3] ""
    uint8_t HUD_LEAD;             // (45|2) [0|3] ""
    uint8_t BOH_3;                // (43|1) [0|3] ""
    uint8_t BOH_4;                // (42|1) [0|3] ""
    uint8_t BOH_5;                // (41|1) [0|3] ""
    uint8_t CRUISE_CONTROL_LABEL; // (40|1) [0|3] ""
    uint8_t SET_ME_X01_2;         // (55|1) [0|1] ""
    uint8_t IMPERIAL_UNIT;        // (54|1) [0|1] ""
    uint8_t ACC_ON;               // (52|1) [0|1] ""
    uint8_t CHIME;                // (51|3) [0|1] ""
    uint8_t SET_ME_X01;           // (48|1) [0|1] ""
    uint8_t ICONS;                // (63|2) [0|1] ""
    uint8_t COUNTER;              // (61|2) [0|3] ""
    uint8_t CHECKSUM;             // (59|4) [0|3] ""
} ACC_HUD;

// ID: 804
typedef struct {
    uint8_t HUD_SPEED_KPH;       // (7|8) [0|255] "kph"
    uint8_t HUD_SPEED_MPH;       // (15|8) [0|255] "mph"
    uint16_t TRIP_FUEL_CONSUMED; // (23|16) [0|255] ""
    uint8_t CRUISE_SPEED_PCM;    // (39|8) [0|255] "255 = no speed"
    uint8_t BOH2;                // (47|8) [0|255] ""
    uint8_t BOH3;                // (55|8) [0|255] ""
    uint8_t COUNTER;             // (61|2) [0|3] ""
    uint8_t CHECKSUM;            // (59|4) [0|15] ""
} CRUISE;

// ID: 806
typedef struct {
    uint8_t DRIVERS_DOOR_OPEN;       // (17|1) [0|1] ""
    uint8_t MAIN_ON;                 // (28|1) [0|1] ""
    uint8_t RIGHT_BLINKER;           // (27|1) [0|1] ""
    uint8_t LEFT_BLINKER;            // (26|1) [0|1] ""
    CMBS_BUTTON_t CMBS_STATES;       // (22|2) [0|3] ""
    uint8_t COUNTER;                 // (61|2) [0|3] ""
    uint8_t CHECKSUM;                // (59|4) [0|15] ""
} SCM_FEEDBACK;

// ID: 829
typedef struct {
    uint8_t CAM_TEMP_HIGH;          // (7|1) [0|255] ""
    uint8_t SET_ME_X41;             // (6|7) [0|127] ""
    uint8_t BOH;                    // (6|7) [0|127] ""
    uint8_t DASHED_LANES;           // (14|1) [0|1] ""
    uint8_t DTC;                    // (13|1) [0|1] ""
    uint8_t LKAS_PROBLEM;           // (12|1) [0|1] ""
    uint8_t LKAS_OFF;               // (11|1) [0|1] ""
    uint8_t SOLID_LANES;            // (10|1) [0|1] ""
    uint8_t LDW_RIGHT;              // (9|1) [0|1] ""
    uint8_t STEERING_REQUIRED;      // (8|1) [0|1] ""
    uint8_t BOH_2;                  // (23|2) [0|4] ""
    uint8_t LDW_PROBLEM;            // (21|1) [0|1] ""
    BEEP_t BEEP;                    // (17|2) [0|1] "Beeps are pleasant, chimes are for warnings etc...";
    uint8_t LDW_ON;                 // (28|1) [0|1] ""
    uint8_t LDW_OFF;                // (27|1) [0|1] ""
    uint8_t CLEAN_WINDSHIELD;       // (26|1) [0|1] ""
    uint8_t SET_ME_X48;             // (31|8) [0|255] ""
    uint8_t COUNTER;                // (37|2) [0|3] ""
    uint8_t CHECKSUM;               // (35|4) [0|15] ""
} LKAS_HUD;

// ID: 862
typedef struct {
    uint8_t ZEROS_BOH;               // (7|50) [0|127] ""
    uint8_t AUTO_HIGHBEAMS_ACTIVE;   // (53|1) [0|1] ""
    uint8_t HIGHBEAMS_ON;            // (52|1) [0|1] ""
    uint8_t ZEROS_BOH_2;             // (51|4) [0|15] ""
    uint8_t COUNTER;                 // (61|2) [0|3] ""
    uint8_t CHECKSUM;                // (59|4) [0|15] ""
} CAMERA_MESSAGES;

// ID: 884
typedef struct {
    uint8_t DASHBOARD_ALERT;    // (39|8) [0|255] ""
    uint8_t AUTO_HEADLIGHTS;    // (46|1) [0|1] ""
    uint8_t HIGH_BEAM_HOLD;     // (47|1) [0|1] ""
    uint8_t HIGH_BEAM_FLASH;    // (45|1) [0|1] ""
    uint8_t HEADLIGHTS_ON;      // (54|1) [0|1] ""
    uint8_t WIPER_SWITCH;       // (53|2) [0|3] ""
    uint8_t COUNTER;            // (61|2) [0|3] ""
    uint8_t CHECKSUM;           // (59|4) [0|15] ""
} STALK_STATUS;

// ID: 891
typedef struct {
    uint8_t WIPERS;             // (17|2) [0|3] ""
    uint8_t LOW_BEAMS;          // (35|1) [0|1] ""
    uint8_t HIGH_BEAMS;         // (34|1) [0|1] ""
    uint8_t PARK_LIGHTS;        // (36|1) [0|1] ""
    uint8_t COUNTER;            // (61|2) [0|3] ""
    uint8_t CHECKSUM;           // (59|4) [0|15] ""
} STALK_STATUS_2;

// ID: 892
typedef struct {
    float CRUISE_SPEED_OFFSET;    // (31|8) [-128|127] "kph"
    uint8_t CHECKSUM;             // (59|4) [0|3] ""
    uint8_t COUNTER;              // (61|2) [0|15] ""
} CRUISE_PARAMS;

// ID: 927
typedef struct {
    uint8_t ZEROS_BOH;               // (7|10) [0|127] ""
    uint8_t CMBS_OFF;                // (12|1) [0|1] ""
    uint8_t RESUME_INSTRUCTION;      // (21|1) [0|1] ""
    uint8_t SET_TO_1;                // (13|1) [0|1] ""
    uint8_t ZEROS_BOH2;              // (11|4) [0|1] ""
    uint8_t APPLY_BRAKES_FOR_CANC;   // (23|1) [0|1] ""
    uint8_t ACC_ALERTS;              // (20|5) [0|1] ""
    uint8_t SET_TO_0;                // (22|1) [0|1] ""
    uint8_t HUD_LEAD;                // (40|1) [0|1] ""
    uint8_t SET_TO_64;               // (31|8) [0|255] ""
    uint8_t LEAD_DISTANCE;           // (39|8) [0|255] ""
    uint8_t ZEROS_BOH3;              // (47|7) [0|127] ""
    uint8_t ZEROS_BOH4;              // (55|8) [0|255] ""
    uint8_t COUNTER;                 // (61|2) [0|3] ""
    uint8_t CHECKSUM;                // (59|4) [0|15] ""
} RADAR_HUD;

// ID: 1029
typedef struct {
    uint8_t DOOR_OPEN_FL;       // (37|1) [0|1] ""
    uint8_t DOOR_OPEN_FR;       // (38|1) [0|1] ""
    uint8_t DOOR_OPEN_RL;       // (39|1) [0|1] ""
    uint8_t DOOR_OPEN_RR;       // (40|1) [0|1] ""
    uint8_t TRUNK_OPEN;         // (41|1) [0|1] ""
    uint8_t COUNTER;            // (61|2) [0|3] ""
    uint8_t CHECKSUM;           // (59|4) [0|15] ""
} DOORS_STATUS;

// ID: 1302
typedef struct {
    uint32_t ODOMETER;            // (7|24) [0|16777215] "km"
    uint8_t COUNTER;              // (61|2) [0|3] ""
    uint8_t CHECKSUM;             // (59|4) [0|3] ""
} ODOMETER;

//ID: 13274
typedef struct {
    uint8_t CAM_TEMP_HIGH;           // (7|1) [0|255] ""
    uint8_t SET_ME_X41;              // (6|7) [0|127] ""
    uint8_t BOH;                     // (6|7) [0|127] ""
    uint8_t DASHED_LANES;            // (14|1) [0|1] ""
    uint8_t DTC;                     // (13|1) [0|1] ""
    uint8_t LKAS_PROBLEM;            // (12|1) [0|1] ""
    uint8_t LKAS_OFF;                // (11|1) [0|1] ""
    uint8_t SOLID_LANES;             // (10|1) [0|1] ""
    uint8_t LDW_RIGHT;               // (9|1) [0|1] ""
    uint8_t STEERING_REQUIRED;       // (8|1) [0|1] ""
    uint8_t BOH_2;                   // (23|2) [0|4] ""
    uint8_t LDW_PROBLEM;             // (21|1) [0|1] ""
    uint8_t BEEP;                    // (17|2) [0|1] ""
    uint8_t SET_ME_X01;              // (20|1) [0|1] ""
    uint8_t LDW_ON;                  // (28|1) [0|1] ""
    uint8_t LDW_OFF;                 // (27|1) [0|1] ""
    uint8_t CLEAN_WINDSHIELD;        // (26|1) [0|1] ""
    uint8_t COUNTER;                 // (37|2) [0|3] ""
    uint8_t CHECKSUM;                // (35|4) [0|15] ""
} LKAS_HUD_A;

// ID: 13275
typedef struct {
    uint8_t CAM_TEMP_HIGH;           // (7|1) [0|255] ""
    uint8_t SET_ME_X41;              // (6|7) [0|127] ""
    uint8_t BOH;                     // (6|7) [0|127] ""
    uint8_t DASHED_LANES;            // (14|1) [0|1] ""
    uint8_t DTC;                     // (13|1) [0|1] ""
    uint8_t LKAS_PROBLEM;            // (12|1) [0|1] ""
    uint8_t LKAS_OFF;                // (11|1) [0|1] ""
    uint8_t SOLID_LANES;             // (10|1) [0|1] ""
    uint8_t LDW_RIGHT;               // (9|1) [0|1] ""
    uint8_t STEERING_REQUIRED;       // (8|1) [0|1] ""
    uint8_t BOH_2;                   // (23|2) [0|4] ""
    uint8_t LDW_PROBLEM;             // (21|1) [0|1] ""
    uint8_t BEEP;                    // (17|2) [0|1] ""
    uint8_t SET_ME_X01;              // (20|1) [0|1] ""
    uint8_t LDW_ON;                  // (28|1) [0|1] ""
    uint8_t LDW_OFF;                 // (27|1) [0|1] ""
    uint8_t CLEAN_WINDSHIELD;        // (26|1) [0|1] ""
    uint8_t COUNTER;                 // (61|2) [0|3] ""
    uint8_t CHECKSUM;                // (59|4) [0|15] ""
} LKAS_HUD_B;
