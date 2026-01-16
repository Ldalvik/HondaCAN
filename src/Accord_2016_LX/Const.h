#pragma once
#include <cstdint>

enum GearShifter
{
    GEAR_L = 32,
    GEAR_S = 16,
    GEAR_D = 8,
    GEAR_N = 4,
    GEAR_R = 2,
    GEAR_P = 1
};

enum Econ
{
    ECON_OFF = 0,
    ECON_ON = 3
};

enum WiperStatus
{
    WIPER_OFF = 0,
    WIPER_LOW = 8,
    WIPER_INTELLIGENT = 16,
    WIPER_HIGH = 24
};

enum HeadlightState
{
    LIGHT_OFF = 0,
    LOW_BEAMS = 64,
    HIGH_BEAMS = 192
};

enum ScmButtons
{
    CRUISE_RES = 128,
    CRUISE_SET = 96,
    CRUISE_CANCEL = 64,
    CRUISE_MAIN = 32,
    VSA_BUTTON = 16
};

/* 
    TORQUE_REQUEST and TORQUE_ESTIMATE are in "engineering units", a normalized/scaled
    measurement that has no known direct conversion to real engine torque. At the peak torque range
    for my vehicle, this value reached 1100. It does seem to be accurate the power curve of the engine.
    Using a reference table of the cars real torque output will give the most accurate results.
    The ECU uses these values among others to reach the requested engine load.
*/
typedef struct
{
    int16_t TORQUE_ESTIMATE; // Estimated engine torque (normalized)
    int16_t TORQUE_REQUEST;  // Requested engine torque (normalized)
    uint8_t CAR_GAS;         // How far the gas pedal is pressed down
} ENGINE_DATA_1;

typedef struct
{
    int8_t STEER_ANGLE_COARSE; // Coarse steering angle sensor position. 0 is far left, 18 far right
    int16_t STEER_ANGLE;       // Fine steering angle sensor position. -512 is far left, 512 is far right
    int16_t STEER_RATE;        // How fast the wheel is being turned (degrees/s)
} STEERING_SENSORS;

typedef struct
{
    float XMISSION_SPEED;             // Speed from the transmission (meters)
    float XMISSION_SPEED_SPEEDOMETER; // Speed displayed on the speedometer (mph)
    uint32_t CURRENT_TRIP_DISTANCE;   // Meters traveled since vehicle start (overflows at 255)
} ENGINE_DATA_2;

typedef struct
{
    uint8_t PEDAL_GAS;   // How far the gas pedal is pressed down
    uint16_t ENGINE_RPM; // Current engine RPM
    bool GAS_PRESSED;    // Gas pedal pressed
    bool BRAKE_SWITCH;   // Brake switch off (unlocks shifter)
    bool BRAKE_PRESSED;  // Brake pedal pressed
} POWERTRAIN_DATA;

typedef struct
{
    float LAT_ACCEL;  // Latitudinal acceleration m/s^2
    float LONG_ACCEL; // Longitudinal acceleration m/s^2
} KINEMATICS;

typedef struct
{
    GearShifter CURRENT_GEAR; // Current gear selection (PRNDSL)
} GEARBOX;

typedef struct
{
    uint16_t USER_BRAKE; // How far the brake pedal is pressed down
    bool ESP_DISABLED;   // VSA off
} VSA_STATUS;

/* 
    FUEL_LEVEL is the raw sensor value from the tank, meaning it will jump around when the
    vehicle moves. To work around this, there are many algorithms to calculate accurate
    fuel level overtime. (I will add this documentation later). When the engine is off 
    but battery is on, it will read live values so you can see it increase when pumping gas.
*/
typedef struct
{
    ScmButtons SCM_BUTTONS;         // Steering wheel and other buttons (VSA, TPMS, etc)
    HeadlightState HEADLIGHT_STATE; // Headlight stalk status
    bool E_BRAKE;                   // E-brake active
    bool AC_COMPRESSOR_ON;          // A/C compressor status
    uint8_t FUEL_LEVEL;             // Live fuel level (0-105%, raw sensor value)
    bool CRUISE_MAIN_ON;            // Cruise control MAIN status
} SCM_FEEDBACK_1;

typedef struct
{
    bool WHEELS_MOVING; // Returns whether or not any wheel is moving
} STANDSTILL;

typedef struct
{
    float LAT_ACCEL;  // Latitudinal acceleration m/s^2
    float LONG_ACCEL; // Longitudinal acceleration m/s^2
} VEHICLE_DYNAMICS;

typedef struct
{
    Econ ECON_ON; // Economy mode
} DRIVE_MODES;

typedef struct
{
    float WHEEL_SPEED_FL; // Front left wheel speed
    float WHEEL_SPEED_FR; // Front right wheel speed
    float WHEEL_SPEED_RL; // Rear left wheel speed
    float WHEEL_SPEED_RR; // Rear right wheel speed
} ROUGH_WHEEL_SPEED;

typedef struct
{
    bool BLINKER_RIGHT;       // Right blinker active
    bool BLINKER_LEFT;        // Left blinker active
    WiperStatus WIPER_STATUS; // Wiper mode
    uint32_t ODOMETER;        // Vehicle odometer (kilometers)
} SCM_FEEDBACK_2;

typedef struct
{
    bool TPMS_BUTTON; // Tire Pressure Monitoring System button
} TPMS_FEEDBACK;

typedef struct
{
    bool DRIVER_LAMP;      // Driver seatbelt lamp
    bool PASS_UNLATCHED;   // Passenger seatbelt unlatched
    bool DRIVER_UNLATCHED; // Driver seatbelt unlatched
    bool PASS_AIRBAG_OFF;  // Passenger airbag disabled
} SEATBELT_STATUS;

typedef struct
{
    float ROUGH_CAR_SPEED; // Vehicle speed (kph)
} CAR_SPEED;

typedef struct
{
    uint8_t ENGINE_TEMP;         // Engine coolant temperature (celsius)
    uint8_t INTAKE_TEMP;         // Intake air temperature (celsius)
    uint16_t TRIP_FUEL_CONSUMED; // Fuel consumed since vehicle start (uL)
} ENGINE_DATA_3;

typedef struct
{
    bool HOOD_OPEN; // Hood latch open
} HOOD_LATCH;

typedef struct
{
    uint8_t WIPER_SPEED; // Wiper speed. 0 is slowest, 60 is fastest.
} SCM_FEEDBACK_3;

typedef struct
{
    bool DOOR_OPEN_FL; // Front left door open
    bool DOOR_OPEN_FR; // Front right door open
    bool DOOR_OPEN_RL; // Rear left door open
    bool DOOR_OPEN_RR; // Rear right door open
    bool TRUNK_OPEN;   // Trunk open
} SCM_FEEDBACK_4;

struct RawCanData_t
{
    ENGINE_DATA_1 EngineData1;
    STEERING_SENSORS SteeringSensors;
    ENGINE_DATA_2 EngineData2;
    POWERTRAIN_DATA PowertrainData;
    KINEMATICS Kinematics;
    GEARBOX Gearbox;
    VSA_STATUS VsaStatus;
    SCM_FEEDBACK_1 ScmFeedback1;
    STANDSTILL Standstill;
    VEHICLE_DYNAMICS VehicleDynamics;
    DRIVE_MODES DriveModes;
    ROUGH_WHEEL_SPEED RoughWheelSpeed;
    SCM_FEEDBACK_2 ScmFeedback2;
    TPMS_FEEDBACK TpmsFeedback;
    SEATBELT_STATUS SeatbeltStatus;
    CAR_SPEED CarSpeed;
    ENGINE_DATA_3 EngineData3;
    HOOD_LATCH HoodLatch;
    SCM_FEEDBACK_3 ScmFeedback3;
    SCM_FEEDBACK_4 ScmFeedback4;
};

struct Telemetry {
  uint8_t  speed;
  uint16_t rpm;
  float    trip_distance;
  float    fuel_consumed;
  uint8_t  fuel_level;
  GearShifter current_gear;
  ScmButtons scm_buttons;
  bool cruise_main_on;
};
