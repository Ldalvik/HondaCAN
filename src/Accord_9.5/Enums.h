#pragma once

// These values usually won't be consistent if not used on a Honda vehicle
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
    WIPER_LOW = 1,
    WIPER_INTELLIGENT = 2,
    WIPER_HIGH = 3
};

enum HeadlightState
{
    LIGHT_OFF = 0,
    LOW_BEAMS = 1,
    HIGH_BEAMS = 3
};

enum ScmButtons
{
    VSA_BUTTON = 1,
    CRUISE_MAIN = 2,
    CRUISE_CANCEL = 4,
    CRUISE_SET = 6,
    CRUISE_RES = 8
};
