/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "mw.h"

#include "cli.h"
#include "telemetry_common.h"

#include "buzzer.h"

flags_t f;
int16_t debug[4];
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
int16_t headFreeModeHold;

uint16_t vbat;                  // battery voltage in 0.1V steps
int32_t amperage;               // amperage read by current sensor in centiampere (1/100th A)
int32_t mAhdrawn;              // milliampere hours drawn from the battery since start
int16_t telemTemperature1;      // gyro sensor temperature

int16_t failsafeCnt = 0;
int16_t failsafeEvents = 0;
int16_t rcData[RC_CHANS];       // interval [1000;2000]
int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];     // lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE
uint16_t rssi;                  // range: [0;1023]
rcReadRawDataPtr rcReadRawFunc = NULL;  // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)

static void pidMultiWii(void);
static void pidRewrite(void);
pidControllerFuncPtr pid_controller = pidMultiWii; // which pid controller are we using, defaultMultiWii

uint8_t dynP8[3], dynI8[3], dynD8[3];
uint8_t rcOptions[CHECKBOXITEMS];

int16_t axisPID[3];

// **********************
// GPS
// **********************
int32_t GPS_coord[2];
int32_t GPS_home[3];
int32_t GPS_hold[3];
uint8_t GPS_numSat;
uint16_t GPS_distanceToHome;        // distance to home point in meters
int16_t GPS_directionToHome;        // direction to home or hol point in degrees
uint16_t GPS_altitude, GPS_speed;   // altitude in 0.1m and speed in 0.1m/s
uint8_t GPS_update = 0;             // it's a binary toogle to distinct a GPS position update
int16_t GPS_angle[3] = { 0, 0, 0 }; // it's the angles that must be applied for GPS correction
uint16_t GPS_ground_course = 0;     // degrees * 10
int16_t nav[2];
int16_t nav_rated[2];               // Adding a rate controller to the navigation to make it smoother
int8_t nav_mode = NAV_MODE_NONE;    // Navigation mode
uint8_t GPS_numCh;                  // Number of channels
uint8_t GPS_svinfo_chn[32];         // Channel number
uint8_t GPS_svinfo_svid[32];        // Satellite ID
uint8_t GPS_svinfo_quality[32];     // Bitfield Qualtity
uint8_t GPS_svinfo_cno[32];         // Carrier to Noise Ratio (Signal Strength)
uint32_t GPS_update_rate[2];        // GPS coordinates updating rate (column 0 = last update time, 1 = current update ms)
uint32_t GPS_svinfo_rate[2];        // GPS svinfo updating rate (column 0 = last update time, 1 = current update ms)
uint32_t GPS_HorizontalAcc;         // Horizontal accuracy estimate (mm)
uint32_t GPS_VerticalAcc;           // Vertical accuracy estimate (mm)

// Automatic ACC Offset Calibration
bool AccInflightCalibrationArmed = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm = false;
bool AccInflightCalibrationActive = false;
uint16_t InflightcalibratingA = 0;

// Battery monitoring stuff
uint8_t batteryCellCount = 3;       // cell count
uint16_t batteryWarningVoltage;     // slow buzzer after this one, recommended 80% of battery used. Time to land.
uint16_t batteryCriticalVoltage;    // annoying buzzer after this one, battery is going to be dead.

// Time of automatic disarm when "Don't spin the motors when armed" is enabled.
static uint32_t disarmTime = 0;

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LED0_TOGGLE;            // switch LEDPIN state
            BEEP_ON;
            delay(wait);
            BEEP_OFF;
        }
        delay(60);
    }
}

void annexCode(void)
{
    static uint32_t calibratedAccTime;
    int32_t tmp, tmp2;
    int32_t axis, prop1, prop2;

    // vbat shit
    static uint8_t vbatTimer = 0;
    static int32_t vbatRaw = 0;
    static int32_t vbatCycleTime = 0;

    // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < cfg.tpa_breakpoint) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t)cfg.dynThrPID * (rcData[THROTTLE] - cfg.tpa_breakpoint) / (2000 - cfg.tpa_breakpoint);
        } else {
            prop2 = 100 - cfg.dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++) {
        tmp = min(abs(rcData[axis] - mcfg.midrc), 500);
        if (axis != 2) {        // ROLL & PITCH
            if (cfg.deadband) {
                if (tmp > cfg.deadband) {
                    tmp -= cfg.deadband;
                } else {
                    tmp = 0;
                }
            }

            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
            prop1 = 100 - (uint16_t)cfg.rollPitchRate[axis] * tmp / 500;
            prop1 = (uint16_t)prop1 * prop2 / 100;
        } else {                // YAW
            if (cfg.yawdeadband) {
                if (tmp > cfg.yawdeadband) {
                    tmp -= cfg.yawdeadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = tmp * -mcfg.yaw_control_direction;
            prop1 = 100 - (uint16_t)cfg.yawRate * abs(tmp) / 500;
        }
        dynP8[axis] = (uint16_t)cfg.P8[axis] * prop1 / 100;
        dynI8[axis] = (uint16_t)cfg.I8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)cfg.D8[axis] * prop1 / 100;
        if (rcData[axis] < mcfg.midrc)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rcData[THROTTLE], mcfg.mincheck, 2000);
    tmp = (uint32_t)(tmp - mcfg.mincheck) * 1000 / (2000 - mcfg.mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    if (f.HEADFREE_MODE) {
        float radDiff = (heading - headFreeModeHold) * M_PI / 180.0f;
        float cosDiff = cosf(radDiff);
        float sinDiff = sinf(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }

    if (feature(FEATURE_VBAT)) {
        vbatCycleTime += cycleTime;
        if (!(++vbatTimer % VBATFREQ)) {
            vbatRaw -= vbatRaw / 8;
            vbatRaw += adcGetChannel(ADC_BATTERY);
            vbat = batteryAdcToVoltage(vbatRaw / 8);

        }
        // Buzzers for low and critical battery levels
        if (vbat <= batteryCriticalVoltage)
            buzzer(BUZZER_BAT_CRIT_LOW);     // Critically low battery
        else if (vbat <= batteryWarningVoltage)
            buzzer(BUZZER_BAT_LOW);     // low battery
    }
    // update buzzer handler
    buzzerUpdate();

    if ((calibratingA > 0 && sensors(SENSOR_ACC)) || (calibratingG > 0)) {      // Calibration phasis
        LED0_TOGGLE;
    } else {
        if (f.ACC_CALIBRATED)
            LED0_OFF;
        if (f.ARMED)
            LED0_ON;

#ifndef CJMCU
        checkTelemetryState();
#endif
    }

#ifdef LEDRING
    if (feature(FEATURE_LED_RING)) {
        static uint32_t LEDTime;
        if ((int32_t)(currentTime - LEDTime) >= 0) {
            LEDTime = currentTime + 50000;
            ledringState();
        }
    }
#endif

    if ((int32_t)(currentTime - calibratedAccTime) >= 0) {
        if (!f.SMALL_ANGLE) {
            f.ACC_CALIBRATED = 0; // the multi uses ACC and is not calibrated or is too much inclinated
            LED0_TOGGLE;
            calibratedAccTime = currentTime + 500000;
        } else {
            f.ACC_CALIBRATED = 1;
        }
    }

    serialCom();

#ifndef CJMCU
    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        handleTelemetry();
    }
#endif

    if (sensors(SENSOR_GPS)) {
        static uint32_t GPSLEDTime;
        if ((int32_t)(currentTime - GPSLEDTime) >= 0 && (GPS_numSat >= 5)) {
            GPSLEDTime = currentTime + 150000;
            LED1_TOGGLE;
        }
    }

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature)
        gyro.temperature(&telemTemperature1);
    else {
        // TODO MCU temp
    }
}

uint16_t pwmReadRawRC(uint8_t chan)
{
    return pwmRead(mcfg.rcmap[chan]);
}

void computeRC(void)
{
    uint16_t capture;
    int i, chan;

    if (feature(FEATURE_SERIALRX)) {
        for (chan = 0; chan < 8; chan++)
            rcData[chan] = rcReadRawFunc(chan);
    } else {
        static int16_t rcDataAverage[8][4];
        static int rcAverageIndex = 0;

        for (chan = 0; chan < 8; chan++) {
            capture = rcReadRawFunc(chan);

            // validate input
            if (capture < PULSE_MIN || capture > PULSE_MAX)
                capture = mcfg.midrc;
            rcDataAverage[chan][rcAverageIndex % 4] = capture;
            // clear this since we're not accessing it elsewhere. saves a temp var
            rcData[chan] = 0;
            for (i = 0; i < 4; i++)
                rcData[chan] += rcDataAverage[chan][i];
            rcData[chan] /= 4;
        }
        rcAverageIndex++;
    }
}

static void mwArm(void)
{
    if (calibratingG == 0 && f.ACC_CALIBRATED) {
        // TODO: feature(FEATURE_FAILSAFE) && failsafeCnt < 2
        // TODO: && ( !feature || ( feature && ( failsafecnt > 2) )
        if (!f.ARMED) {         // arm now!
            f.ARMED = 1;
            headFreeModeHold = heading;
            // Beep for inform about arming
#ifdef GPS
            if (feature(FEATURE_GPS) && f.GPS_FIX && GPS_numSat >= 5)
                buzzer(BUZZER_ARMING_GPS_FIX);
            else
                buzzer(BUZZER_ARMING);
#else
            buzzer(BUZZER_ARMING);
#endif
        }
    } else if (!f.ARMED) {
        blinkLED(2, 255, 1);
    }
}

static void mwDisarm(void)
{
    if (f.ARMED) {
        f.ARMED = 0;
        // Beep for inform about disarming
        buzzer(BUZZER_DISARMING);
        // Reset disarm time so that it works next time we arm the board.
        if (disarmTime != 0)
            disarmTime = 0;
    }
}

static void mwVario(void)
{

}

static int32_t errorGyroI[3] = { 0, 0, 0 };
static int32_t errorAngleI[2] = { 0, 0 };

static void pidMultiWii(void)
{
    int axis, prop;
    int32_t error, errorAngle;
    int32_t PTerm, ITerm, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int32_t delta1[3], delta2[3];
    int32_t deltaSum;
    int32_t delta;

    // **** PITCH & ROLL & YAW PID ****
    prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])); // range [0;500]
    for (axis = 0; axis < 3; axis++) {
        if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2) { // MODE relying on ACC
            // 50 degrees max inclination
            errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int)mcfg.max_angle_inclination), +mcfg.max_angle_inclination) - angle[axis] + cfg.angleTrim[axis];
            PTermACC = errorAngle * cfg.P8[PIDLEVEL] / 100; // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
            PTermACC = constrain(PTermACC, -cfg.D8[PIDLEVEL] * 5, +cfg.D8[PIDLEVEL] * 5);

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            ITermACC = (errorAngleI[axis] * cfg.I8[PIDLEVEL]) >> 12;
        }
        if (!f.ANGLE_MODE || f.HORIZON_MODE || axis == 2) { // MODE relying on GYRO or YAW axis
            error = (int32_t)rcCommand[axis] * 10 * 8 / cfg.P8[axis];
            error -= gyroData[axis];

            PTermGYRO = rcCommand[axis];

            errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
            if ((abs(gyroData[axis]) > 640) || ((axis == YAW) && (abs(rcCommand[axis]) > 100)))
                errorGyroI[axis] = 0;
            ITermGYRO = (errorGyroI[axis] / 125 * cfg.I8[axis]) >> 6;
        }
        if (f.HORIZON_MODE && axis < 2) {
            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } else {
            if (f.ANGLE_MODE && axis < 2) {
                PTerm = PTermACC;
                ITerm = ITermACC;
            } else {
                PTerm = PTermGYRO;
                ITerm = ITermGYRO;
            }
        }

        PTerm -= (int32_t)gyroData[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation
        delta = gyroData[axis] - lastGyro[axis];
        lastGyro[axis] = gyroData[axis];
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * dynD8[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
}

#define GYRO_I_MAX 256

static void pidRewrite(void)
{
    int32_t errorAngle = 0;
    int axis;
    int32_t delta, deltaSum;
    static int32_t delta1[3], delta2[3];
    int32_t PTerm, ITerm, DTerm;
    static int32_t lastError[3] = { 0, 0, 0 };
    int32_t AngleRateTmp, RateError;

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        if (axis == 2) { // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
            AngleRateTmp = (((int32_t)(cfg.yawRate + 27) * rcCommand[YAW]) >> 5);
        } else {
            // calculate error and limit the angle to 50 degrees max inclination
            errorAngle = (constrain(rcCommand[axis] + GPS_angle[axis], -500, +500) - angle[axis] + cfg.angleTrim[axis]) / 10.0f; // 16 bits is ok here
            if (!f.ANGLE_MODE) { //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                AngleRateTmp = ((int32_t)(cfg.rollPitchRate[axis] + 27) * rcCommand[axis]) >> 4;

                if (f.HORIZON_MODE) {
                    // mix up angle error to desired AngleRateTmp to add a little auto-level feel
                    AngleRateTmp += (errorAngle * cfg.I8[PIDLEVEL]) >> 8;
                }
            } else { // it's the ANGLE mode - control is angle based, so control loop is needed
                AngleRateTmp = (errorAngle * cfg.P8[PIDLEVEL]) >> 4;
            }
        }

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRateTmp - gyroData[axis];

        // -----calculate P component
        PTerm = (RateError * cfg.P8[axis]) >> 7;
        // -----calculate I component
        // there should be no division before accumulating the error to integrator, because the precision would be reduced.
        // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        // Time correction (to avoid different I scaling for different builds based on average cycle time)
        // is normalized to cycle time = 2048.
        errorGyroI[axis] = errorGyroI[axis] + ((RateError * cycleTime) >> 11) * cfg.I8[axis];

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t)(-GYRO_I_MAX) << 13, (int32_t)(+GYRO_I_MAX) << 13);
        ITerm = errorGyroI[axis] >> 13;

        //-----calculate D-term
        delta = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastError[axis] = RateError;

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta = (delta * ((uint16_t)0xFFFF / (cycleTime >> 4))) >> 6;
        // add moving average here to reduce noise
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * cfg.D8[axis]) >> 8;

        // -----calculate total PID output
        axisPID[axis] = PTerm + ITerm + DTerm;
    }
}

void setPIDController(int type)
{
    switch (type) {
        case 0:
        default:
            pid_controller = pidMultiWii;
            break;
        case 1:
            pid_controller = pidRewrite;
            break;
    }
}

void loop(void)
{	
    static uint32_t loopTime;	

    currentTime = micros();
    if (mcfg.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + mcfg.looptime;

        // Measure loop rate just afer reading the sensors
        currentTime = micros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;
        
			  //µÿ√Ê’æ
        serialCom();

			  //Update Stick
			  motor[0] = 1000 + adcGetChannel(ADC_STICK_RIGHT_ROLL) / 4;
			  motor[1] = 1000 + adcGetChannel(ADC_STICK_RIGHT_PITCH) / 4;
			  motor[2] = 1000 + adcGetChannel(ADC_STICK_LEFT_PITCH) / 4;
			  motor[3] = 1000 + adcGetChannel(ADC_STICK_LEFT_ROLL) / 4;			  
			  motor[4] = 1000 + adcGetChannel(ADC_TRIM_RIGHT_ROLL) / 4;
			  
			  motor[5] = btnChannel[0];
			  motor[6] = btnChannel[1];
			  motor[7] = btnChannel[2];
    }
		
		btnScan(); 		
		RFLoop();  
		buzzerUpdate();
}
