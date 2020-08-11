#include <Arduino.h>
#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include "AccelStepper.h"
#include "GpsDebug.h"
#include <Wire.h>
#include "HMC5883L.h"
#include <Smoothed.h>    // Include the library
#include "PID_v1.h"
#include "LED.h"


//Debug definitions

//#define GPS_DEFINED_HEADING

//Refers to the geared-down movement of the stand as a whole.
// 200 * (90/20) = steps * micro-steps * bigGear / littleGear
#define STEPS_PER_ROTATION 200 * 16 * 90 / 20
#define HOME_OFFSET 127
#define MOTOR_ACCEL 20
#define SYMMETRIC_MAX 120

#define HOME_SPEED 20
#define NORMAL_SPEED 30

//Pin definitions
#define PIN_LED_R D4
#define PIN_LED_G D3
#define PIN_ENDSTOP A0
#define PIN_DIR D2
#define PIN_STEP D1
#define PIN_MAG_SDA D6
#define PIN_MAG_SCL D5
#define PIN_GPS_TX D8
#define PIN_GPS_RX D7
#define GPS_BAUD 9600
#define GPS_BLOCK 16

enum KeepMode {
    KEEP_HEADING = 'H',
    KEEP_GPS_LOCATION = 'G'
};

enum SerialCommand {
    SET_MODE_HEADING = 'H',
    SET_MODE_GPS = 'G',
    SET_PID_TUNINGS = 'P',
};

LED rLed(PIN_LED_R);
LED gLed(PIN_LED_G);
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);
TinyGPSPlus gps;
SoftwareSerial gpsSS;
HMC5883L mag;
Smoothed<double> filterHeading;
Smoothed<double> filterLng;
Smoothed<double> filterLat;

double declinationAngle = 2.0;
uint32_t statusInterval = 5000;
uint32_t lastStatus = 0;

KeepMode currentMode = KEEP_HEADING;
double standOriginHeading = 0;
long stepperPosAtRead = 0;
long loopTime = 0;
long dLoop = 0;

double heading = 0;
double targetBearing = 0;
double targetLat = 0;
double targetLng = 0;

double standTargetBearing, standHeading, output;
double kp = 2.0l;
double ki = 0.0l;
double kd = 2.0l;

double stepperVelocity = 0;
double stepperAcceleration = 0;

PID sPid(&standHeading, &stepperAcceleration, &standTargetBearing, kp, ki, kd, P_ON_E, REVERSE);

bool isGPSLocked = false;

long standDegToSteps(double deg) {
    return (long) (deg / 360.0l * STEPS_PER_ROTATION);
}

double stepsToStandDeg(long steps) {
    return (double) (steps * 360) / ((double) STEPS_PER_ROTATION);
}

void fail(const char *reason, int LEDMode) {
    digitalWrite(PIN_LED_G, HIGH);
    digitalWrite(PIN_LED_R, LOW);
    Serial.println("ERROR:");
    Serial.println(reason);
    rLed.setState(LEDMode);
    while (true) {
        ESP.wdtFeed();
        rLed.tick();
    } //PREVENT ALL ESCAPE MWAHAHAA
}

double wrap180(double in) {
    while (in < -180) {
        in += 360;
    }
    while (in > 180) {
        in -= 360;
    }
    return in;
}

void setup() {
    Serial.begin(115200);
    Serial.println("\r\n\r\nStarting...");

    /*
    if (!mag.begin(PIN_MAG_SDA, PIN_MAG_SCL)) {
        Serial.println("Failed to init magnetometer!");
        exit(0);
    }

    HMC5833LConfiguration cfg = {};
    cfg.gain = HMC5883L_CFG::GAIN::G_1_3GA;
    cfg.samples = HMC5883L_CFG::SAMPLES::S1;
    cfg.mode = HMC5883L_CFG::MODE::SINGLE;
    mag.setConfiguration(cfg);
*/

    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_ENDSTOP, INPUT);

    //Setup components
    digitalWrite(PIN_LED_G, LOW);
    digitalWrite(PIN_LED_R, LOW);

    sPid.SetMode(AUTOMATIC);
    sPid.SetOutputLimits(-30, 30);

    //Home the stepper.
    Serial.println("Homing!");
    stepper.setMaxSpeed(standDegToSteps(HOME_SPEED));
    stepper.setAcceleration(standDegToSteps(MOTOR_ACCEL));
    stepper.setMinPulseWidth(5);

    stepper.setSpeed(standDegToSteps(-HOME_SPEED));

    rLed.setState(LED::FAST_BLINK);
    gLed.setState(LED::FAST_BLINK);
    uint32_t tOut = millis() + 20000;
    while (analogRead(PIN_ENDSTOP) < 1020) {
        stepper.runSpeed();
        ESP.wdtFeed();
        rLed.tick();
        gLed.tick();

        if (millis() > tOut) {
            fail("Homing Failed!", LED::CONSTANT_BLINK);
        }
    }
    stepper.setCurrentPosition(-standDegToSteps(HOME_OFFSET));
    //stepper.runToNewPosition(0);
    //delay(10000);
    Serial.println("Homed!");


    stepper.setMaxSpeed(standDegToSteps(NORMAL_SPEED));
    stepper.setAcceleration(standDegToSteps(MOTOR_ACCEL));


    //Start critical components (If these fail *cough* i2c *cough*, red LED will stay lit indicating fault)
    //Done here B/C the endstop can crash i2c B/C of the way it's connected :/
    digitalWrite(PIN_LED_G, HIGH);
    digitalWrite(PIN_LED_R, LOW);

    Serial.println("Starting GPS serial and magnetometer!");
    gpsSS.begin(GPS_BAUD, SWSERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX, false, 4 * GPS_BLOCK);
    Wire.begin(PIN_MAG_SDA, PIN_MAG_SCL);

    if (!mag.begin(PIN_MAG_SDA, PIN_MAG_SCL)) {
        Serial.println("Failed to init magnetometer!");
        exit(0);
    }

    Serial.println("Applying configurations!");
    HMC5833LCalibration cal;
    cal.xGainFactor = 1.028821;
    cal.yGainFactor = 1.050185l;
    cal.zGainFactor = 0.927202l;
    cal.xOffset = -595.327452;
    cal.yOffset = -885.181265;
    cal.zOffset = 650.424602;
    mag.setCalibration(cal);

    filterHeading.begin(SMOOTHED_EXPONENTIAL, 10);
    filterLng.begin(SMOOTHED_EXPONENTIAL, 10);
    filterLat.begin(SMOOTHED_EXPONENTIAL, 10);


    HMC5833LConfiguration cfg = {};
    cfg.gain = HMC5883L_CFG::GAIN::G_1_3GA;
    cfg.rate = HMC5883L_CFG::RATE::D_75HZ;
    cfg.samples = HMC5883L_CFG::SAMPLES::S8;
    cfg.mode = HMC5883L_CFG::MODE::SINGLE;
    mag.setConfiguration(cfg);

    Serial.println("Started! :)");

    digitalWrite(PIN_LED_G, LOW);
    digitalWrite(PIN_LED_R, LOW);

    lastStatus = millis();
}

void loop() {
    dLoop = micros() - loopTime;
    loopTime = micros();
    if (millis() - lastStatus > statusInterval) {
        Serial.printf("\n-----------------------STATUS-----------------------\n");
        switch (currentMode) {
            case KEEP_HEADING:
                Serial.println("Mode: KEEP_HEADING");
                break;
            case KEEP_GPS_LOCATION:
                Serial.println("Mode: KEEP_GPS_LOCATION");
                break;
        }
        Serial.printf("Lat:%f\tLng:%f\tSat:%d\n", gps.location.lat(), gps.location.lng(), gps.satellites.value());
        Serial.printf("Target:%f\tCurrent:%f\tOrigin Heading:%f\n", targetBearing, heading, standOriginHeading);
        lastStatus += statusInterval;
    }

    if (Serial.available()) {
        static char buf[255];
        static size_t head = 0;
        char c = Serial.read();
        buf[head] = c;
        head++;
        if (c == '\n') {
            char *pch;
            buf[head - 1] = 0;

            switch (buf[0]) { //Switch on command char.
                case SET_MODE_HEADING:
                    targetBearing = strtod(&buf[1], &pch);
                    Serial.println(buf);
                    currentMode = KEEP_HEADING;
                    Serial.printf("New target bearing: %f\n", targetBearing);
                    break;

                case SET_MODE_GPS:
                    targetLat = strtod(&buf[1], &pch);
                    targetLng = strtod(pch, &pch);
                    Serial.printf("New target GPS lat,lon: %f,%f\n", targetLat, targetLng);
                    if (isGPSLocked) {
                        Serial.printf("Distance to location: %f\n",
                                      gps.distanceBetween(targetLat, targetLng, filterLat.get(), filterLng.get()));
                    }
                    currentMode = KEEP_GPS_LOCATION;
                    break;
                case SET_PID_TUNINGS:
                    kp = strtod(&buf[1], &pch);
                    ki = strtod(pch, &pch);
                    kd = strtod(pch, &pch);

                    Serial.printf("New tunings: kp=%f\tki=%f\tkd=%f\n", kp, ki, kd);
                    sPid.SetTunings(kp, ki, kd);
                    break;
                default:
                    break;
            }

            //Serial.println("-------");
            //Serial.println(buf);

            //kp = strtod(buf, &pch);
            // ki = strtod(pch, &pch);
            // kd = strtod(pch, &pch);

            //Serial.printf("kP:%f\tkI:%f\tkD:%f\r\n", kp, ki, kd);
            //Serial.printf("New target bearing:%f\n", targetBearing);
            //Serial.printf("kP:%f\tkI:%f\tkD:%f\r\n", kp, ki, kd);
            //sPid.SetTunings(kp, ki, kd);


            for (size_t i = 0; i < head; i++) { buf[i] = 0; }
            head = 0;
        }
    }

    //GPS Handling code
    if (gpsSS.available()) {
        gps.encode(gpsSS.read());
    }

    if (gps.location.isUpdated()) {
        static bool lastLockState = false; //Used to track GPS lock state *changes*
        isGPSLocked = gps.location.lat() != 0 && gps.location.lng() != 0;


        if (isGPSLocked != lastLockState) {
            Serial.printf("GPS LOCKED WITH %d SATELLITES!\n", gps.satellites.value());
            digitalWrite(PIN_LED_G, LOW);
            digitalWrite(PIN_LED_R, HIGH);
        }

        if (!isGPSLocked) Serial.printf("GPS not yet locked with %d satellites.\n", gps.satellites.value());

        filterLng.add(gps.location.lng());
        filterLat.add(gps.location.lat());

        lastLockState = isGPSLocked;
    }

    //Compass handling code.
    if (mag.didReadBegin()) { //Runs once per reading
        stepperPosAtRead = stepper.currentPosition();
    }

    if (mag.available()) {
        Vector3<double> data = mag.calibratedRead();
        heading = -(atan2(data.x, -data.y) * 180 / M_PI) + declinationAngle;
        heading = wrap180(heading);
        //filterHeading.add(heading);

        //Update target bearing based on current mode.
        switch (currentMode) {
            case KEEP_HEADING: {
                break; //Target bearing was written when set, don't change anything.
            }
            case KEEP_GPS_LOCATION: {
                if (isGPSLocked) { //Both lat and long are non-zero
                    double lat = filterLat.get();
                    double lng = filterLng.get();

                    targetBearing = gps.courseTo(lat, lng, targetLat, targetLng);
                    targetBearing = wrap180(targetBearing);
                }
                break;
            }
            default:
                break;
        }


        standOriginHeading = heading - stepsToStandDeg(stepperPosAtRead);
        standHeading = stepsToStandDeg(stepperPosAtRead); //Current
        standTargetBearing = wrap180(targetBearing - standOriginHeading);
        standOriginHeading = wrap180(standOriginHeading);

        //Serial.printf("SATS:%d\tLNG:%f\tLAT:%f\r\n", gps.satellites.value(), gps.location.lng(), gps.location.lat());
        //Serial.printf("%f,%f,%f\t%f,%f\t%d\r\n", targetBearing, current, output, data.x, data.y,
        //              gps.satellites.value());
        //Serial.printf("Current:%f\tOrigin:%f\tStand:%f\tStandTarget:%f\r\n", heading, standOriginHeading, standHeading, standTargetBearing);
        //Serial.printf("");
        //Serial.printf("Heading: %f\tStandDeg:%f\tSteps:%ld\n", headingDegrees, standDeg, stepper.currentPosition());
        //Serial.printf("%f\t%ld\t%ld\r\n", stepsToStandDeg(stepper.currentPosition()), stepper.currentPosition(), standDegToSteps(output));
        //Serial.printf("%f\t%f\r\n", data.x, data.y);
    }

/*
    sPid.Compute();
    long newSpeed = standDegToSteps(-output);


    */
    sPid.Compute();

    //If within 10 degrees of target, switch to PID-controlled mode.
    if ((stepsToStandDeg(stepper.currentPosition()) > standTargetBearing - 10) &&
        (stepsToStandDeg(stepper.currentPosition()) < standTargetBearing + 10) ) {
        stepperVelocity += -stepperAcceleration * (double) dLoop / 1000000.0l;

        if ((stepperVelocity < 0l && stepsToStandDeg(stepper.currentPosition()) <= -SYMMETRIC_MAX) ||
            (stepperVelocity > 0l && stepsToStandDeg(stepper.currentPosition()) >= SYMMETRIC_MAX)) {
            stepper.setSpeed(0);
        } else {
            stepper.setSpeed(standDegToSteps(stepperVelocity));
        }

        stepper.runSpeed();

    } else {
        if (standTargetBearing > SYMMETRIC_MAX) {
            stepper.moveTo(standDegToSteps(SYMMETRIC_MAX));
        } else if (standTargetBearing < -SYMMETRIC_MAX) {
            stepper.moveTo(standDegToSteps(-SYMMETRIC_MAX));
        } else {
            //stepper.speed();
            stepper.moveTo(standDegToSteps(standTargetBearing));
        }
        //stepperAcceleration = 0;
        stepperVelocity = stepsToStandDeg((long) stepper.speed());
    }


    stepper.run();
    mag.loop();
}
