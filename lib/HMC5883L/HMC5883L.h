//
// Created by Dominic on 8/3/2020.
//

#ifndef HMC5883L_H
#define HMC5883L_H

#include <Arduino.h>
#include "wire.h"
#include "HMC5883l_cfg.h"
#include "AsyncI2CMaster.h"


using namespace HMC5883L_CFG;

template<typename T = float>
struct Vector3 {
    T x = 0;
    T z = 0;
    T y = 0;
};

struct RequestArg {
    bool lock = true;
    uint8_t status = 0;
    uint8_t *buffer;
    uint8_t dataLen;
public:
    RequestArg(uint8_t *buf, uint8_t len) {
        buffer = buf;
        dataLen = len;
        lock = true;
        status = 0;
    }
};

struct SendArg {
    bool lock = true;
    uint8_t status = 0;
};


//FSM functions
enum struct STATE : int {
    WAITING_TO_WRITE_MODE,
    WAITING_TO_REQUEST,
    REQUESTING_DATA,
    DATA_RECEIVED,
    WAITING,
    STOPPED,
    INIT,
    TEST,
};

struct FSMAsyncArg {
    uint8_t *buffer;
    STATE *state;
    STATE cbState;
    uint8_t *status;
};

struct HMC5833LConfiguration {
    SAMPLES samples = SAMPLES::S1;
    RATE rate = RATE::D_30HZ;
    FLOW flow = FLOW::NONE;
    GAIN gain = GAIN::G_1_3GA;
    MODE mode = MODE::SINGLE;
    uint32_t sampleInterval = 1000000 / 100;
};

struct HMC5833LCalibration {
    double xGainFactor = 1;
    double yGainFactor = 1;
    double zGainFactor = 1;
    double xOffset = 0;
    double yOffset = 0;
    double zOffset = 0;
};


class HMC5883L {
private:
    AsyncI2CMaster aWire;

    //Configuration Structs
    HMC5833LConfiguration configuration;
    HMC5833LCalibration calibration;

    //Data vars
    Vector3<int16_t> rawData;
    uint8_t buffer[16];

    //Internal state tracking vars.
    uint32_t lastReadTime = 0;
    bool isDataAvailable_TRK = false;
    bool didReadBegin_TRK = false;
    bool isInitialized = false;
    uint8_t i2cStatus = I2C_STATUS_OK;
    STATE state = STATE::INIT;


    //***********************************************************
    //| Private methods to interface with the raw HMC registers |
    //***********************************************************

    //Read [len] bytes into [buf].
    //Optional starting address [reg] can be passed to read from that address.
    uint8_t readSync(uint8_t *buf, uint8_t len);

    uint8_t readSync(uint8_t *buf, uint8_t len, uint8_t reg);

    //Read a single byte from either the current or [reg] register and return it.
    uint8_t readSync8();

    uint8_t readSync8(uint8_t reg);

    //Write [len] bytes from [data] to the HMC's registers.
    //A register address [reg] can be passed to start from that address.
    //
    // NOTE: Because of how the HMC handles pointing, the first byte of any
    // raw data will specify a register address and not actually be written.
    void writeSync(uint8_t *data, uint8_t len, uint8_t reg);

    void writeSync(uint8_t *data, uint8_t len);

    void writeSync8(uint8_t data, uint8_t reg);

    void writeSync8(uint8_t data);

    //Sets the current register pointer to [reg]
    void setRegisterPointer(uint8_t reg);

    //Async R/W operations ONLY to be used inside the FSM. They directly modify state and
    // all data is written to the class's [buffer] variable.
    void readAsync(REGISTERS rootReg, uint8_t len, STATE nextState);

    void readAsync(uint8_t len, STATE nextState);

    void writeAsync(uint8_t *data, uint8_t len, STATE nextState);

    void writePointerAsync(REGISTERS reg, STATE nextState);

    //A struct used to pass args needed for FSM operation to async R/W calls.
    FSMAsyncArg fsmAsyncArg = {buffer, &state, STATE::STOPPED, &i2cStatus};

public:
    bool begin(int sda, int sdl);

    bool available() { return isDataAvailable_TRK; }

    bool didReadBegin() {
        if (didReadBegin_TRK) {
            didReadBegin_TRK = false;
            return true;
        }
        return false;
    }

    void loop();

    //Conversion functions
    Vector3<int16_t> bufToRawVec(uint8_t *buffer);

    Vector3<double> rawVecToCalibratedVec(Vector3<int16_t> rawData, HMC5833LCalibration calibration);

    //Read functions
    void rawReadInto(int16_t *x, int16_t *y, int16_t *z);

    void calibratedReadInto(double *x, double *y, double *z);

    Vector3<int16_t> rawRead();

    Vector3<double> calibratedRead();

    //Configuration functions
    bool setConfiguration(HMC5833LConfiguration configuration);

    HMC5833LConfiguration getConfiguration() { return configuration; }

    void setCalibration(HMC5833LCalibration calibration) { this->calibration = calibration; }

    HMC5833LCalibration setCalibration() { return calibration; }

    //Calibration functions
    HMC5833LCalibration preformInteractiveCalibration(Stream &stream);

    Vector3<double> preformGainCalibration(Stream &stream, FLOW flow, Vector3<double> targetExcitation);

    Vector3<double> preformOffsetCalibration(Stream &stream, HMC5833LCalibration gainCalibration);

    static void printCalibration(Stream &stream, HMC5833LCalibration calibration);


};


#endif //HMC5883L_H
