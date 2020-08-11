//
// Created by Dominic on 8/3/2020.
//

#include "HMC5883L.h"

using namespace HMC5883L_CFG;

bool HMC5883L::begin(int sda, int scl) {
    aWire.begin(sda, scl);
    if ((readSync8(REGISTERS::IDENT_A) != EXPECTED_IDENT::A) ||
        (readSync8(REGISTERS::IDENT_B) != EXPECTED_IDENT::B) ||
        (readSync8(REGISTERS::IDENT_C) != EXPECTED_IDENT::C)) {
        return false;
    }
    isInitialized = true;

    /*Init*/
    return setConfiguration(configuration);
}

bool HMC5883L::setConfiguration(HMC5833LConfiguration configuration) {
    this->configuration = configuration;
    if (isInitialized) {
        //Generate configuration bytes (Masks ensure cfg. errors don't affect other things.)
        uint8_t A = 0x00;
        A |= configuration.samples << 5 & SAMPLES_MASK;
        A |= configuration.rate << 2 & DATA_RATE_MASK;
        A |= configuration.flow & MEASUREMENT_FLOW_MASK;

        uint8_t B = 0x00;
        B |= configuration.gain << 5 & GAIN_MASK;

        uint8_t M = 0x00;
        M |= configuration.mode;

        //Write configuration bytes
        writeSync8(A, REGISTERS::CONFIG_A);
        writeSync8(B, REGISTERS::CONFIG_B);
        writeSync8(M, REGISTERS::MEASUREMENT_MODE);

        //Check config applied correctly.
        return (readSync8(REGISTERS::CONFIG_A) == A) && (readSync8(REGISTERS::CONFIG_B) == B) &&
               (readSync8(REGISTERS::MEASUREMENT_MODE) == M);

    }
    return false;
}

void HMC5883L::loop() {
    if (!isInitialized) { return; }
    switch (state) {
        case STATE::INIT : {
            state = STATE::WAITING_TO_WRITE_MODE;
            break;
        }
        case STATE::WAITING_TO_WRITE_MODE: {

            uint32_t t = micros();
            if (t - lastReadTime > configuration.sampleInterval) {
                while (t - lastReadTime > configuration.sampleInterval) { lastReadTime += configuration.sampleInterval; }
                uint8_t req[2] = {REGISTERS::MEASUREMENT_MODE, MODE::SINGLE};
                didReadBegin_TRK = true;
                writeAsync(req, 2, STATE::WAITING_TO_REQUEST);
            }
            break;
        }
        case STATE::WAITING_TO_REQUEST: {
            uint32_t t = micros();
            if (t - lastReadTime > 6000) { //Wait 10ms
                state = STATE::REQUESTING_DATA;
            }
            break;
        }
        case STATE::REQUESTING_DATA: {
            //Serial.println("REQ");
            readAsync(6, STATE::DATA_RECEIVED);
            break;
        }
        case STATE::DATA_RECEIVED: {
            //Serial.println("REC");
            rawData.x = ((uint16_t) buffer[0]) << 8 | buffer[1];
            rawData.z = ((uint16_t) buffer[2]) << 8 | buffer[3];
            rawData.y = ((uint16_t) buffer[4]) << 8 | buffer[5];
            isDataAvailable_TRK = true;
            state = STATE::WAITING_TO_WRITE_MODE;
            break;
        }
        case STATE::WAITING: {
            break;
        }
        default:
            break;
    }
    aWire.loop();
}

void HMC5883L::readAsync(uint8_t len, STATE nextState) {
//If state is already waiting, exit - we shouldn't be called twice in the same state.
    if (state == STATE::WAITING) { return; }

    //Modify our held argument request with the new callback state.
    fsmAsyncArg.cbState = nextState;
    uint8_t reg = REGISTERS::OUT_X_M;
    //Send [rootreg] to set the register pointer and request [len] bytes.
    aWire.request(ADDRESS, &reg, 1, len,
                  [](uint8_t status, void *arg, uint8_t *data, uint8_t datalen) {

                      //Cast void * arg to the FMSRequestArg struct we passed earlier.
                      FSMAsyncArg *a = static_cast<FSMAsyncArg *>(arg);

                      //Copy data from i2c buffer to the buffer passed in the arg struct
                      memcpy(a->buffer, data, datalen);

                      //Dereference and set the passed status variable.
                      *(a->status) = status;

                      //Dereference and set the passed state variable to the passed new state.
                      *(a->state) = a->cbState;
                  }, static_cast<void *>(&fsmAsyncArg));
    state = STATE::WAITING;
}

void HMC5883L::readAsync(REGISTERS rootReg, uint8_t len, STATE nextState) {
    //If state is already waiting, exit - we shouldn't be called twice in the same state.
    if (state == STATE::WAITING) { return; }

    //Modify our held argument request with the new callback state.
    fsmAsyncArg.cbState = nextState;

    //Send [rootreg] to set the register pointer and request [len] bytes.
    aWire.request(ADDRESS, (uint8_t *) &rootReg, 1, len,
                  [](uint8_t status, void *arg, uint8_t *data, uint8_t datalen) {

                      //Cast void * arg to the FMSRequestArg struct we passed earlier.
                      FSMAsyncArg *a = static_cast<FSMAsyncArg *>(arg);

                      //Copy data from i2c buffer to the buffer passed in the arg struct
                      memcpy(a->buffer, data, datalen);

                      //Dereference and set the passed status variable.
                      *(a->status) = status;

                      //Dereference and set the passed state variable to the passed new state.
                      *(a->state) = a->cbState;
                  }, static_cast<void *>(&fsmAsyncArg));
    state = STATE::WAITING;
}

void HMC5883L::writeAsync(uint8_t *data, uint8_t len, STATE nextState) {
    //If state is already waiting, exit - we shouldn't be called twice in the same state.
    if (state == STATE::WAITING) { return; }

    //Modify our held argument request with the new callback state.
    fsmAsyncArg.cbState = nextState;

    aWire.send(ADDRESS, data, len,
               [](uint8_t status, void *arg) {

                   //Cast void * arg to the FMSRequestArg struct we passed earlier.
                   FSMAsyncArg *a = static_cast<FSMAsyncArg *>(arg);

                   //Dereference and set the passed status variable.
                   *(a->status) = status;

                   //Dereference and set the passed state variable to the passed new state.
                   *(a->state) = a->cbState;
               }, static_cast<void *>(&fsmAsyncArg));

    state = STATE::WAITING;
}

void HMC5883L::writePointerAsync(REGISTERS reg, STATE nextState) {
    writeAsync((uint8_t *) &reg, 1, nextState);
}

void HMC5883L::rawReadInto(int16_t *x, int16_t *y, int16_t *z) {
    *x = rawData.x;
    *y = rawData.y;
    *z = rawData.z;
    isDataAvailable_TRK = false;
}

void HMC5883L::calibratedReadInto(double *x, double *y, double *z) {
    *x = rawData.x * gainToFactor(configuration.gain) * calibration.xGainFactor + calibration.xOffset;
    *y = rawData.y * gainToFactor(configuration.gain) * calibration.yGainFactor + calibration.yOffset;
    *z = rawData.z * gainToFactor(configuration.gain) * calibration.zGainFactor + calibration.zOffset;
    isDataAvailable_TRK = false;
}

Vector3<int16_t> HMC5883L::rawRead() {
    isDataAvailable_TRK = false;
    return rawData;
}

Vector3<double> HMC5883L::calibratedRead() {
    Vector3<double> data;
    calibratedReadInto(&data.x, &data.y, &data.z);
    isDataAvailable_TRK = false;
    return data;
}


void HMC5883L::printCalibration(Stream &stream, HMC5833LCalibration calibration) {
    stream.printf("xGainFactor:\t%f\n", calibration.xGainFactor);
    stream.printf("yGainFactor:\t%f\n", calibration.yGainFactor);
    stream.printf("zGainFactor:\t%f\n", calibration.zGainFactor);
    stream.printf("xOffset:\t%f\n", calibration.xOffset);
    stream.printf("yOffset:\t%f\n", calibration.yOffset);
    stream.printf("zOffset:\t%f\n", calibration.zOffset);
}


Vector3<double> HMC5883L::preformGainCalibration(Stream &stream, FLOW flow, Vector3<double> targetExcitation) {

    uint8_t buf[16];
    uint16_t ctr = 0;
    Vector3<int16_t> data = {};

    //Enable the passed field self-test with 8 averaged samples.
    HMC5833LConfiguration cfg;
    cfg.flow = flow;
    cfg.gain = GAIN::G_4_7GA;
    cfg.rate = RATE::D_30HZ;
    cfg.samples = SAMPLES::S8;
    cfg.mode = MODE::CONTINUOUS;
    setConfiguration(cfg);

    stream.printf("HMC5883L configured for gain calibration, taking 100 samples...\n");

    //Take 100 readings
    while (ctr < 100) {
        readSync(buf, 6, REGISTERS::OUT_X_M);
        data = bufToRawVec(buf);
        ctr++;
        delay(33);
    }

    stream.printf("Took %u readings to generate gain factors:\n", ctr);
    //Get scaled readings using a default calibration.
    Vector3<double> scaledData = rawVecToCalibratedVec(data, HMC5833LCalibration());
    Vector3<double> correctionFactors = {};
    correctionFactors.x = fabs(targetExcitation.x / scaledData.x);
    correctionFactors.y = fabs(targetExcitation.y / scaledData.y);
    correctionFactors.z = fabs(targetExcitation.z / scaledData.z);


    switch (flow) {
        case FLOW::POSITIVE:
            stream.printf("(+) Scaled X: %f;\tCorrection factor: %f\n", scaledData.x, correctionFactors.x);
            stream.printf("(+) Scaled Y: %f;\tCorrection factor: %f\n", scaledData.y, correctionFactors.y);
            stream.printf("(+) Scaled Z: %f;\tCorrection factor: %f\n", scaledData.z, correctionFactors.z);
            break;
        case FLOW::NEGATIVE:
            stream.printf("(-) Scaled X: %f;\tCorrection factor: %f\n", scaledData.x, correctionFactors.x);
            stream.printf("(-) Scaled Y: %f;\tCorrection factor: %f\n", scaledData.y, correctionFactors.y);
            stream.printf("(-) Scaled Z: %f;\tCorrection factor: %f\n", scaledData.z, correctionFactors.z);
            break;
        default:
            break;
    }

    return correctionFactors;
}


Vector3<double> HMC5883L::preformOffsetCalibration(Stream &stream, HMC5833LCalibration gainCalibration) {
#define waitForUser() while(stream.available() <= 0) {;}
#define flushStream() while(stream.available()) {stream.read();}

    uint8_t buf[16];
    Vector3<double> data = {};

    HMC5833LConfiguration cfg;
    cfg.flow = FLOW::NONE;
    cfg.gain = GAIN::G_1_3GA;
    cfg.rate = RATE::D_30HZ;
    cfg.samples = SAMPLES::S2;
    cfg.mode = MODE::CONTINUOUS;
    setConfiguration(cfg);

    stream.printf("HMC5883L configured for offset calibration.\n");
    stream.printf("Rotate the HMC5883L in all directions until the max. values shown stop changing.\n");
    stream.printf("Then, send any character to finish.\n");
    stream.printf("Send any character to start.\n");

    waitForUser();
    flushStream();

    Vector3<double> mins = {};
    mins.x = infinity();
    mins.y = infinity();
    mins.z = infinity();

    Vector3<double> maxes = {};
    maxes.x = -infinity();
    maxes.y = -infinity();
    maxes.z = -infinity();

    stream.printf("Starting.......\n");
    for (int i = 0; i < 50; i++) {
        readSync(buf, 6, REGISTERS::OUT_X_M);
        delay(33);
    } //Dump first 100 vals.

    while (stream.available() <= 0) {
        readSync(buf, 6, REGISTERS::OUT_X_M);
        data = rawVecToCalibratedVec(bufToRawVec(buf), gainCalibration);

        mins.x = min<double>(mins.x, data.x);
        mins.y = min<double>(mins.y, data.y);
        mins.z = min<double>(mins.z, data.z);

        maxes.x = max<double>(maxes.x, data.x);
        maxes.y = max<double>(maxes.y, data.y);
        maxes.z = max<double>(maxes.z, data.z);

        Serial.printf("xMin: %f,\tyMin: %f,\tzMin: %f,\txMax: %f,\tyMax: %f,\tzMax: %f\n", mins.x, mins.y, mins.z,
                      maxes.x, maxes.y, maxes.z);
        delay(33);
    }
    flushStream();
    Serial.printf("\nFinished! Final Max/Mins:\n");
    Serial.printf("xMin: %f,\tyMin: %f,\tzMin: %f,\txMax: %f,\tyMax: %f,\tzMax: %f\n", mins.x, mins.y, mins.z,
                  maxes.x, maxes.y, maxes.z);

    Vector3<double> offsets = {};
    offsets.x = (maxes.x - mins.x) / 2 - maxes.x;
    offsets.y = (maxes.y - mins.y) / 2 - maxes.y;
    offsets.z = (maxes.z - mins.z) / 2 - maxes.z;

    return offsets;
}


HMC5833LCalibration HMC5883L::preformInteractiveCalibration(Stream &stream) {
    if (!isInitialized) {
        stream.printf("Error in HMC5833L calibration:\n");
        stream.printf("HMC5883L object hasn't started! Did you call <name_of_HMC5833L_object>.begin(Wire); ?\n");
        return HMC5833LCalibration();
    }

    HMC5833LCalibration newCal = {};
    Vector3<double> targetExcitation = {};
    targetExcitation.x = XY_EXCITATION;
    targetExcitation.y = XY_EXCITATION;
    targetExcitation.z = Z_EXCITATION;

    stream.printf("\n--------------Calibrating Positive Gain--------------\n");
    Vector3<double> correctionFactor_p = preformGainCalibration(stream, FLOW::POSITIVE, targetExcitation);
    stream.printf("\n--------------Calibrating Negative Gain--------------\n");
    Vector3<double> correctionFactor_n = preformGainCalibration(stream, FLOW::NEGATIVE, targetExcitation);

    newCal.xGainFactor = (correctionFactor_p.x + correctionFactor_n.x) / 2;
    newCal.yGainFactor = (correctionFactor_p.y + correctionFactor_n.y) / 2;
    newCal.zGainFactor = (correctionFactor_p.z + correctionFactor_n.z) / 2;

    stream.printf("AVG. gain factors:\n");
    stream.printf("X: %f\n", newCal.xGainFactor);
    stream.printf("Y: %f\n", newCal.yGainFactor);
    stream.printf("Z: %f\n", newCal.zGainFactor);

    if (newCal.xGainFactor < 0.8 || newCal.xGainFactor > 1.2 ||
        newCal.yGainFactor < 0.8 || newCal.yGainFactor > 1.2 ||
        newCal.zGainFactor < 0.8 || newCal.zGainFactor > 1.2) {
        stream.printf(
                "Calibration values are out of acceptable range. Please restart the HMC5883L and program by fully cutting and then restoring power.\n");
        HMC5833LCalibration c = {};
        return c;
    }

    stream.printf("\n--------------Calibrating Offsets--------------\n");
    Vector3<double> offsets = preformOffsetCalibration(stream, newCal);
    newCal.xOffset = offsets.x;
    newCal.yOffset = offsets.y;
    newCal.zOffset = offsets.z;

    stream.printf("\n--------------HMC5833L Calibration Results--------------\n");
    stream.printf("(Hint: Use these to create a HMC5833LCalibration struct and pass it to "
                  "your HMC5883L object to enable accurate readings)\n");
    printCalibration(stream, newCal);

    return newCal;
}

Vector3<int16_t> HMC5883L::bufToRawVec(uint8_t *buffer) {
    Vector3<int16_t> vec = {};
    vec.x = ((int16_t) buffer[0]) << 8 | buffer[1];
    vec.z = ((int16_t) buffer[2]) << 8 | buffer[3];
    vec.y = ((int16_t) buffer[4]) << 8 | buffer[5];
    return vec;
}

Vector3<double> HMC5883L::rawVecToCalibratedVec(Vector3<int16_t> rawData, HMC5833LCalibration calibration) {
    Vector3<double> vec = {};
    vec.x = ((double) rawData.x) * gainToFactor(configuration.gain) * calibration.xGainFactor +
            calibration.xOffset;
    vec.y = ((double) rawData.y) * gainToFactor(configuration.gain) * calibration.yGainFactor +
            calibration.yOffset;
    vec.z = ((double) rawData.z) * gainToFactor(configuration.gain) * calibration.zGainFactor +
            calibration.zOffset;
    return vec;
}

uint8_t HMC5883L::readSync(uint8_t *buf, uint8_t len) {
    RequestArg cb(buf, len);
    aWire.request(ADDRESS, NULL, 0, len,
                  [](uint8_t status, void *arg, uint8_t *data, uint8_t datalen) {
                      RequestArg *ra = static_cast<RequestArg *>(arg);
                      memcpy(ra->buffer, data, ra->dataLen);
                      ra->lock = false;
                      ra->status = status;
                  }, static_cast<void *>(&cb));

    while (cb.lock) {
        ESP.wdtFeed();
        aWire.loop();
    }
    return cb.status;
}

uint8_t HMC5883L::readSync(uint8_t *buf, uint8_t len, uint8_t reg) {
    RequestArg cb(buf, len);
    aWire.request(ADDRESS, &reg, 1, len,
                  [](uint8_t status, void *arg, uint8_t *data, uint8_t datalen) {
                      RequestArg *ra = static_cast<RequestArg *>(arg);
                      memcpy(ra->buffer, data, ra->dataLen);
                      ra->lock = false;
                      ra->status = status;
                  }, static_cast<void *>(&cb));

    while (cb.lock) {
        ESP.wdtFeed();
        aWire.loop();
    }
    return cb.status;
}

uint8_t HMC5883L::readSync8() {
    uint8_t result = 0x00;
    readSync(&result, 1);
    return result;
}

uint8_t HMC5883L::readSync8(uint8_t reg) {
    uint8_t result = 0x00;
    readSync(&result, 1, reg);
    return result;
}

void HMC5883L::writeSync(uint8_t *data, uint8_t len, uint8_t reg) {
    uint8_t req[len + 1];
    memcpy(&req[1], data, len);
    req[0] = reg;

    writeSync(req, static_cast<uint8_t>(len + 1));
}

void HMC5883L::writeSync(uint8_t *data, uint8_t len) {
    SendArg sa;
    aWire.send(ADDRESS, data, len,
               [](uint8_t status, void *arg) {
                   SendArg *sap = static_cast<SendArg *>(arg);
                   sap->lock = false;
                   sap->status = status;
               }, static_cast<void *>(&sa));
    while (sa.lock) {
        ESP.wdtFeed();
        aWire.loop();
    }
}

void HMC5883L::setRegisterPointer(uint8_t reg) {
    writeSync8(reg, 1);
}

void HMC5883L::writeSync8(uint8_t data, uint8_t reg) {
    writeSync(&data, 1, reg);
}

void HMC5883L::writeSync8(uint8_t data) {
    writeSync(&data, 1);
}






