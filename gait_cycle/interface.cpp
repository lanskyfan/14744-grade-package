#include "myApp.h"
#include "interface.h"

// sensor libraries for data interperting
#include "meas_acc/resources.h"
#include "meas_gyro/resources.h"
#include "meas_magn/resources.h"
#include "meas_imu/resources.h"
#include <ui_ind/resources.h>

// This code is modified by Yifan Lan (Andrew ID: yifanlan)

//Commands formated as byte array with [Command, data?...] (data optional)
enum Commands 
{
    HELLO = 0,
    BEGIN_SUB=1,
    END_SUB=2,
    BLINK=3,
};
//Responses formated as byte array with [Response, tag, data?...] (data optional)
enum Responses 
{
    COMMAND_RESULT = 1,
    DATA = 2,
    ERROR = 3,
};
          
const char IMUPath[]="/Meas/IMU6/104";
const char GYROPath[]="/Meas/Gyro/52"; // path to gyroscope data, using 52 frequency
const uint8_t DEFAULT_REFERENCE=99; //appears as 63 in hex
const uint8_t GYRO_REF=20;
const int8_t UP_THRESHOLD=100; // gyroscope threshold for swing phase
const int8_t DOWN_THRESHOLD=-50; // gyroscope threshold for stance phase
const int8_t UP = 0; // swing phase detection
const int8_t DOWN = 1; // stance phase detection
const uint8_t GYRO_TAG = 5; // tag for gyroscope data

uint32_t steps=0; // The total number of steps
uint8_t direction=UP; // Direction of hands, 0 for swings up, 1 for putting down

void myApp::handleCommand(uint8_t cmd, const uint8_t values[], size_t len){
    switch (cmd)
    {
        case Commands::HELLO:
        {
            // Hello response, for use as a sanity check <3
            uint8_t helloMsg[] = {'H','e','l','l','o'};
            //tags aren't explicitly necessary, but they're a good way of grouping responses.
            //The included console uses them to filter and format responses.
            uint8_t tag=1;
            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
        }
        break;
        case Commands::BEGIN_SUB:
        {
            //unsubscribes to prevent duplicate subscriptions
            unsubscribe(DEFAULT_REFERENCE);
            // reset the number of steps
            steps = 0;
            //subscribes to the accelerometer data
            subscribe(GYROPath, sizeof(GYROPath), GYRO_REF);
        }
        break;
        case Commands::END_SUB:
        {
            //unsubscribes
            unsubscribe(GYRO_REF);
        }
        break;
        case Commands::BLINK:
        {
            //  blinking the Movesense device's LED light 
            // 3 times with 2 second pauses in between in each blink.
            ledSetPattern(1000, 2000, 3);
        }
        break;
    }
}

void myApp::processData(wb::ResourceId resourceId, const wb::Value &value){
    // only process data from gyroscope subscription
    if(findDataSub(resourceId)->clientReference != GYRO_REF)
        return;
    const WB_RES::GyroData &data = value.convertTo<WB_RES::GyroData&>();
    const wb::Array<wb::FloatVector3D> &gyroData = data.arrayGyro;
    // average gyroscope data on z-axis
    float averageGyro=0;
    for (size_t i=0; i<gyroData.size();i++) {
        wb::FloatVector3D a = gyroData[i];
        // only take acceleration on x-axis
        float gyro = a.z;
        averageGyro += gyro;
    }
    averageGyro /= gyroData.size();

    // each step should satisfy the direction criteria and threshold criteria
    char message[20];
    int len = 0;
    bool incr = false; // indicator to increment the step count

    // case where swing phase is detected
    if (averageGyro > UP_THRESHOLD && direction == UP) {
        sprintf(message, "Swing ");
        len = 6;
        // sendPacket(swingMessage, sizeof(swingMessage), GYRO_TAG, Responses::COMMAND_RESULT);
        incr = true;
    // case where stance phase is detected
    } else if (averageGyro < DOWN_THRESHOLD && direction == DOWN) {
        sprintf(message, "Stance ");
        len = 7;
        incr = true;
    }

    if (incr) {
        steps+=1;
        // change direction to avoid recording steps multiple times
        direction = 1 - direction;
        int nDigits = floor(log10(steps)) + 1;
        char stepStr[12];
        
        // copy the message to output
        sprintf(stepStr, "%d", steps);
        uint8_t output[20];
        for (int i = 0; i < len; i++) {
            output[i] = uint8_t(message[i]);
        }
        for (int i = len; i < 12 + len; i++) {
            output[i] = uint8_t(stepStr[i - len]);
        }
        // send data using the GYRO_TAG tag
        sendPacket(output, len + nDigits, GYRO_TAG, Responses::COMMAND_RESULT);
    }

}
