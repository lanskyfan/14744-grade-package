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
const char ACCPath[]="/Meas/Acc/52"; // path to accelerometer, using 52 frequency
const uint8_t DEFAULT_REFERENCE=99; //appears as 63 in hex
const uint8_t ACC_REF=10; // reference to accelerometer subscription
const int8_t UP_THRESHOLD=-3; // acceleration threshold for swinging up the hand
const int8_t DOWN_THRESHOLD=-17; // acceleration threshold for putting down the hand
const int8_t UP = 0; // swinging up direction
const int8_t DOWN = 1; // swinging down direction
const uint8_t ACC_TAG = 5; // tag for accelerometer data

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
            subscribe(ACCPath, sizeof(ACCPath), ACC_REF);
        }
        break;
        case Commands::END_SUB:
        {
            //unsubscribes
            unsubscribe(ACC_REF);
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
    // only process data from accelerometer subscription
    if(findDataSub(resourceId)->clientReference != ACC_REF)
        return;
    const WB_RES::AccData &data = value.convertTo<WB_RES::AccData&>();
    const wb::Array<wb::FloatVector3D> &accData = data.arrayAcc;
    // average acceleration on x-axis
    float averageAcc=0;
    for (size_t i=0; i<accData.size();i++) {
        wb::FloatVector3D a = accData[i];
        // only take acceleration on x-axis
        float acc = a.x;
        averageAcc += acc;
    }
    averageAcc /= accData.size();

    // each step should satisfy the direction criteria and acceleration criteria
    if ((averageAcc > UP_THRESHOLD && direction == UP) ||
        (averageAcc < DOWN_THRESHOLD && direction == DOWN)) {
        steps+=1;
        // change direction to avoid recording steps multiple times
        direction = 1 - direction;
        int nDigits = floor(log10(steps)) + 1;
        char str[12];
        
        sprintf(str, "%d", steps);
        uint8_t output[12];
        for (int i = 0; i < 12; i++) {
            output[i] = uint8_t(str[i]);
        }
        // send data using the ACC_TAG tag
        sendPacket(output, nDigits, ACC_TAG, Responses::COMMAND_RESULT);
        // set led to blink 1 time
        ledSetPattern(250, 250, 1);
    }
    
}
