
/*
TODO
Swap sensor at pin 10 to pin 12
connect servos to pin 10, 11
*/
#include <Arduino.h>
#include <UltraDistSensor.h>
#include "Wire.h"
#include <Servo.h>
/*
UltraDistSensor FCU;
UltraDistSensor FLU;
UltraDistSensor FRU;
UltraDistSensor LFU;
UltraDistSensor LBU;
UltraDistSensor RFU;
UltraDistSensor RBU;
UltraDistSensor BRU;
UltraDistSensor BLU;
*/

UltraDistSensor UDS[9];

Servo ServoRight;
Servo ServoLeft;

#define i2cAddress 0x35 // slave adress
#define wheelbase 54    // wheelbase dim for calculating Ackermann geometry.
#define steerGuard 11   // Guardvalue recieved befor steering data

int LeftServoPin = 10;
int RightServoPin = 11;
int ReadDelay = 30; // Ultrasonic sensor ping delay
byte left = 90;      // Left angle request
byte right = 90;     // Right angle request

short table[] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; //0->8

void requestData()
{
    uint8_t Buffer[9 * 2];
    for (int i = 0; i <= 8; i += 2)
    {
        Buffer[i] = table[i] & 0xff;
        Buffer[i + 1] = (table[i] >> 8) & 0xff;
    }
    Wire.write(Buffer, 18);
}

void receiveData(int byteRecieved)
{
    Serial.print("Received bytes: ");
    Serial.println(byteRecieved);

    if (Wire.available() && Wire.read() == steerGuard) // Check for first byte steerGuard.
    {
        right = Wire.read();
        left = Wire.read();
        Serial.print("Left:");
        Serial.print(left);
        Serial.print("  Right:");
        Serial.println(right);
    }
}

void initUDS() // Initialize UltraDistSensor
{
    int sensors[9] = {6, 4, 8, 5, 7, 2, 12, 3, 9};
    for (int i; i < 9; i++)
    {
        UDS[i].attach(sensors[i]);
    }
    /*
    FCU.attach(6);  //0
    BRU.attach(4);  //1
    BLU.attach(8);  //2
    FRU.attach(5);  //3
    FLU.attach(7);  //4
    RFU.attach(2);  //5
    LFU.attach(12); //6 // OSB previous connected to pin 10!
    RBU.attach(3);  //7
    LBU.attach(9);  //8
    */
    Serial.println("---UltraDistSensor Initialized!---");
}

void initSteering(int rightServo, int leftServo)
{
    pinMode(rightServo, OUTPUT);
    pinMode(leftServo, OUTPUT);
    ServoLeft.attach(rightServo);
    ServoRight.attach(leftServo);
    Serial.println("---Servo steering Initialized!---");
}

void Print()
{
    Serial.println("-------------------------------------------------------");
    Serial.println("-------------------------------------------------------");
    Serial.println("-------------------------------------------------------");
    for (int i; i < 9; i++)
    {
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print("reading : ");
        Serial.print(table[i]);
        Serial.println(" cm");
    }
}

void sonicPing()
{
    for (int i; i < 9; i++){
        table[i]=UDS[i].distanceInCm();
        delay(ReadDelay);
    }
/*
    table[0] = FCU.distanceInCm();
    delay(ReadDelay);

    table[1] = BRU.distanceInCm();
    delay(ReadDelay);
    table[2] = BLU.distanceInCm();
    delay(ReadDelay);

    table[3] = FRU.distanceInCm();
    delay(ReadDelay);
    table[4] = FLU.distanceInCm();
    delay(ReadDelay);

    table[5] = RFU.distanceInCm();
    delay(ReadDelay);
    table[6] = LFU.distanceInCm();
    delay(ReadDelay);

    table[7] = RBU.distanceInCm();
    delay(ReadDelay);
    table[8] = LBU.distanceInCm();
    delay(ReadDelay);
*/
    //Print();
    //delay(1000);
}

void steeringSweep(Servo servo)
{
    int k = 1;
    if (servo.read() < 90)
    {
        while (servo.read() < 180)
        {
            servo.write((servo.read()) + k);
            delay(10);
            Serial.print("Angle: ");
            Serial.println(servo.read());
        }
    }
    else
    {
        while (servo.read() > 0)
        {
            servo.write((servo.read()) - k);
            delay(10);
            Serial.print("Angle: ");
            Serial.println(servo.read());
        }
    }
}

void doSteer(Servo servoR, Servo servoL)
{
    servoR.write(right);
    servoL.write(left);
    Serial.print(right);
    Serial.println(left);
}

void setup()
{
    //Serial.begin(9600);

    Wire.begin(i2cAddress);
    Wire.onRequest(requestData);
    Wire.onReceive(receiveData);

    initUDS(); // Ultrasonic sensor Initialized

    initSteering(RightServoPin, LeftServoPin);
}

void loop()
{
    sonicPing();
    doSteer(ServoRight, ServoLeft);

    //steeringSweep(ServoRight);
    //steeringSweep(ServoLeft);
    delay(10);
}