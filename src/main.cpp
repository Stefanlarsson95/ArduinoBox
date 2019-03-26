
#include <Arduino.h>
#include <UltraDistSensor.h>
#include "Wire.h"
#include <Servo.h>

UltraDistSensor UDS[9];
Servo ServoRight;
Servo ServoLeft;

#define i2cAddress 0x35 // Slave adress
#define steerGuard 11   // Guardvalue recieved befor steering data.
#define ReadDelay 20    // Ultrasonic sensor ping delay (millisecond)

int LeftServoPin = 10;
int RightServoPin = 11;

int sensors[9] = {6, 4, 8, 5, 7, 2, 12, 3, 9}; // Order of sensors.
byte left = 90;                                // Left angle request
byte right = 90;                               // Right angle request
short table[9];                                // Table of distances

// Actuate steer request
void doSteer()
{
    ServoRight.write(right);
    ServoLeft.write(left);
}

//  On request return table of distances.
void requestData()
{
    uint8_t Buffer[9 * 2];
    for (byte i = 0; i <= 8; i++)
    {
        Buffer[i * 2] = table[i] & 0xff;
        Buffer[i * 2 + 1] = (table[i] >> 8) & 0xff;
    }
    Wire.write(Buffer, 18);
}

//  Interpret recievedData as steer value.
void receiveData(int byteRecieved)
{
    Serial.print("Received bytes: ");
    Serial.println(byteRecieved);

    // Check for first byte steerGuard.
    if (Wire.available() && Wire.read() == steerGuard)
    {
        right = Wire.read();
        left = Wire.read();
        Serial.print("Left:");
        Serial.print(left);
        Serial.print("  Right:");
        Serial.println(right);
    }
    doSteer();
}

// Initialize UltraDistSensor
void initUDS()
{
    for (byte i = 0; i < 9; i++)
    {
        UDS[i].attach(sensors[i]);
    }
    Serial.println("---UltraDistSensor Initialized!---");
}

// Initialize Steering
void initSteering(int rightServo, int leftServo)
{
    pinMode(rightServo, OUTPUT);
    pinMode(leftServo, OUTPUT);
    ServoLeft.attach(rightServo);
    ServoRight.attach(leftServo);
    Serial.println("---Servo steering Initialized!---");
}

// Debug function for Sesor readout
void Print()
{
    for (byte j = 0; j < 9; j++)
    {
        Serial.print("Sensor ");
        Serial.print(j + 1);
        Serial.print(" : ");
        Serial.print(table[j]);
        Serial.println(" cm");
    }
}

// Read sonic sensords
void sonicPing()
{
    for (byte i = 0; i < 9; i++)
    {
        table[i] = constrain(UDS[i].distanceInCm(), -1, 32767); // con
        delay(ReadDelay);
    }
}

void setup()
{
    Serial.begin(115200);

    // Setup I2C communication.
    Wire.begin(i2cAddress);
    Wire.onRequest(requestData);
    Wire.onReceive(receiveData);
    // Initialize Hardware.
    initUDS();
    initSteering(RightServoPin, LeftServoPin);
    // Actuate steer.
    doSteer();
}

void loop()
{
    //Read sensors.
    sonicPing();
    Print();
    //delay(30);
}