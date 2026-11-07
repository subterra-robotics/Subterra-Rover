/*
    This file is part of the Arduino project. It is licensed under the GNU
    General Public License version 3 (GPLv3) or later. You can find the
    full license text at https://www.gnu.org/licenses/gpl-3.0.html.

    Developer: Yunus Hassen

*/

// ------ Inlcudes ------
#include <arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AccelStepper.h>
// #include <tfmini.h>

// ------ Defines ------

#define SERIAL_BAUD 9600
#define RX (uint8_t)16
#define TX (uint8_t)17

#define STEPS_PER_REVOLUTION 2048


// ------ Global Variables ------

Adafruit_MPU6050 mpu;
AccelStepper stepper(AccelStepper::HALF4WIRE, 4, 18, 5, 19); 
// TFMini lidar;

volatile float g_stepperSpeed = 200.0f;   // steps/sec, written by control code

volatile int32_t g_adjustDirection = 0; // 0 = no adjustment, positive = tilt up, negative = tilt down, written by control code

// ------ Function Declarations ------

// void uTaskRunStepper(void *pvParameters);

void uTaskAdjustTilt(void *pvParameters);

// ------ Main Functions ------
void setup()
{
    Serial.begin(SERIAL_BAUD);

    while (!mpu.begin())
    {
        Serial.println("MPU6050 not connected!");
        delay(1000);
    }
    Serial.println("MPU6050 ready!");

    // lidar.begin(Serial, RX, TX, SERIAL_BAUD);
    // while (!lidar.isAlive())
    // {
    //     Serial.println("TFmini not connected!");
    //     delay(1000);
    //     lidar.update();
    // }
    // Serial.println("TFmini ready!");


    // xTaskCreatePinnedToCore(
    //     uTaskRunStepper, 
    //     "StepperTask", 
    //     10000, 
    //     NULL, 1, NULL, 0
    // );

    xTaskCreatePinnedToCore(
        uTaskAdjustTilt, 
        "AdjustTiltTask", 
        10000, 
        NULL, 1, NULL, 0
    );
}

void loop()
{
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();

        sensors_event_t a, g, t;
        mpu.getAccelerometerSensor()->getEvent(&a);
        mpu.getGyroSensor()->getEvent(&g);
        mpu.getTemperatureSensor()->getEvent(&t);

        Serial.printf("[%" PRIu32 "] \r\n", lastPrint);

        Serial.printf("- Temp: %.2f C \r\n", t.temperature);
        Serial.printf("- Accel X: %.2f, Y: %.2f, Z: %.2f m/s^2 \r\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
        
        Serial.printf("- Gyro: X: %.2f, Y: %.2f, Z: %.2f rad/s\r\n", g.gyro.x, g.gyro.y, g.gyro.z);

        Serial.printf("- Adjust Direction: %d \r\n", g_adjustDirection);
    
    
    }
    
}

// ------ Function Definitions ------

// void uTaskRunStepper(void *pvParameters)
// {
//     stepper.setMaxSpeed(800);
//     stepper.setSpeed(g_stepperSpeed);
    
//     float lastSpeed = g_stepperSpeed;

//     for (;;)
//     {
//         float speed = g_stepperSpeed;
//         if (speed != lastSpeed)
//         {
//             stepper.setSpeed(speed);
//             speed = g_stepperSpeed;
//         }

//         stepper.runSpeed();
//         vTaskDelay(1);                       // ~1 ms: yields to the scheduler
//     }
// }

void uTaskAdjustTilt(void *pvParameters)
{
    sensors_event_t g;
    
    for(;;)
    {
        mpu.getGyroSensor()->getEvent(&g);
        if (g.gyro.x > 0.1f)
        {
            g_adjustDirection = 1*(10*abs(g.gyro.x)); // tilt up
        }
        else if (g.gyro.x < -0.1f)
        {
            g_adjustDirection = -1*(10*abs(g.gyro.x)); // tilt down
        }
        else
        {
            g_adjustDirection = 0; // no adjustment
        }


        vTaskDelay(100);                       // ~100 ms: yields to the scheduler
    }
}