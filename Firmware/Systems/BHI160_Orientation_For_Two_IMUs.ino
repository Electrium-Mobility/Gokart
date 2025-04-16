// Wire library to set up I2C communications
#include <Wire.h>

// Bosch header files installed locally as an arduino library
#include "bhy.h"
#include "firmware/Bosch_PCB_7183_di03_BMI160_BMM150-7183_di03.2.1.11696_170103.h"

/*
First sensor = reference IMU
Second sensor = steering wheel IMU
*/

#define BHY_INT_PIN_1 10 // Pin for the first sensor interrupt (reference IMU)
#define BHY_INT_PIN_2 11 // Pin for the second sensor interrupt (steering wheel IMU)

/* Sensor instances */
BHYSensor bhi160_1;
BHYSensor bhi160_2;

/* Interrupt flags */
volatile bool intrToggled_1 = false;
volatile bool intrToggled_2 = false;

/* New data checking flags */
bool newOrientationData_1 = false;
bool newOrientationData_2 = false;

/* Orientation data variables*/
float yaw_1, roll_1, pitch_1;
float yaw_2, roll_2, pitch_2;

/* Sensor statuses */
uint8_t status_1, status_2;

/* Define functions used in this code */
bool checkSensorStatus(BHYSensor& sensor);
float calculateSteeringWheelRotation(float referenceYaw, float currentYaw);

/* Sets interrupt flags to true */
void bhyInterruptHandler1(void) {
    intrToggled_1 = true;
}
void bhyInterruptHandler2(void) {
    intrToggled_2 = true;
}

/* Waits for interrupt flags to be true */
void waitForBhyInterrupt1(void) {
    while (!intrToggled_1);
    intrToggled_1 = false;
}
void waitForBhyInterrupt2(void) {
    while (!intrToggled_2);
    intrToggled_2 = false;
}

/* Collects new orientation data */
void orientationHandler1(bhyVector data, bhyVirtualSensor type) {
    yaw_1 = data.x;
    roll_1 = data.z;
    pitch_1 = data.y;
    status_1 = data.status;
    newOrientationData_1 = true;
}
void orientationHandler2(bhyVector data, bhyVirtualSensor type) {
    yaw_2 = data.x;
    roll_2 = data.z;
    pitch_2 = data.y;
    status_2 = data.status;
    newOrientationData_2 = true;
}

/* Setup */
void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (Serial) {
        Serial.println("Serial working");
    }

    /* Configure the first BHI160 (steering wheel IMU) */
    attachInterrupt(BHY_INT_PIN_1, bhyInterruptHandler1, RISING);

    bhi160_1.begin(BHY_I2C_ADDR);

    /* Check to see if something went wrong. */
    if (!checkSensorStatus(bhi160_1))
        return;

    Serial.println("First Sensor found over I2C! Product ID: 0x" + String(bhi160_1.productId, HEX));

    Serial.println("Uploading Firmware for the First Sensor.");
    bhi160_1.loadFirmware(bhy1_fw);

    if (!checkSensorStatus(bhi160_1))
        return;

    intrToggled_1 = false; /* Clear interrupt status received during firmware upload */
    waitForBhyInterrupt1(); /* Wait for meta events from boot up */
    Serial.println("First Firmware Booted");

    /* Install a metaevent callback handler and a timestamp callback handler here if required before the first run */
    bhi160_1.run(); /* The first run processes all boot events */

    /* Install a vector callback function to process the data received from the wake up Orientation sensor */
    if (bhi160_1.installSensorCallback(BHY_VS_ORIENTATION, true, orientationHandler1)) {
        checkSensorStatus(bhi160_1));
        return;
    }
    else
        Serial.println("Orientation callback for the first sensor installed");

    /* Enable the Orientation virtual sensor that gives you the heading (yaw), roll, pitch
       based of data from the accelerometer, gyroscope and magnetometer.
       The sensor is set into wake up mode so as to interrupt the host when a new sample is available
       Additionally, the FIFO buffer of the sensor is flushed for all previous data
       The maximum report latency of the sensor sample, the sensitivity and the dynamic range
       are set to 0
     */

    // Enable the orientation virtual sensor for the first sensor
    if (bhi160_1.configVirtualSensor(BHY_VS_ORIENTATION, true, BHY_FLUSH_ALL, 200, 0, 0, 0)) {
        Serial.println("Failed to enable virtual sensor for the first sensor or (" + bhi160_1.getSensorName(BHY_VS_ORIENTATION) + "). Loaded firmware may not support requested sensor id.");
    }
    else
        Serial.println("First sensor or (" + bhi160_1.getSensorName(BHY_VS_ORIENTATION) + ") virtual sensor enabled");



    /* Configure the second BHI160 (steering wheel IMU) */
    attachInterrupt(BHY_INT_PIN_2, bhyInterruptHandler2, RISING);

    bhi160_2.begin(BHY_I2C_ADDR2);

    /* Check to see if something went wrong. */
    if (!checkSensorStatus(bhi160_2))
        return;

    Serial.println("Second Sensor found over I2C! Product ID: 0x" + String(bhi160_2.productId, HEX));

    Serial.println("Uploading Firmware for the Second Sensor.");
    bhi160_2.loadFirmware(bhy1_fw);

    if (!checkSensorStatus(bhi160_2))
        return;

    intrToggled_2 = false; /* Clear interrupt status received during firmware upload */
    waitForBhyInterrupt2(); /* Wait for meta events from boot up */
    Serial.println("Second Firmware Booted");

    /* Install a metaevent callback handler and a timestamp callback handler here if required before the first run */
    bhi160_2.run(); /* The first run processes all boot events */

    /* Install a vector callback function to process the data received from the wake up Orientation sensor */
    if (bhi160_2.installSensorCallback(BHY_VS_ORIENTATION, true, orientationHandler2)) {
        checkSensorStatus(bhi160_2));
        return;
    }
    else
        Serial.println("Orientation callback for the first sensor installed");

    /* Enable the Orientation virtual sensor that gives you the heading (yaw), roll, pitch
       based of data from the accelerometer, gyroscope and magnetometer.
       The sensor is set into wake up mode so as to interrupt the host when a new sample is available
       Additionally, the FIFO buffer of the sensor is flushed for all previous data
       The maximum report latency of the sensor sample, the sensitivity and the dynamic range
       are set to 0
     */

    // Enable the orientation virtual sensor for the second sensor
    if (bhi160_2.configVirtualSensor(BHY_VS_ORIENTATION, true, BHY_FLUSH_ALL, 200, 0, 0, 0)) {
        Serial.println("Failed to enable virtual sensor for the second sensor or (" + bhi160_2.getSensorName(BHY_VS_ORIENTATION) + "). Loaded firmware may not support requested sensor id.");
    }
    else
        Serial.println("Second sensor or (" + bhi160_2.getSensorName(BHY_VS_ORIENTATION) + ") virtual sensor enabled");
}

void loop() {
    // Process first sensor data
    if (intrToggled_1) {
        intrToggled_1 = false;
        bhi160_1.run();
        checkSensorStatus(bhi160_1);
        if (newOrientationData_1) {
            Serial.println("Reference IMU Data: " + String(yaw_1) + "," + String(pitch_1) + "," + String(roll_1));
            newOrientationData_1 = false;
        }
    }

    // Process second sensor data
    if (intrToggled_2) {
        intrToggled_2 = false;
        bhi160_2.run();
        checkSensorStatus(bhi160_2);
        if (newOrientationData_2) {
            Serial.println("Steering Wheel IMU Data: " + String(yaw_2) + "," + String(pitch_2) + "," + String(roll_2));
            newOrientationData_2 = false;
            
            /* Calculate the steering wheel rotation based on the yaw values */
            float steeringAngle = calculateSteeringWheelRotation(yaw_1, yaw_2);
            Serial.println("Steering Wheel Angle: " + String(steeringAngle) + " degrees");
        }
    }
}

/* Function to calculate the angle of rotation of the steering wheel */
float calculateSteeringWheelRotation(float referenceYaw, float currentYaw) {
    float angle = currentYaw - referenceYaw; // Calculate the angle of rotation

    if (angle < 0) { // Normalize the angle to be in the range of 0-360 degrees
        angle += 360;
    }

    return angle;
}

bool checkSensorStatus(BHYSensor& sensor) {
    if (sensor.status == BHY_OK)
        return true;

    if (sensor.status < BHY_OK) { /* All error codes are negative */
        Serial.println("Error code: (" + String(sensor.status) + "). " + sensor.getErrorString(sensor.status));
        return false; /* Something has gone wrong */
    }
    else { /* All warning codes are positive */
        Serial.println("Warning code: (" + String(sensor.status) + ").");
        return true;
    }

    return true;
}