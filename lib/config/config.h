#ifndef CONFIG_H
#define CONFIG_H

#include "step.h" // Include the Step struct definition

// Define the maximum number of steps allowed
#define MAX_STEPS_ALLOWED 20

// OpenRouteService API Key
extern const char *apiKey; // Declare as extern
extern const char *host;   // Declare as extern

extern Step steps[MAX_STEPS_ALLOWED]; // Declare as extern

// MPU6050 I2C pins
#define MPU6050_SDA 21
#define MPU6050_SCL 22
#define dw digitalWrite
#define dr digitalRead

#define MT1_L 25
#define MT1_R 26
#define MT2_L 14
#define MT2_R 27

#define ENC_L_A 32
#define ENC_L_B 33
#define ENC_R_A 18
#define ENC_R_B 5

#define PWM_CHANNEL0 0
#define PWM_CHANNEL1 1
#define PWM_CHANNEL2 2
#define PWM_CHANNEL3 3

#define GPS_RX 16
#define GPS_TX 17
#endif