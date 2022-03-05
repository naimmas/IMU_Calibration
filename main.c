#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include "Calibration.c"

#define FILENAME_SIZE 1024
#define MAX_LINE 2048

typedef enum
{
    X,
    Y,
    Z
} Axes_typedef;

float ax_max = 0;
float ax_min = 0;
float ay_max = 0;
float ay_min = 0;
float az_max = 0;
float az_min = 0;


int main(void)
{
    LSM303D dev;
    L3GD20 dev2;
    MAC_output_t accCal;
    MAC_output_t CalGyro;
    AccCalBegin(&accCal, &dev);
    printf("Acc OFFSET     ->%5.3f, %5.3f, %5.3f\n", accCal.AccOffset[0], accCal.AccOffset[1], accCal.AccOffset[2]);
    printf("Acc SCALE      ->%5.3f, %5.3f, %5.3f\n", accCal.AccScale[0], accCal.AccScale[1], accCal.AccScale[2]);
    printf("Acc CALIBRATED ->%5.3f, %5.3f, %5.3f\n\n", accCal.AccCal[0], accCal.AccCal[1], accCal.AccCal[2]);
    GyroCalBegin(&CalGyro,  &dev2);
    printf("Gyro OFFSET     ->%5.3f, %5.3f, %5.3f\n", CalGyro.GyroBias[0], CalGyro.GyroBias[1], CalGyro.GyroBias[2]);
    printf("Gyro CALIBRATED ->%5.3f, %5.3f, %5.3f\n", CalGyro.GyroCal[0], CalGyro.GyroCal[1], CalGyro.GyroCal[2]);


}
