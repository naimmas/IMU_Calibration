#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "../LSM303D/LSM303.h"
#include "../L3GD20/L3GD20.h"

typedef struct
{
  float Acc[3];   /* Acceleration in X, Y, Z axis in [g] */
  float Gyro[3];  /* Gyro data in X, Y, Z axis in [rads] */
  //int TimeStamp;  /* Time stamp for accelerometer sensor output in [ms] */
} MAC_input_t;

typedef struct
{
  float AccOffset[3];              /* Accelerometer offset value in [g] */
  float AccScale[3];               /* Scale factor each axes*/
  float GyroBias[3];               /* Bias factor each axes*/
  float AccCal[3];                 /*Calibrated Acc Values*/
  float GyroCal[3];               /*Calibrared gyro values*/
} MAC_output_t;


typedef enum
{
  MAC_CALERORR = 0,
  //MAC_CALQSTATUSPOOR,
  MAC_CALQSTATUSOK,
  //MAC_CALQSTATUSGOOD
} MAC_cal_quality_t;

MAC_cal_quality_t AccCalBegin(MAC_output_t *dataOut, LSM303D *dev);

MAC_cal_quality_t GyroCalBegin(MAC_output_t *dataOut, L3GD20 *dev);
#ifdef C_SIMULATION
typedef  enum {
    GYRO, ACCELEROMETER, MAGNEOMETER
}IMU_Typedef;
typedef struct {
    LSM303D *ACMA_S;
    L3GD20  *GY_S;
}IMU_SENOR_Typedef;
void StringToFloat(char str[], IMU_SENOR_Typedef *device, IMU_Typedef imu)
{
    char *pt;
    pt = strtok (str,",");
    float x = atof(pt);
    pt = strtok (NULL, ",");
    float y = atof(pt);
    pt = strtok (NULL, ",");
    float z = atof(pt);
    pt = strtok (NULL, ",");

    switch (imu) {
        case GYRO:
            device->GY_S->gyro[0] = x;
            device->GY_S->gyro[1] = y;
            device->GY_S->gyro[2] = z;
            break;
        case ACCELEROMETER:
            device->ACMA_S->acc[0] = x;
            device->ACMA_S->acc[1] = y;
            device->ACMA_S->acc[2] = z;
            break;
        case MAGNEOMETER:
            device->ACMA_S->mag[0] = x;
            device->ACMA_S->mag[1] = y;
            device->ACMA_S->mag[2] = z;
            break;

    }




}

int GetIMUData(IMU_Typedef imu, IMU_SENOR_Typedef *device, int read_line)
{
    // file pointer will be used to open/read the file
    FILE *file;
    char filename[1024];
    // used to store the filename and each line from the file
    switch (imu) {
        case GYRO:
            sprintf(filename,"../Sensor_Calibration/GyroTest.txt");
            break;
        case ACCELEROMETER:
            sprintf(filename,"../Sensor_Calibration/AccTest.txt");
            break;
        case MAGNEOMETER:
            sprintf(filename,"../Sensor_Calibration/MagTest.txt");
            break;
    }


    char buffer[2048];

    // stores the line number of the line the user wants to read from the file
    read_line++;
    // prompt the user for the line to read, store it into read_line


    // open the the file in read mode
    file = fopen(filename, "r");

    // if the file failed to open, exit with an error message and status
    if (file == NULL)
    {
        printf("Error opening file.\n");
        return 1;
    }

    // we'll keep reading the file so long as keep_reading is true, and we'll
    // keep track of the current line of the file using current_line
    bool keep_reading = true;
    int current_line = 1;
    do
    {
        // read the next line from the file, store it into buffer
        fgets(buffer, 2048, file);

        // if we've reached the end of the file, we didn't find the line
        if (feof(file))
        {
            // stop reading from the file, and tell the user the number of lines in
            // the file as well as the line number they were trying to read as the
            // file is not large enough
            keep_reading = false;
            printf("File %d lines.\n", current_line-1);
            printf("Couldn't find line %d.\n", read_line);
            return 1;
        }
            // if we've found the line the user is looking for, print it out
        else if (current_line == read_line)
        {
            keep_reading = false;
            StringToFloat(buffer, device, imu);
        }

        // continue to keep track of the current line we are reading
        current_line++;

    } while (keep_reading);

    // close our access to the file
    fclose(file);

    // notably at this point in the code, buffer will contain the line of the
    // file we were looking for if it was found successfully, so it could be
    // used for other purposes at this point as well...
    return 0;
}
#endif
#ifdef __cplusplus
}
#endif

#endif