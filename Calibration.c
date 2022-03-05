#include "Calibration.h"
#ifndef C_SIMULATION
#include "stm32h743xx.h"

#else
#define LINE 1199
#endif
MAC_cal_quality_t AccCalBegin(MAC_output_t *dataOut, LSM303D *dev)
{
    float ax_max = 0;
    float ax_min = 0;
    float ay_max = 0;
    float ay_min = 0;
    float az_max = 0;
    float az_min = 0;
    #ifndef C_SIMULATION

    int ReadingInterval = 50;   /*ms*/
    int ReadingDuration = 5000; /*ms*/
    int beginTime = HAL_GetTick();
    while (HAL_GetTick() - beginTime < ReadingDuration)
    #else
    IMU_SENOR_Typedef sens;
    sens.ACMA_S = dev;
    int i = 0;
    while(i<LINE)
    #endif
    {   
        #ifndef C_SIMULATION
        if (LSM303D_ReadAcc(dev) != LSM303_SENSOR_CONNECTION_OK)
        {
            return MAC_CALERORR;
        }
        #else
        GetIMUData(ACCELEROMETER, &sens,i);
        #endif

        if (dev->acc[0] > ax_max)
        {
            ax_max = dev->acc[0];
        }
        else if (dev->acc[0] < ax_min)
        {
            ax_min = dev->acc[0];
        }

        if (dev->acc[1] > ay_max)
        {
            ay_max = dev->acc[1];
        }
        else if (dev->acc[1] < ay_min)
        {
            ay_min = dev->acc[1];
        }

        if (dev->acc[2] > az_max)
        {
            az_max = dev->acc[2];
        }
        else if (dev->acc[2] < az_min)
        {
            az_min = dev->acc[2];
        }
        #ifndef C_SIMULATION        //No need to delay on reading data from file
        HAL_Delay(ReadingInterval);
        #else
        i++;
        #endif
    }
    dataOut->AccOffset[0] = ((ax_max + ax_min) / 2);
    dataOut->AccOffset[1] = ((ay_max + ay_min) / 2);
    dataOut->AccOffset[2] = ((az_max + az_min) / 2);

    dataOut->AccScale[0] = GRAVITY / (ax_max - dataOut->AccOffset[0]);
    dataOut->AccScale[1] = GRAVITY / (ay_max - dataOut->AccOffset[1]);
    dataOut->AccScale[2] = GRAVITY / (az_max - dataOut->AccOffset[2]);


    return MAC_CALQSTATUSOK;
}

MAC_cal_quality_t GyroCalBegin(MAC_output_t *dataOut, L3GD20 *dev)
{

    float gxBias = 0;
    float gyBias = 0;
    float gzBias = 0;
    int sum = 0;
    int ReadingInterval = 5;    /*ms*/
    int ReadingDuration = 5000; /*ms*/

#ifndef C_SIMULATION
    int beginTime = HAL_GetTick();
    while (HAL_GetTick() - beginTime < ReadingDuration)
#else
    IMU_SENOR_Typedef sens;
    sens.GY_S = dev;
    int i=0;
    while(i<LINE)
#endif
    {
        #ifndef C_SIMULATION
        if (L3GD20_ReadGyro(dev) != L3GD20_SENSOR_CONNECTION_OK)
        {
            return MAC_CALERORR;
        }
        #else
        if(GetIMUData(GYRO, &sens, i))
            return;
#endif
        gxBias += dev->gyro[0];
        gyBias += dev->gyro[1];
        gzBias += dev->gyro[2];
        sum++;
#ifndef C_SIMULATION        //No need to delay on reading data from file
        HAL_Delay(ReadingInterval);
#else
        i++;
#endif
    }
    dataOut->GyroBias[0] = gxBias / (float)sum;
    dataOut->GyroBias[1] = gyBias / (float)sum;
    dataOut->GyroBias[2] = gzBias / (float)sum;
    return MAC_CALQSTATUSOK;
}
void GetGyro(MAC_output_t *dataOut, L3GD20 *dev)
{
    L3GD20_ReadGyro(dev);
    dataOut->GyroCal[0] = dev->gyro[0] - dataOut->GyroBias[0];
    dataOut->GyroCal[1] = dev->gyro[1] - dataOut->GyroBias[1];
    dataOut->GyroCal[2] = dev->gyro[2] - dataOut->GyroBias[2];
}
void GetAcc(MAC_output_t *dataOut, LSM303D *dev)
{
    LSM303D_ReadAcc(dev);
    dataOut->AccCal[0] = (dev->acc[0] - dataOut->AccOffset[0]) * dataOut->AccScale[0];
    dataOut->AccCal[1] = (dev->acc[1] - dataOut->AccOffset[1]) * dataOut->AccScale[1];
    dataOut->AccCal[2] = (dev->acc[2] - dataOut->AccOffset[2]) * dataOut->AccScale[2];
}