#pragma once
#include "Wire.h"
enum ICM_20948_Status_e { ICM_20948_Stat_Ok, ICM_20948_Stat_FIFONoDataAvail, ICM_20948_Stat_FIFOMoreDataAvail };
enum { INV_ICM20948_SENSOR_ROTATION_VECTOR, INV_ICM20948_SENSOR_ACCELEROMETER };
enum { DMP_ODR_Reg_Quat9, DMP_ODR_Reg_Accel };
#define DMP_header_bitmap_Quat9 1
#define DMP_header_bitmap_Accel 2
struct icm_20948_DMP_data_t { int header; struct { struct { int32_t Q1,Q2,Q3; } Data; } Quat9; struct { struct { int16_t X,Y,Z; } Data; } Raw_Accel; };
struct ICM_20948_I2C { ICM_20948_Status_e status; void begin(TwoWire&,int){} ICM_20948_Status_e initializeDMP(){return status;} ICM_20948_Status_e enableDMPSensor(int){return status;} ICM_20948_Status_e setDMPODRrate(int,int){return status;} ICM_20948_Status_e enableFIFO(){return status;} ICM_20948_Status_e enableDMP(){return status;} ICM_20948_Status_e resetDMP(){return status;} ICM_20948_Status_e resetFIFO(){return status;} ICM_20948_Status_e readDMPdataFromFIFO(icm_20948_DMP_data_t*){return status;}
 ICM_20948_Status_e setBiasGyroX(int){return status;} ICM_20948_Status_e setBiasGyroY(int){return status;} ICM_20948_Status_e setBiasGyroZ(int){return status;} ICM_20948_Status_e setBiasAccelX(int){return status;} ICM_20948_Status_e setBiasAccelY(int){return status;} ICM_20948_Status_e setBiasAccelZ(int){return status;} ICM_20948_Status_e setBiasCPassX(int){return status;} ICM_20948_Status_e setBiasCPassY(int){return status;} ICM_20948_Status_e setBiasCPassZ(int){return status;} };
