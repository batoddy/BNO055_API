/*
 * BNO055_API.c
 *
 *  Created on: May 26, 2023
 *      Author: Batuhan
 */
#include "BNO055_API.h"
#include "main.h"
uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1<<14);    // 2^14

extern I2C_HandleTypeDef hi2c1;
#define BNO055_I2C &hi2c1

void bno055_delay(int time) {
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}
void bno055_writeData(uint8_t reg, uint8_t data) {
  uint8_t txdata[2] = {reg, data};
  uint8_t status;
  status = HAL_I2C_Master_Transmit(BNO055_I2C, BNO055_I2C_ADDR << 1,
                                   txdata, sizeof(txdata), 10);
  if (status == HAL_OK) {
    return;
  }
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
  HAL_I2C_Master_Transmit(BNO055_I2C, BNO055_I2C_ADDR << 1, &reg, 1,
                          100);
  HAL_I2C_Master_Receive(BNO055_I2C, BNO055_I2C_ADDR << 1, data, len,
                         100);

}

void bno055_setPage(uint8_t page) {
	bno055_writeData(BNO055_PAGE_ID, page);
}

void bno055_setOperationMode(bno055_opmode_t mode) {
  bno055_writeData(BNO055_OPR_MODE, mode);
  if (mode == BNO055_OPERATION_MODE_CONFIG) {
    bno055_delay(19);
  } else {
    bno055_delay(7);
  }
}

void bno055_setOperationModeConfig() {
  bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF() {
  bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

void bno055_reset() {
  bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
  bno055_delay(700);
}

int8_t bno055_getTemp() {
  bno055_setPage(0);
  uint8_t t;
  bno055_readData(BNO055_TEMP, &t, 1);
  return t;
}

void bno055_setup() {
  bno055_reset();

  uint8_t id = 0;
  bno055_readData(BNO055_CHIP_ID, &id, 1);
  if (id != BNO055_ID) {
    printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
  }
  bno055_setPage(0);
  bno055_writeData(BNO055_SYS_TRIGGER, 0x0);

  // Select BNO055 config mode
  bno055_setOperationModeConfig();
  bno055_delay(10);
}

bno055_vector_t bno055_getVector(uint8_t vec) {
  bno055_setPage(0);
  uint8_t buffer[8];    // Quaternion need 8 bytes

  if (vec == BNO055_VECTOR_QUATERNION)
    bno055_readData(vec, buffer, 8);
  else
    bno055_readData(vec, buffer, 6);

  double scale = 1;

  if (vec == BNO055_VECTOR_MAGNETOMETER) {
    scale = magScale;
  } else if (vec == BNO055_VECTOR_ACCELEROMETER ||
           vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
    scale = accelScale;
  } else if (vec == BNO055_VECTOR_GYROSCOPE) {
    scale = angularRateScale;
  } else if (vec == BNO055_VECTOR_EULER) {
    scale = eulerScale;
  } else if (vec == BNO055_VECTOR_QUATERNION) {
    scale = quaScale;
  }

  bno055_vector_t xyz = {.w = 0, .x = 0, .y = 0, .z = 0};
  if (vec == BNO055_VECTOR_QUATERNION) {
    xyz.w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    xyz.z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
  } else {
    xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
  }

  return xyz;
}

bno055_vector_t bno055_getVectorAccelerometer() {
  return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}
bno055_vector_t bno055_getVectorMagnetometer() {
  return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}
bno055_vector_t bno055_getVectorGyroscope() {
  return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}
bno055_vector_t bno055_getVectorEuler() {
  return bno055_getVector(BNO055_VECTOR_EULER);
}
bno055_vector_t bno055_getVectorLinearAccel() {
  return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}
bno055_vector_t bno055_getVectorGravity() {
  return bno055_getVector(BNO055_VECTOR_GRAVITY);
}
bno055_vector_t bno055_getVectorQuaternion() {
  return bno055_getVector(BNO055_VECTOR_QUATERNION);
}

//---------------------------FUNCTION PART------------------------------------//

uint8_t rawIMUdata_8[25];
uint16_t rawIMUdata_16[9];

uint8_t caliber_value = 250;

IMU_offset imu_offset;

bno055_vector_t accel_for_offset;
bno055_vector_t gyro_for_offset;
bno055_vector_t euler_for_offset;

// pg 29
const uint8_t GyroPowerMode = NormalG;
const uint8_t GyroRange = GFS_1000DPS;
const uint8_t GyroBandwidth = GBW_523Hz;

// pg 28
const uint8_t AccelRange = AFS_8G;
const uint8_t AccelMode = NormalA;
const uint8_t AccelBandwidth = ABW_1000Hz;

// pg21
const uint8_t PWRMode = Normalpwr;
const uint8_t OPRMode = IMU_;

void bno055_config()
{
    bno055_setup();
    bno055_setOperationModeNDOF();
}

void init_IMU(void)
{
    bno055_config();

    offset_IMU();

    HAL_I2C_Mem_Read_IT(BNO055_I2C, BNO055_I2C_ADDR_LO << 1, BNO055_ACC_DATA_X_LSB, 1, rawIMUdata_8, 24);
}

void offset_IMU(void)
{
    static uint8_t counter;
    for (counter = 0; counter < caliber_value; counter++)
    {
        accel_for_offset = bno055_getVectorAccelerometer();
        imu_offset.accel.x += accel_for_offset.x;
        imu_offset.accel.y += accel_for_offset.y;
        imu_offset.accel.z += accel_for_offset.z;

        gyro_for_offset = bno055_getVectorGyroscope();
        imu_offset.gyro.x += gyro_for_offset.x;
        imu_offset.gyro.y += gyro_for_offset.y;
        imu_offset.gyro.z += gyro_for_offset.z;

        euler_for_offset = bno055_getVectorEuler();
        imu_offset.euler.x += euler_for_offset.x;
        imu_offset.euler.y += euler_for_offset.y;
        imu_offset.euler.z += euler_for_offset.z;
        bno055_delay(1);
    }

    imu_offset.accel.x /= caliber_value;
    imu_offset.accel.y /= caliber_value;
    imu_offset.accel.z /= caliber_value;

    imu_offset.gyro.x /= caliber_value;
    imu_offset.gyro.y /= caliber_value;
    imu_offset.gyro.z /= caliber_value;

    imu_offset.euler.x /= caliber_value;
    imu_offset.euler.y /= caliber_value;
    imu_offset.euler.z /= caliber_value;
}

void read_IMU(IMU *imu)
{
    //---------------Accel---------------//
    rawIMUdata_16[0] = ((uint16_t)rawIMUdata_8[1] << 8) | ((uint16_t)rawIMUdata_8[0]);
    rawIMUdata_16[1] = ((uint16_t)rawIMUdata_8[3] << 8) | ((uint16_t)rawIMUdata_8[2]);
    rawIMUdata_16[2] = ((uint16_t)rawIMUdata_8[5] << 8) | ((uint16_t)rawIMUdata_8[4]);
    //---------------Gyro---------------//
    rawIMUdata_16[3] = ((uint16_t)rawIMUdata_8[13] << 8) | ((uint16_t)rawIMUdata_8[12]);
    rawIMUdata_16[4] = ((uint16_t)rawIMUdata_8[15] << 8) | ((uint16_t)rawIMUdata_8[14]);
    rawIMUdata_16[5] = ((uint16_t)rawIMUdata_8[17] << 8) | ((uint16_t)rawIMUdata_8[16]);
    //---------------Euler---------------//
    rawIMUdata_16[6] = ((uint16_t)rawIMUdata_8[19] << 8) | ((uint16_t)rawIMUdata_8[18]);
    rawIMUdata_16[7] = ((uint16_t)rawIMUdata_8[21] << 8) | ((uint16_t)rawIMUdata_8[20]);
    rawIMUdata_16[8] = ((uint16_t)rawIMUdata_8[23] << 8) | ((uint16_t)rawIMUdata_8[22]);

    imu->accel.x = /*kalman filter*/ ((float)rawIMUdata_16[0]) / 100 - imu_offset.accel.x;
    imu->accel.y = /*kalman filter*/ ((float)rawIMUdata_16[1]) / 100 - imu_offset.accel.y;
    imu->accel.z = /*kalman filter*/ ((float)rawIMUdata_16[2]) / 100 - imu_offset.accel.z;

    imu->gyro.x = /*kalman filter*/ ((float)rawIMUdata_16[3]) / 16 - imu_offset.gyro.x;
    imu->gyro.y = /*kalman filter*/ ((float)rawIMUdata_16[4]) / 16 - imu_offset.gyro.y;
    imu->gyro.z = /*kalman filter*/ ((float)rawIMUdata_16[5]) / 16 - imu_offset.gyro.z;

    imu->euler.roll = /*kalman filter*/ ((float)rawIMUdata_16[6]) / 16 - imu_offset.euler.x;
    imu->euler.pitch = /*kalman filter*/ ((float)rawIMUdata_16[7]) / 16 - imu_offset.euler.y;
    imu->euler.yaw = /*kalman filter*/ ((float)rawIMUdata_16[8]) / 16 - imu_offset.euler.z;

    HAL_I2C_Mem_Read_IT(BNO055_I2C, BNO055_I2C_ADDR_LO << 1, BNO055_ACC_DATA_X_LSB, 1, rawIMUdata_8, 24);
}
