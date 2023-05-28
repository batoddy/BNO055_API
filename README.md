# BNO055_API
A bno055 library which can usable with both polling or interrupt way. I2C protocol used.

The "bno055_stm32.h" library was implemented to this library.

###### BNO055_API.c
```C
extern I2C_HandleTypeDef hi2c1; // Change the I2C Handler
#define BNO055_I2C &hi2c1	// *
```
Change the I2C handler for your project

Use ```init_IMU();``` function for initializing the module in USER CODE BEGIN 2 

If you don't want to use in interrupt mode, delete or comment out the
###### BNO055_API.c
```C
HAL_I2C_Mem_Read_IT(BNO055_I2C, BNO055_I2C_ADDR_LO << 1, BNO055_ACC_DATA_X_LSB, 1, rawIMUdata_8, 24); 
``` 
line in ```init_IMU();``` function. For getting data with polling method, use bno055_getVector();
functions.

If you want to use interrupt method, use this CallBack function in ```main.c``` file. 
```C
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == hi2c1.Instance)
  {
    /* BNO055 IT*/
	  read_IMU(&imu);
  }
}
```
you can see a example usage of the library with the interrupt mode.
