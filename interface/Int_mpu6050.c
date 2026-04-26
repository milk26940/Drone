#include "Int_mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>

int16_t acc_x_offset = 0;
int16_t acc_y_offset = 0;
int16_t acc_z_offset = 0;
int16_t gyro_x_offset = 0;
int16_t gyro_y_offset = 0;
int16_t gyro_z_offset = 0;

void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
  UNUSED(I2Cx);
  UNUSED(I2C_EVENT);
}

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
  uint8_t Data = 0;

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);

  return Data;
}

void MPU6050_Init(void)
{
  MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
  MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
  MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
  MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
  MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
  MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
  return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
  uint8_t Data[14] = {0};

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, Data, 14, 100);

  *AccX = (int16_t)((Data[0] << 8) | Data[1]);
  *AccY = (int16_t)((Data[2] << 8) | Data[3]);
  *AccZ = (int16_t)((Data[4] << 8) | Data[5]);
  *GyroX = (int16_t)((Data[8] << 8) | Data[9]);
  *GyroY = (int16_t)((Data[10] << 8) | Data[11]);
  *GyroZ = (int16_t)((Data[12] << 8) | Data[13]);
}

void Int_MPU6050_Get_Acc(Accel_struct *accel)
{
  int16_t gyro_x = 0;
  int16_t gyro_y = 0;
  int16_t gyro_z = 0;

  MPU6050_GetData(&accel->accel_x, &accel->accel_y, &accel->accel_z,
                  &gyro_x, &gyro_y, &gyro_z);
}

void Int_MPU6050_Get_Data(Gyro_Accel_Struct *data)
{
  MPU6050_GetData(&data->accel.accel_x, &data->accel.accel_y, &data->accel.accel_z,
                  &data->gyro.gyro_x, &data->gyro.gyro_y, &data->gyro.gyro_z);

  data->accel.accel_x -= acc_x_offset;
  data->accel.accel_y -= acc_y_offset;
  data->accel.accel_z -= acc_z_offset;
  data->gyro.gyro_x -= gyro_x_offset;
  data->gyro.gyro_y -= gyro_y_offset;
  data->gyro.gyro_z -= gyro_z_offset;
}

/**
 * @brief 在初始化MPU6050完成之后 对MPU6050进行零偏校准
 */
void Int_MPU6050_calculate_offset(void)
{
  Accel_struct current_accel = {0};
  Accel_struct last_accel = {0};
  uint8_t count = 0;

  Int_MPU6050_Get_Acc(&last_accel);

  while (count < 100)
  {
    Int_MPU6050_Get_Acc(&current_accel);

    if (abs(current_accel.accel_x - last_accel.accel_x) < 400 &&
        abs(current_accel.accel_y - last_accel.accel_y) < 400 &&
        abs(current_accel.accel_z - last_accel.accel_z) < 400)
    {
      count++;
    }
    else
    {
      count = 0;
    }

    last_accel = current_accel;
    vTaskDelay(pdMS_TO_TICKS(6));
  }

  Gyro_Accel_Struct gyro_accel_data = {0};
  int32_t acc_x_sum = 0;
  int32_t acc_y_sum = 0;
  int32_t acc_z_sum = 0;
  int32_t gyro_x_sum = 0;
  int32_t gyro_y_sum = 0;
  int32_t gyro_z_sum = 0;

  for (uint8_t i = 0; i < 100; i++)
  {
    Int_MPU6050_Get_Data(&gyro_accel_data);

    acc_x_sum += gyro_accel_data.accel.accel_x;
    acc_y_sum += gyro_accel_data.accel.accel_y;
    acc_z_sum += (gyro_accel_data.accel.accel_z - 2048);
    gyro_x_sum += gyro_accel_data.gyro.gyro_x;
    gyro_y_sum += gyro_accel_data.gyro.gyro_y;
    gyro_z_sum += gyro_accel_data.gyro.gyro_z;

    vTaskDelay(pdMS_TO_TICKS(6));
  }

  acc_x_offset = acc_x_sum / 100;
  acc_y_offset = acc_y_sum / 100;
  acc_z_offset = acc_z_sum / 100;
  gyro_x_offset = gyro_x_sum / 100;
  gyro_y_offset = gyro_y_sum / 100;
  gyro_z_offset = gyro_z_sum / 100;
}
