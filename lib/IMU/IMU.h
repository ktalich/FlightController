#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <MPU9250.h>

#define I2C_SDA 8
#define I2C_SCL 9
#define MPU9250_IMU_ADDRESS 0x68
#define MAGNETIC_DECLINATION 4.94
#define IMU_CALIBRATION_EEPROM_ADRESS 0x01

struct CalibrationData
{
    float accBiasX;
    float accBiasY;
    float accBiasZ;
    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;
    float magBiasX;
    float magBiasY;
    float magBiasZ;
    float magScaleX;
    float magScaleY;
    float magScaleZ;
    //add checksum
};

class IMU
{
    MPU9250 mpu;

public:
    volatile float Pitch, Roll, Yaw;
    volatile float GyroX, GyroY, GyroZ;

    IMU()
    {
    }

    void begin()
    {
        Serial.println("Starting I2C bus...");
        Wire.setSDA(I2C_SDA);
        Wire.setSCL(I2C_SCL);
        Wire.begin();
        Wire.setClock(400000);

        MPU9250Setting setting;
        setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
        setting.gyro_fs_sel = GYRO_FS_SEL::G1000DPS;
        setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
        setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_250HZ;
        setting.gyro_fchoice = 0x03;
        setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ;//DLPF_45HZ
        setting.accel_fchoice = 0x01;
        setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;//DLPF_45HZ
        mpu.setup(MPU9250_IMU_ADDRESS, setting);
        delay(5000);

        mpu.verbose(true);

        mpu.setMagneticDeclination(MAGNETIC_DECLINATION);
        mpu.selectFilter(QuatFilterSel::MADGWICK);
        mpu.setFilterIterations(12);

        Serial.println("IMU Ready!");
        mpu.verbose(false);
        delay(1000);
    }

    void calibrateImu()
    {
        Serial.println("Calibrating Gyroscope & Accelerometer...");
        delay(1000);

        mpu.calibrateAccelGyro();

        Serial.println("Gyroscope & Accelerometer calibrated.");
        Serial.println("Calibrating Magnetometer... Move in a figure 8 pattern in 5 sec...");
        delay(5000);

        mpu.calibrateMag();

        Serial.println("Magnetometer calibrated.");

        Serial.println("Saving calibration data...");

        CalibrationData calData = {
            mpu.getAccBias(0),
            mpu.getAccBias(1),
            mpu.getAccBias(2),
            mpu.getGyroBias(0),
            mpu.getGyroBias(1),
            mpu.getGyroBias(2),
            mpu.getMagBias(0),
            mpu.getMagBias(1),
            mpu.getMagBias(2),
            mpu.getMagScale(0),
            mpu.getMagScale(1),
            mpu.getMagScale(2)};
        EEPROM.write(0, 1);
        EEPROM.put(IMU_CALIBRATION_EEPROM_ADRESS, calData);

        Serial.print("Accelerometer bias X:\t");
        Serial.print(calData.accBiasX);
        Serial.print("\t");
        Serial.print("Accelerometer bias Y:\t");
        Serial.print(calData.accBiasY);
        Serial.print("\t");
        Serial.print("Accelerometer bias Z:\t");
        Serial.print(calData.accBiasZ);
        Serial.print("\t");
        Serial.println();
        Serial.print("Gyroscope bias X:\t");
        Serial.print(calData.gyroBiasX);
        Serial.print("\t");
        Serial.print("Gyroscope bias Y:\t");
        Serial.print(calData.gyroBiasY);
        Serial.print("\t");
        Serial.print("Gyroscope bias Z:\t");
        Serial.print(calData.gyroBiasX);
        Serial.print("\t");
        Serial.println();
        Serial.print("Magnetometer bias X:\t");
        Serial.print(calData.magBiasX);
        Serial.print("\t");
        Serial.print("Magnetometer bias Y:\t");
        Serial.print(calData.magBiasY);
        Serial.print("\t");
        Serial.print("Magnetometer bias Z:\t");
        Serial.print(calData.magBiasZ);
        Serial.print("\t");
        Serial.println();
        Serial.print("Magnetometer scale X:\t");
        Serial.print(calData.magScaleX);
        Serial.print("\t");
        Serial.print("Magnetometer scale Y:\t");
        Serial.print(calData.magScaleY);
        Serial.print("\t");
        Serial.print("Magnetometer scale Z:\t");
        Serial.print(calData.magScaleZ);
        Serial.print("\t");
        Serial.println();

        delay(2000);

        Serial.println("Calibration data saved.");
    }

    void loadCalibration()
    {
        Serial.println("Loading calibration data...");

        CalibrationData calData;
        EEPROM.get(IMU_CALIBRATION_EEPROM_ADRESS, calData);

        mpu.setAccBias(calData.accBiasX, calData.accBiasY, calData.accBiasZ);
        mpu.setGyroBias(calData.gyroBiasX, calData.gyroBiasY, calData.gyroBiasZ);
        mpu.setMagBias(calData.magBiasX, calData.magBiasY, calData.magBiasZ);
        mpu.setMagScale(calData.magScaleX, calData.magScaleY, calData.magScaleZ);

        Serial.println("Calibration loaded.");
        Serial.print("Accelerometer bias X:\t");
        Serial.print(calData.accBiasX);
        Serial.print("\t");
        Serial.print("Accelerometer bias Y:\t");
        Serial.print(calData.accBiasY);
        Serial.print("\t");
        Serial.print("Accelerometer bias Z:\t");
        Serial.print(calData.accBiasZ);
        Serial.print("\t");
        Serial.println();
        Serial.print("Gyroscope bias X:\t");
        Serial.print(calData.gyroBiasX);
        Serial.print("\t");
        Serial.print("Gyroscope bias Y:\t");
        Serial.print(calData.gyroBiasY);
        Serial.print("\t");
        Serial.print("Gyroscope bias Z:\t");
        Serial.print(calData.gyroBiasX);
        Serial.print("\t");
        Serial.println();
        Serial.print("Magnetometer bias X:\t");
        Serial.print(calData.magBiasX);
        Serial.print("\t");
        Serial.print("Magnetometer bias Y:\t");
        Serial.print(calData.magBiasY);
        Serial.print("\t");
        Serial.print("Magnetometer bias Z:\t");
        Serial.print(calData.magBiasZ);
        Serial.print("\t");
        Serial.println();
        Serial.print("Magnetometer scale X:\t");
        Serial.print(calData.magScaleX);
        Serial.print("\t");
        Serial.print("Magnetometer scale Y:\t");
        Serial.print(calData.magScaleY);
        Serial.print("\t");
        Serial.print("Magnetometer scale Z:\t");
        Serial.print(calData.magScaleZ);
        Serial.print("\t");
        Serial.println();
    }

    void update()
    {
        if (mpu.update())
        {
            Pitch = mpu.getPitch();
            Roll = mpu.getRoll();
            Yaw = mpu.getYaw();

            GyroX = mpu.getGyroX();
            GyroY = mpu.getGyroY();
            GyroZ = mpu.getGyroZ();
        }

        // if (mpu.update()) {
        //   ypr[0] = mpu.getEulerZ();
        //   ypr[1] = mpu.getEulerY();
        //   ypr[2] = mpu.getEulerX();
        //   Serial.printf("yaw: %f, pitch: %f, roll: %f\n", ypr[0], ypr[1], ypr[2]);
        // }
    }
};
