#pragma once


// ========== КЛАСС ACLGYR ==========
class AclGyr {
private:
  DFRobot_BMI160 bmi160;
  const int8_t i2c_addr = 0x69;
  
  // Калибровочные коэффициенты для гироскопа (смещение нуля в LSB)
  // gyro_SI[рад/с] = (raw_LSB - gyro_b[i]) * GYRO_LSB_TO_RAD_S
  float gyro_b[3] = {0, 0, 0};    // Смещение нуля по осям X,Y,Z (LSB)
  
  // Калибровочные коэффициенты для акселерометра (линейное преобразование)
  // accel_SI[м/с²] = accel_k[i] * raw_LSB
  float accel_k[3] = {0.000598, 0.000598, 0.000598}; // Коэффициенты для X,Y,Z
  
  // Константы преобразования по умолчанию (для диапазона ±250°/с и ±2g)
  const float GYRO_LSB_TO_RAD_S = 0.000133158; // LSB → рад/с (131.0 LSB/°/с * π/180)
  const float ACCEL_LSB_TO_MS2 = 0.000598;     // LSB → м/с² (16384 LSB/g * 9.80665)
  
  LiquidCrystal* lcd;

public:
  // Конструктор
  AclGyr(LiquidCrystal* lcd_ptr = nullptr) : lcd(lcd_ptr) {}
  
  // Инициализация датчика (без калибровки)
  bool begin(uint8_t sda_pin = 18, uint8_t scl_pin = 19) {
    Wire.begin(sda_pin, scl_pin);
    
    if (bmi160.softReset() != BMI160_OK) {
      return false;
    }
    
    if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
      return false;
    }
    
    return true;
  }
  
  // Получение калиброванных данных в СИ
  // gyroX,gyroY,gyroZ - угловые скорости в рад/с
  // accelX,accelY,accelZ - линейные ускорения в м/с²
  bool getCalibratedDataSI(float &gyroX, float &gyroY, float &gyroZ,
                          float &accelX, float &accelY, float &accelZ) {
    int16_t raw[6];
    
    if (bmi160.getAccelGyroData(raw) != BMI160_OK) {
      return false;
    }
    
    // Гироскоп: угловая скорость в рад/с
    gyroX = (raw[0] - gyro_b[0]) * GYRO_LSB_TO_RAD_S;
    gyroY = (raw[1] - gyro_b[1]) * GYRO_LSB_TO_RAD_S;
    gyroZ = (raw[2] - gyro_b[2]) * GYRO_LSB_TO_RAD_S;
    
    // Акселерометр: линейное ускорение в м/с²
    accelX = raw[3] * accel_k[0]; // Используем калибровочный коэффициент
    accelY = raw[4] * accel_k[1];
    accelZ = raw[5] * accel_k[2];
    
    return true;
  }
  
  // Получение сырых данных (для отладки)
  bool getRawData(int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ,
                  int16_t &accelX, int16_t &accelY, int16_t &accelZ) {
    int16_t raw[6];
    
    if (bmi160.getAccelGyroData(raw) != BMI160_OK) {
      return false;
    }
    
    gyroX = raw[0];
    gyroY = raw[1];
    gyroZ = raw[2];
    accelX = raw[3];
    accelY = raw[4];
    accelZ = raw[5];
    
    return true;
  }
  
  // Установка калибровочных коэффициентов гироскопа (смещение нуля в LSB)
  void setGyroCalibration(float biasX, float biasY, float biasZ) {
    gyro_b[0] = biasX;
    gyro_b[1] = biasY;
    gyro_b[2] = biasZ;
  }
  
  // Установка калибровочных коэффициентов акселерометра (LSB → м/с²)
  void setAccelCalibration(float coeffX, float coeffY, float coeffZ) {
    accel_k[0] = coeffX;
    accel_k[1] = coeffY;
    accel_k[2] = coeffZ;
  }
  
  // Получение текущих коэффициентов
  void getCalibration(float &gbX, float &gbY, float &gbZ,
                     float &akX, float &akY, float &akZ) {
    gbX = gyro_b[0];
    gbY = gyro_b[1];
    gbZ = gyro_b[2];
    akX = accel_k[0];
    akY = accel_k[1];
    akZ = accel_k[2];
  }
  
  // Тест датчика
  bool test() {
    int16_t raw[6];
    return (bmi160.getAccelGyroData(raw) == BMI160_OK);
  }
};