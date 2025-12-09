#pragma once

#include "DFRobot_BMI160.h"
#include <LiquidCrystal.h>
#include <Wire.h>
#include <aclgyr.h>
#include <allfilt.h>
#include<integr.h>

// ========== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ==========
LiquidCrystal lcd(25, 26, 22, 23, 21, 27);
AclGyr imu(&lcd);            // IMU с калибровкой
TrajectoryIntegrator integrator(0.01f); // Интегратор с dt=0.01 с (100 Гц)

// ========== ПЕРЕМЕННЫЕ ДЛЯ СБОРА ДАННЫХ ==========
const int WINDOW_SIZE = 10;           // Размер окна для фильтрации
float accel_x_buffer[WINDOW_SIZE];    // Буфер для акселерометра X
float accel_y_buffer[WINDOW_SIZE];    // Буфер для акселерометра Y
float accel_z_buffer[WINDOW_SIZE];    // Буфер для акселерометра Z
float gyro_x_buffer[WINDOW_SIZE];     // Буфер для гироскопа X
float gyro_y_buffer[WINDOW_SIZE];     // Буфер для гироскопа Y
float gyro_z_buffer[WINDOW_SIZE];     // Буфер для гироскопа Z

int buffer_index = 0;                 // Текущий индекс в буфере
bool buffer_full = false;             // Флаг заполнения буфера

// ========== НАСТРОЙКА ПЕРИОДОВ ВЫВОДА ==========
unsigned long last_integration_time = 0;
const unsigned long INTEGRATION_INTERVAL = 10; // 10 мс (100 Гц)

unsigned long last_display_time = 0;
const unsigned long DISPLAY_INTERVAL = 100;    // 100 мс (10 Гц)

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ ИНЕРЦИАЛЬНОЙ НАВИГАЦИИ ===");
  
  // 1. ИНИЦИАЛИЗАЦИЯ IMU
  Serial.println("1. Инициализация IMU BMI160...");
  if (!imu.begin()) {
    Serial.println("ОШИБКА: Не удалось инициализировать IMU!");
    while(1);
  }
  Serial.println("IMU инициализирован успешно");
  
  // 2. ОПРЕДЕЛЕНИЕ НАЧАЛЬНОЙ ОРИЕНТАЦИИ ПО ГРАВИТАЦИИ
  Serial.println("\n2. Определение начальной ориентации...");
  Serial.println("Держите датчик неподвижно 2 секунды");
  
  const int INIT_SAMPLES = 200;  // 200 выборок за ~2 секунды
  float init_accel_x[INIT_SAMPLES];
  float init_accel_y[INIT_SAMPLES];
  float init_accel_z[INIT_SAMPLES];
  
  // Сбор данных для определения гравитации
  for (int i = 0; i < INIT_SAMPLES; i++) {
    float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
    
    // Получаем калиброванные данные
    if (imu.getCalibratedDataSI(gyroX, gyroY, gyroZ, accelX, accelY, accelZ)) {
      init_accel_x[i] = accelX;
      init_accel_y[i] = accelY;
      init_accel_z[i] = accelZ;
    }
    
    delay(10); // 100 Гц
  }
  
  // Усредняем вектор гравитации
  float avg_gx = 0, avg_gy = 0, avg_gz = 0;
  for (int i = 0; i < INIT_SAMPLES; i++) {
    avg_gx += init_accel_x[i];
    avg_gy += init_accel_y[i];
    avg_gz += init_accel_z[i];
  }
  avg_gx /= INIT_SAMPLES;
  avg_gy /= INIT_SAMPLES;
  avg_gz /= INIT_SAMPLES;
  
  Serial.print("Средний вектор гравитации: (");
  Serial.print(avg_gx, 3); Serial.print(", ");
  Serial.print(avg_gy, 3); Serial.print(", ");
  Serial.print(avg_gz, 3); Serial.println(") м/с²");
  
  // 3. ИНИЦИАЛИЗАЦИЯ ИНТЕГРАТОРА
  Serial.println("\n3. Инициализация интегратора траектории...");
  integrator.reset(); // Сбрасываем состояние
  integrator.initFromGravityVector(avg_gx, avg_gy, avg_gz);
  
  // 4. ИНИЦИАЛИЗАЦИЯ БУФЕРОВ
  Serial.println("4. Инициализация буферов фильтрации...");
  for (int i = 0; i < WINDOW_SIZE; i++) {
    accel_x_buffer[i] = avg_gx;
    accel_y_buffer[i] = avg_gy;
    accel_z_buffer[i] = avg_gz;
    gyro_x_buffer[i] = 0;
    gyro_y_buffer[i] = 0;
    gyro_z_buffer[i] = 0;
  }
  
  Serial.println("\n=== СИСТЕМА ГОТОВА К РАБОТЕ ===");
  Serial.println("Начальные условия:");
  Serial.println("- Позиция: (0, 0, 0) м");
  Serial.println("- Скорость: (0, 0, 0) м/с");
  Serial.println("- Ориентация определена по гравитации");
  Serial.println("\n=== НАЧАЛО ИНТЕГРИРОВАНИЯ ===");
  Serial.println("Время(с) | Позиция X,Y,Z(м) | Скорость X,Y,Z(м/с) | Углы Roll,Pitch,Yaw(рад)");
  Serial.println("---------------------------------------------------------------------------");
  
  last_integration_time = millis();
  last_display_time = millis();
}

// ========== LOOP ==========
void loop() {
  unsigned long current_time = millis();
  
  // 1. СБОР И КАЛИБРОВКА ДАННЫХ С IMU
  float raw_gyroX, raw_gyroY, raw_gyroZ;
  float raw_accelX, raw_accelY, raw_accelZ;
  
  if (!imu.getCalibratedDataSI(raw_gyroX, raw_gyroY, raw_gyroZ, 
                               raw_accelX, raw_accelY, raw_accelZ)) {
    Serial.println("ОШИБКА чтения IMU!");
    delay(100);
    return;
  }
  
  // 2. ЗАПОЛНЕНИЕ БУФЕРОВ
  accel_x_buffer[buffer_index] = raw_accelX;
  accel_y_buffer[buffer_index] = raw_accelY;
  accel_z_buffer[buffer_index] = raw_accelZ;
  gyro_x_buffer[buffer_index] = raw_gyroX;
  gyro_y_buffer[buffer_index] = raw_gyroY;
  gyro_z_buffer[buffer_index] = raw_gyroZ;
  
  buffer_index = (buffer_index + 1) % WINDOW_SIZE;
  if (buffer_index == 0) buffer_full = true;
  
  // 3. ФИЛЬТРАЦИЯ (когда буфер заполнен)
  if (buffer_full && (current_time - last_integration_time >= INTEGRATION_INTERVAL)) {
    // Применяем фильтрацию ко всему окну
    FilteredAverages filtered = all_filt(
      accel_x_buffer, accel_y_buffer, accel_z_buffer,
      gyro_x_buffer, gyro_y_buffer, gyro_z_buffer,
      WINDOW_SIZE
    );
    
    // 4. ИНТЕГРИРОВАНИЕ ФИЛЬТРОВАННЫХ ДАННЫХ
    integrator.update(
      filtered.Acl_Xavg, filtered.Acl_Yavg, filtered.Acl_Zavg,
      filtered.Gyr_Xavg, filtered.Gyr_Yavg, filtered.Gyr_Zavg
    );
    
    last_integration_time = current_time;
  }
  
  // 5. ВЫВОД РЕЗУЛЬТАТОВ В SERIAL MONITOR (10 Гц)
  if (current_time - last_display_time >= DISPLAY_INTERVAL) {
    // Получаем текущее состояние
    StateVector state = integrator.getCurrentState();
    
    // Выводим все 12 значений в СИ
    Serial.print(state.x, 3); Serial.print(",");
    Serial.print(state.y, 3); Serial.print(",");
    Serial.print(state.z, 3); Serial.print(",");
    
    Serial.print(state.vx, 3); Serial.print(",");
    Serial.print(state.vy, 3); Serial.print(",");
    Serial.print(state.vz, 3); Serial.print(",");
    
    Serial.print(state.ax, 3); Serial.print(",");
    Serial.print(state.ay, 3); Serial.print(",");
    Serial.print(state.az, 3); Serial.print(",");
    
    Serial.print(state.roll, 4); Serial.print(",");
    Serial.print(state.pitch, 4); Serial.print(",");
    Serial.print(state.yaw, 4); Serial.print(",");
    
    Serial.print(state.wx, 4); Serial.print(",");
    Serial.print(state.wy, 4); Serial.print(",");
    Serial.println(state.wz, 4);
    
    last_display_time = current_time;
  }
  
  // Минимальная задержка для стабильной частоты
  delay(1);
}

// ========== ФУНКЦИЯ ДЛЯ ЧТЕНИЯ ДАННЫХ В CSV ФОРМАТЕ ==========
/*
Для чтения данных в Excel/MATLAB/Python используйте следующий формат:
Время(неявно), PosX, PosY, PosZ, VelX, VelY, VelZ, AccX, AccY, AccZ, Roll, Pitch, Yaw, Wx, Wy, Wz

Пример строки:
0.123, 0.001, 0.002, 0.003, 0.100, 0.050, -0.020, 0.500, 0.200, 9.810, 0.052, -0.035, 0.001, 0.010, -0.005, 0.002

Где:
- Позиция: метры (м)
- Скорость: м/с
- Ускорение: м/с²
- Углы: радианы (рад)
- Угловые скорости: рад/с
*/