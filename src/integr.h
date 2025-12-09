#pragma once
#include <vector>
#include <cmath>

// Структура для хранения вектора состояния
struct StateVector {
    float x, y, z;       // Позиция (м)
    float vx, vy, vz;    // Скорость (м/с)
    float ax, ay, az;    // Ускорение (м/с²)
    float roll, pitch, yaw;    // Углы ориентации (рад)
    float wx, wy, wz;    // Угловая скорость (рад/с)
};

// Класс для хранения траекторий и интегрирования
class TrajectoryIntegrator {
private:
    // Векторы траекторий
    std::vector<float> pos_x;
    std::vector<float> pos_y;
    std::vector<float> pos_z;
    
    std::vector<float> vel_x;
    std::vector<float> vel_y;
    std::vector<float> vel_z;
    
    std::vector<float> accel_body_x;
    std::vector<float> accel_body_y;
    std::vector<float> accel_body_z;
    
    std::vector<float> accel_world_x;
    std::vector<float> accel_world_y;
    std::vector<float> accel_world_z;
    
    std::vector<float> linear_accel_world_x; // Без гравитации
    std::vector<float> linear_accel_world_y;
    std::vector<float> linear_accel_world_z;
    
    std::vector<float> angle_roll;
    std::vector<float> angle_pitch;
    std::vector<float> angle_yaw;
    
    std::vector<float> angvel_x;
    std::vector<float> angvel_y;
    std::vector<float> angvel_z;
    
    // Кватернион ориентации
    float q0, q1, q2, q3;
    
    // Временные параметры
    float current_time;
    float dt;
    
    // Пределы памяти
    const size_t max_history = 1000;
    
public:
    // Конструктор
    TrajectoryIntegrator(float time_step = 0.01f) : dt(time_step) {
        reset();
    }
    
    // Сброс к начальным условиям
    void reset() {
        // Очищаем все векторы
        pos_x.clear(); pos_y.clear(); pos_z.clear();
        vel_x.clear(); vel_y.clear(); vel_z.clear();
        accel_body_x.clear(); accel_body_y.clear(); accel_body_z.clear();
        accel_world_x.clear(); accel_world_y.clear(); accel_world_z.clear();
        linear_accel_world_x.clear(); linear_accel_world_y.clear(); linear_accel_world_z.clear();
        angle_roll.clear(); angle_pitch.clear(); angle_yaw.clear();
        angvel_x.clear(); angvel_y.clear(); angvel_z.clear();
        
        // Начальное состояние
        pos_x.push_back(0.0f); pos_y.push_back(0.0f); pos_z.push_back(0.0f);
        vel_x.push_back(0.0f); vel_y.push_back(0.0f); vel_z.push_back(0.0f);
        accel_body_x.push_back(0.0f); accel_body_y.push_back(0.0f); accel_body_z.push_back(0.0f);
        accel_world_x.push_back(0.0f); accel_world_y.push_back(0.0f); accel_world_z.push_back(0.0f);
        linear_accel_world_x.push_back(0.0f); linear_accel_world_y.push_back(0.0f); linear_accel_world_z.push_back(0.0f);
        
        // Начальные углы: 0, 0, 0 (будет переопределено через initFromGravity)
        angle_roll.push_back(0.0f);
        angle_pitch.push_back(0.0f);
        angle_yaw.push_back(0.0f);
        
        angvel_x.push_back(0.0f); angvel_y.push_back(0.0f); angvel_z.push_back(0.0f);
        
        // Единичный кватернион (будет переопределено)
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
        
        current_time = 0.0f;
    }
    
    // ИНИЦИАЛИЗАЦИЯ ПО ВЕКТОРУ ГРАВИТАЦИИ (самая важная функция!)
    void initFromGravityVector(float gx, float gy, float gz) {
        // Нормализуем вектор гравитации
        float g_norm = sqrt(gx*gx + gy*gy + gz*gz);
        if (g_norm > 0) {
            gx /= g_norm;
            gy /= g_norm;
            gz /= g_norm;
        }
        
        // Вычисляем начальные углы по вектору гравитации
        // Гравитация направлена ВНИЗ в мировой системе (0,0,-1)
        // Но акселерометр измеряет силу реакции опоры, которая направлена ВВЕРХ
        
        // pitch (тангаж) - угол вокруг оси Y
        // sin(pitch) = -gx / |g| (с учетом знака)
        float pitch = asin(-gx); // gx - проекция гравитации на ось X тела
        
        // roll (крен) - угол вокруг оси X
        // roll = atan2(gy, gz)
        float roll = atan2(gy, gz);
        
        // yaw (рыскание) - не определяется по гравитации, оставляем 0
        float yaw = 0.0f;
        
        // Устанавливаем начальные углы
        angle_roll.clear(); angle_roll.push_back(roll);
        angle_pitch.clear(); angle_pitch.push_back(pitch);
        angle_yaw.clear(); angle_yaw.push_back(yaw);
        
        // Обновляем кватернион из углов Эйлера
        updateQuaternionFromEuler(roll, pitch, yaw);
        
        Serial.print("Initialized from gravity vector: ");
        Serial.print("g=("); Serial.print(gx, 3); Serial.print(", ");
        Serial.print(gy, 3); Serial.print(", "); Serial.print(gz, 3); Serial.println(")");
        Serial.print("Initial angles: roll="); Serial.print(roll * 180.0/PI, 1);
        Serial.print("°, pitch="); Serial.print(pitch * 180.0/PI, 1);
        Serial.print("°, yaw="); Serial.print(yaw * 180.0/PI, 1); Serial.println("°");
    }
    
    // Основной метод обновления С ВЫЧИТАНИЕМ ГРАВИТАЦИИ В МИРОВЫХ КООРДИНАТАХ
    void update(
        float accel_body_x_new, float accel_body_y_new, float accel_body_z_new,
        float gyro_x_new, float gyro_y_new, float gyro_z_new
    ) {
        // 1. СОХРАНЯЕМ ИЗМЕРЕННЫЕ УСКОРЕНИЯ В СИСТЕМЕ ТЕЛА
        accel_body_x.push_back(accel_body_x_new);
        accel_body_y.push_back(accel_body_y_new);
        accel_body_z.push_back(accel_body_z_new);
        
        // 2. ОБНОВЛЯЕМ УГЛЫ ОРИЕНТАЦИИ
        updateAngles(gyro_x_new, gyro_y_new, gyro_z_new);
        
        // 3. ПРЕОБРАЗУЕМ УСКОРЕНИЯ В МИРОВУЮ СИСТЕМУ КООРДИНАТ
        float world_accel_x, world_accel_y, world_accel_z;
        bodyToWorld(accel_body_x_new, accel_body_y_new, accel_body_z_new,
                   world_accel_x, world_accel_y, world_accel_z);
        
        // 4. СОХРАНЯЕМ ПОЛНЫЕ УСКОРЕНИЯ В МИРОВОЙ СИСТЕМЕ
        accel_world_x.push_back(world_accel_x);
        accel_world_y.push_back(world_accel_y);
        accel_world_z.push_back(world_accel_z);
        
        // 5. ВЫЧИТАЕМ ГРАВИТАЦИЮ В МИРОВЫХ КООРДИНАТАХ
        // Гравитация направлена ВНИЗ по оси Z мира: (0, 0, -9.80665)
        float linear_accel_x = world_accel_x;
        float linear_accel_y = world_accel_y;
        float linear_accel_z = world_accel_z + 9.80665f; // + потому что гравитация вниз
        
        // 6. СОХРАНЯЕМ ЛИНЕЙНЫЕ УСКОРЕНИЯ (БЕЗ ГРАВИТАЦИИ)
        linear_accel_world_x.push_back(linear_accel_x);
        linear_accel_world_y.push_back(linear_accel_y);
        linear_accel_world_z.push_back(linear_accel_z);
        
        // 7. ИНТЕГРИРУЕМ ЛИНЕЙНЫЕ УСКОРЕНИЯ ДЛЯ ПОЛУЧЕНИЯ СКОРОСТИ
        integrateAcceleration(linear_accel_x, linear_accel_y, linear_accel_z);
        
        // 8. ИНТЕГРИРУЕМ СКОРОСТЬ ДЛЯ ПОЛУЧЕНИЯ ПОЗИЦИИ
        integrateVelocity();
        
        // 9. УПРАВЛЕНИЕ РАЗМЕРОМ ИСТОРИИ
        manageHistorySize();
        
        current_time += dt;
    }
    
    // Получение текущего состояния (возвращаем линейные ускорения без гравитации)
    StateVector getCurrentState() {
        StateVector state;
        
        if (!pos_x.empty()) {
            state.x = pos_x.back();
            state.y = pos_y.back();
            state.z = pos_z.back();
            
            state.vx = vel_x.back();
            state.vy = vel_y.back();
            state.vz = vel_z.back();
            
            // Возвращаем ЛИНЕЙНЫЕ ускорения (без гравитации)
            state.ax = linear_accel_world_x.back();
            state.ay = linear_accel_world_y.back();
            state.az = linear_accel_world_z.back();
            
            state.roll = angle_roll.back();
            state.pitch = angle_pitch.back();
            state.yaw = angle_yaw.back();
            
            state.wx = angvel_x.back();
            state.wy = angvel_y.back();
            state.wz = angvel_z.back();
        }
        
        return state;
    }
    
    // Получение гравитационного вектора в системе тела
    void getGravityInBodyFrame(float &gx, float &gy, float &gz) {
        // Гравитация в мировой системе: (0, 0, -9.80665)
        float world_gravity[3] = {0, 0, -9.80665f};
        
        // Преобразуем в систему тела
        float R[3][3];
        getRotationMatrix(R);
        
        // Обратное преобразование (мировая → тело)
        gx = R[0][0] * world_gravity[0] + R[1][0] * world_gravity[1] + R[2][0] * world_gravity[2];
        gy = R[0][1] * world_gravity[0] + R[1][1] * world_gravity[1] + R[2][1] * world_gravity[2];
        gz = R[0][2] * world_gravity[0] + R[1][2] * world_gravity[1] + R[2][2] * world_gravity[2];
    }
    
    // Определение начального положения по усредненному вектору гравитации
    static void estimateInitialOrientation(const float* accel_x, const float* accel_y, const float* accel_z, 
                                         int sample_count, float &init_roll, float &init_pitch, float &init_yaw) {
        // Усредняем измерения акселерометра
        float avg_gx = 0, avg_gy = 0, avg_gz = 0;
        for (int i = 0; i < sample_count; i++) {
            avg_gx += accel_x[i];
            avg_gy += accel_y[i];
            avg_gz += accel_z[i];
        }
        avg_gx /= sample_count;
        avg_gy /= sample_count;
        avg_gz /= sample_count;
        
        // Нормализуем
        float g_norm = sqrt(avg_gx*avg_gx + avg_gy*avg_gy + avg_gz*avg_gz);
        if (g_norm > 0) {
            avg_gx /= g_norm;
            avg_gy /= g_norm;
            avg_gz /= g_norm;
        }
        
        // Вычисляем углы
        // pitch = asin(-gx) - потому что при наклоне вперед gx становится положительным
        init_pitch = asin(-avg_gx);
        
        // roll = atan2(gy, gz)
        init_roll = atan2(avg_gy, avg_gz);
        
        // yaw не определяется по гравитации
        init_yaw = 0.0f;
        
        Serial.print("Gravity-based initialization: ");
        Serial.print("avg_g=("); Serial.print(avg_gx, 3); Serial.print(", ");
        Serial.print(avg_gy, 3); Serial.print(", "); Serial.print(avg_gz, 3); Serial.println(")");
        Serial.print("Angles: roll="); Serial.print(init_roll * 180.0/PI, 1);
        Serial.print("°, pitch="); Serial.print(init_pitch * 180.0/PI, 1);
        Serial.print("°, yaw="); Serial.print(init_yaw * 180.0/PI, 1); Serial.println("°");
    }
    
    // Вспомогательные геттеры
    void getBodyAcceleration(float &ax, float &ay, float &az) {
        if (!accel_body_x.empty()) {
            ax = accel_body_x.back();
            ay = accel_body_y.back();
            az = accel_body_z.back();
        } else {
            ax = ay = az = 0.0f;
        }
    }
    
    void getWorldAcceleration(float &ax, float &ay, float &az) {
        if (!accel_world_x.empty()) {
            ax = accel_world_x.back();
            ay = accel_world_y.back();
            az = accel_world_z.back();
        } else {
            ax = ay = az = 0.0f;
        }
    }
    
    void getLinearAcceleration(float &ax, float &ay, float &az) {
        if (!linear_accel_world_x.empty()) {
            ax = linear_accel_world_x.back();
            ay = linear_accel_world_y.back();
            az = linear_accel_world_z.back();
        } else {
            ax = ay = az = 0.0f;
        }
    }
    
    void getRotationMatrix(float R[3][3]) {
        R[0][0] = 1 - 2*q2*q2 - 2*q3*q3;
        R[0][1] = 2*(q1*q2 - q0*q3);
        R[0][2] = 2*(q1*q3 + q0*q2);
        
        R[1][0] = 2*(q1*q2 + q0*q3);
        R[1][1] = 1 - 2*q1*q1 - 2*q3*q3;
        R[1][2] = 2*(q2*q3 - q0*q1);
        
        R[2][0] = 2*(q1*q3 - q0*q2);
        R[2][1] = 2*(q2*q3 + q0*q1);
        R[2][2] = 1 - 2*q1*q1 - 2*q2*q2;
    }
    
    void getQuaternion(float &q0_out, float &q1_out, float &q2_out, float &q3_out) {
        q0_out = q0;
        q1_out = q1;
        q2_out = q2;
        q3_out = q3;
    }
    
    void setTimeStep(float time_step) {
        dt = time_step;
    }
    
    size_t getHistorySize() {
        return pos_x.size();
    }
    
    float getCurrentTime() {
        return current_time;
    }
    
private:
    // Обновление кватерниона из углов Эйлера
    void updateQuaternionFromEuler(float roll, float pitch, float yaw) {
        float cy = cos(yaw * 0.5f);
        float sy = sin(yaw * 0.5f);
        float cp = cos(pitch * 0.5f);
        float sp = sin(pitch * 0.5f);
        float cr = cos(roll * 0.5f);
        float sr = sin(roll * 0.5f);
        
        q0 = cr * cp * cy + sr * sp * sy;
        q1 = sr * cp * cy - cr * sp * sy;
        q2 = cr * sp * cy + sr * cp * sy;
        q3 = cr * cp * sy - sr * sp * cy;
    }
    
    // Обновление углов ориентации
    void updateAngles(float wx, float wy, float wz) {
        // Сохраняем угловые скорости
        angvel_x.push_back(wx);
        angvel_y.push_back(wy);
        angvel_z.push_back(wz);
        
        // Интегрируем для получения углов Эйлера
        float new_roll = angle_roll.back() + wx * dt;
        float new_pitch = angle_pitch.back() + wy * dt;
        float new_yaw = angle_yaw.back() + wz * dt;
        
        // Нормализация углов
        new_roll = normalizeAngle(new_roll);
        new_pitch = normalizeAngle(new_pitch);
        new_yaw = normalizeAngle(new_yaw);
        
        // Сохраняем углы
        angle_roll.push_back(new_roll);
        angle_pitch.push_back(new_pitch);
        angle_yaw.push_back(new_yaw);
        
        // Обновляем кватернион
        updateQuaternionFromAngularVelocity(wx, wy, wz);
    }
    
    // Обновление кватерниона через угловую скорость
    void updateQuaternionFromAngularVelocity(float wx, float wy, float wz) {
        // Производная кватерниона
        float q0_dot = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
        float q1_dot = 0.5f * ( q0 * wx - q3 * wy + q2 * wz);
        float q2_dot = 0.5f * ( q3 * wx + q0 * wy - q1 * wz);
        float q3_dot = 0.5f * (-q2 * wx + q1 * wy + q0 * wz);
        
        // Интегрирование методом Эйлера
        q0 += q0_dot * dt;
        q1 += q1_dot * dt;
        q2 += q2_dot * dt;
        q3 += q3_dot * dt;
        
        // Нормализация
        float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm > 0.0f) {
            q0 /= norm;
            q1 /= norm;
            q2 /= norm;
            q3 /= norm;
        }
    }
    
    // Преобразование из системы тела в мировую систему
    void bodyToWorld(float bx, float by, float bz,
                     float &wx, float &wy, float &wz) {
        wx = (1 - 2*q2*q2 - 2*q3*q3) * bx +
             2*(q1*q2 - q0*q3) * by +
             2*(q1*q3 + q0*q2) * bz;
        
        wy = 2*(q1*q2 + q0*q3) * bx +
             (1 - 2*q1*q1 - 2*q3*q3) * by +
             2*(q2*q3 - q0*q1) * bz;
        
        wz = 2*(q1*q3 - q0*q2) * bx +
             2*(q2*q3 + q0*q1) * by +
             (1 - 2*q1*q1 - 2*q2*q2) * bz;
    }
    
    // Интегрирование ускорения
    void integrateAcceleration(float ax, float ay, float az) {
        // Метод прямоугольников
        float new_vx = vel_x.back() + ax * dt;
        float new_vy = vel_y.back() + ay * dt;
        float new_vz = vel_z.back() + az * dt;
        
        vel_x.push_back(new_vx);
        vel_y.push_back(new_vy);
        vel_z.push_back(new_vz);
    }
    
    // Интегрирование скорости
    void integrateVelocity() {
        // Метод прямоугольников
        float new_x = pos_x.back() + vel_x.back() * dt;
        float new_y = pos_y.back() + vel_y.back() * dt;
        float new_z = pos_z.back() + vel_z.back() * dt;
        
        pos_x.push_back(new_x);
        pos_y.push_back(new_y);
        pos_z.push_back(new_z);
    }
    
    // Управление размером истории
    void manageHistorySize() {
        if (pos_x.size() > max_history) {
            // Удаляем старые записи
            pos_x.erase(pos_x.begin());
            pos_y.erase(pos_y.begin());
            pos_z.erase(pos_z.begin());
            
            vel_x.erase(vel_x.begin());
            vel_y.erase(vel_y.begin());
            vel_z.erase(vel_z.begin());
            
            accel_body_x.erase(accel_body_x.begin());
            accel_body_y.erase(accel_body_y.begin());
            accel_body_z.erase(accel_body_z.begin());
            
            accel_world_x.erase(accel_world_x.begin());
            accel_world_y.erase(accel_world_y.begin());
            accel_world_z.erase(accel_world_z.begin());
            
            linear_accel_world_x.erase(linear_accel_world_x.begin());
            linear_accel_world_y.erase(linear_accel_world_y.begin());
            linear_accel_world_z.erase(linear_accel_world_z.begin());
            
            angle_roll.erase(angle_roll.begin());
            angle_pitch.erase(angle_pitch.begin());
            angle_yaw.erase(angle_yaw.begin());
            
            angvel_x.erase(angvel_x.begin());
            angvel_y.erase(angvel_y.begin());
            angvel_z.erase(angvel_z.begin());
        }
    }
    
    // Нормализация угла
    float normalizeAngle(float angle) {
        while (angle > PI) angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }
};