#pragma once

#include<vector>
// Структура для возврата средних отфильтрованных значений
struct FilteredAverages {
    float Acl_Xavg;
    float Acl_Yavg; 
    float Acl_Zavg;
    float Gyr_Xavg;
    float Gyr_Yavg;
    float Gyr_Zavg;
};

// Вспомогательная функция для медианной фильтрации окна
float medianFilter(const float* data, int size, int window_size, int center_idx) {
    // Определяем границы окна
    int half_window = window_size / 2;
    int start_idx = max(0, center_idx - half_window);
    int end_idx = min(size - 1, center_idx + half_window);
    int window_count = end_idx - start_idx + 1;
    
    // Копируем данные окна в массив
    float window[15]; // Максимум 15 точек в окне
    int count = 0;
    for (int i = start_idx; i <= end_idx; i++) {
        window[count++] = data[i];
    }
    
    // Сортировка пузырьком
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (window[i] > window[j]) {
                float temp = window[i];
                window[i] = window[j];
                window[j] = temp;
            }
        }
    }
    
    // Возвращаем медиану
    if (count % 2 == 0) {
        return (window[count/2 - 1] + window[count/2]) / 2.0f;
    } else {
        return window[count/2];
    }
}

// Вспомогательная функция для фильтра низких частот окна
float lowPassFilterWindow(const float* data, int size, float alpha) {
    if (size == 0) return 0;
    
    float filtered = data[0];
    for (int i = 1; i < size; i++) {
        filtered = (1.0f - alpha) * filtered + alpha * data[i];
    }
    return filtered;
}

// Основная функция фильтрации
FilteredAverages all_filt(
    const float* Acl_X, const float* Acl_Y, const float* Acl_Z,
    const float* Gyr_X, const float* Gyr_Y, const float* Gyr_Z,
    int data_size
) {
    FilteredAverages result = {0, 0, 0, 0, 0, 0};
    
    if (data_size <= 0) return result;
    
    // 1. ПОДГОТОВКА ДАННЫХ В МАССИВЫ
    float accel_X[data_size];
    float accel_Y[data_size];
    float accel_Z[data_size];
    float gyro_X[data_size];
    float gyro_Y[data_size];
    float gyro_Z[data_size];
    
    // Копируем данные
    for (int i = 0; i < data_size; i++) {
        accel_X[i] = Acl_X[i];
        accel_Y[i] = Acl_Y[i];
        accel_Z[i] = Acl_Z[i];
        gyro_X[i] = Gyr_X[i];
        gyro_Y[i] = Gyr_Y[i];
        gyro_Z[i] = Gyr_Z[i];
    }
    
    // ПАРАМЕТРЫ ФИЛЬТРОВ
    const int MEDIAN_WINDOW_ACCEL = min(5, data_size);  // Окно медианного фильтра для акселерометра
    const int MEDIAN_WINDOW_GYRO = min(3, data_size);   // Окно медианного фильтра для гироскопа
    const float ALPHA_ACCEL = 0.3f;  // Коэффициент сглаживания для акселерометра
    const float ALPHA_GYRO = 0.5f;   // Коэффициент сглаживания для гироскопа
    
    // 2. ФИЛЬТРАЦИЯ АКСЕЛЕРОМЕТРА
    
    // Этап 1: Медианная фильтрация (удаление выбросов)
    float accel_X_med[data_size];
    float accel_Y_med[data_size];
    float accel_Z_med[data_size];
    
    for (int i = 0; i < data_size; i++) {
        accel_X_med[i] = medianFilter(accel_X, data_size, MEDIAN_WINDOW_ACCEL, i);
        accel_Y_med[i] = medianFilter(accel_Y, data_size, MEDIAN_WINDOW_ACCEL, i);
        accel_Z_med[i] = medianFilter(accel_Z, data_size, MEDIAN_WINDOW_ACCEL, i);
    }
    
    // Этап 2: Фильтр низких частот
    float accel_X_lpf = lowPassFilterWindow(accel_X_med, data_size, ALPHA_ACCEL);
    float accel_Y_lpf = lowPassFilterWindow(accel_Y_med, data_size, ALPHA_ACCEL);
    float accel_Z_lpf = lowPassFilterWindow(accel_Z_med, data_size, ALPHA_ACCEL);
    
    // Этап 3: Скользящее среднее (финальное сглаживание)
    float sum_accel_X = 0, sum_accel_Y = 0, sum_accel_Z = 0;
    for (int i = 0; i < data_size; i++) {
        sum_accel_X += accel_X_med[i];
        sum_accel_Y += accel_Y_med[i];
        sum_accel_Z += accel_Z_med[i];
    }
    
    result.Acl_Xavg = (accel_X_lpf + sum_accel_X / data_size) / 2.0f;
    result.Acl_Yavg = (accel_Y_lpf + sum_accel_Y / data_size) / 2.0f;
    result.Acl_Zavg = (accel_Z_lpf + sum_accel_Z / data_size) / 2.0f;
    
    // 3. ФИЛЬТРАЦИЯ ГИРОСКОПА
    
    // Этап 1: Медианная фильтрация (удаление выбросов)
    float gyro_X_med[data_size];
    float gyro_Y_med[data_size];
    float gyro_Z_med[data_size];
    
    for (int i = 0; i < data_size; i++) {
        gyro_X_med[i] = medianFilter(gyro_X, data_size, MEDIAN_WINDOW_GYRO, i);
        gyro_Y_med[i] = medianFilter(gyro_Y, data_size, MEDIAN_WINDOW_GYRO, i);
        gyro_Z_med[i] = medianFilter(gyro_Z, data_size, MEDIAN_WINDOW_GYRO, i);
    }
    
    // Этап 2: Фильтр низких частот
    float gyro_X_lpf = lowPassFilterWindow(gyro_X_med, data_size, ALPHA_GYRO);
    float gyro_Y_lpf = lowPassFilterWindow(gyro_Y_med, data_size, ALPHA_GYRO);
    float gyro_Z_lpf = lowPassFilterWindow(gyro_Z_med, data_size, ALPHA_GYRO);
    
    // Этап 3: Компенсация дрейфа (вычитание среднего)
    float sum_gyro_X = 0, sum_gyro_Y = 0, sum_gyro_Z = 0;
    for (int i = 0; i < data_size; i++) {
        sum_gyro_X += gyro_X_med[i];
        sum_gyro_Y += gyro_Y_med[i];
        sum_gyro_Z += gyro_Z_med[i];
    }
    
    result.Gyr_Xavg = gyro_X_lpf - (sum_gyro_X / data_size) * 0.1f; // Частичная компенсация дрейфа
    result.Gyr_Yavg = gyro_Y_lpf - (sum_gyro_Y / data_size) * 0.1f;
    result.Gyr_Zavg = gyro_Z_lpf - (sum_gyro_Z / data_size) * 0.1f;
    
    return result;
}

// Упрощенная версия для использования с динамическими массивами
FilteredAverages all_filt_simple(
    const float Acl_X[], const float Acl_Y[], const float Acl_Z[],
    const float Gyr_X[], const float Gyr_Y[], const float Gyr_Z[],
    int data_size
) {
    return all_filt(Acl_X, Acl_Y, Acl_Z, Gyr_X, Gyr_Y, Gyr_Z, data_size);
}