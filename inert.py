import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Чтение данных из файла
try:
    df = pd.read_csv(r"C:\Users\LaL\Desktop\input.txt")
    print(f"Успешно загружено {len(df)} строк из файла input.csv")
    print(f"Колонки: {list(df.columns)}")
except FileNotFoundError:
    print("Ошибка: файл 'input.csv' не найден!")
    print("Проверьте, что файл находится в той же папке, что и скрипт.")
    exit()
except Exception as e:
    print(f"Ошибка при чтении файла: {e}")
    exit()

# Время (10 мс между строками)
time = np.arange(len(df)) * 0.01

# Создаем фигуру с 15 подграфиками
plt.figure(figsize=(16, 10))

# 1) Графики угловой скорости (Wx, Wy, Wz)
plt.subplot(5, 3, 1)
plt.plot(time, df['Wx'], 'r-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Угловая скорость Wx', fontsize=10, fontweight='bold')
plt.ylabel('рад/с', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.subplot(5, 3, 2)
plt.plot(time, df['Wy'], 'g-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Угловая скорость Wy', fontsize=10, fontweight='bold')
plt.ylabel('рад/с', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.subplot(5, 3, 3)
plt.plot(time, df['Wz'], 'b-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Угловая скорость Wz', fontsize=10, fontweight='bold')
plt.ylabel('рад/с', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

# 2) Графики угловых координат (Roll, Pitch, Yaw)
plt.subplot(5, 3, 4)
plt.plot(time, df['Roll'], 'r-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Roll (крен)', fontsize=10, fontweight='bold')
plt.ylabel('рад', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.subplot(5, 3, 5)
plt.plot(time, df['Pitch'], 'g-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Pitch (тангаж)', fontsize=10, fontweight='bold')
plt.ylabel('рад', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.subplot(5, 3, 6)
plt.plot(time, df['Yaw'], 'b-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Yaw (рыскание)', fontsize=10, fontweight='bold')
plt.ylabel('рад', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

# 3) Графики ускорений (Ax, Ay, Az)
plt.subplot(5, 3, 7)
plt.plot(time, df['Ax'], 'r-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Ускорение Ax', fontsize=10, fontweight='bold')
plt.ylabel('м/с²', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.subplot(5, 3, 8)
plt.plot(time, df['Ay'], 'g-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Ускорение Ay', fontsize=10, fontweight='bold')
plt.ylabel('м/с²', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.subplot(5, 3, 9)
plt.plot(time, df['Az'], 'b-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Ускорение Az', fontsize=10, fontweight='bold')
plt.ylabel('м/с²', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

# 4) Графики скоростей (Vx, Vy, Vz)
plt.subplot(5, 3, 10)
plt.plot(time, df['Vx'], 'r-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Скорость Vx', fontsize=10, fontweight='bold')
plt.ylabel('м/с', fontsize=9)
plt.xlabel('Время (с)', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.subplot(5, 3, 11)
plt.plot(time, df['Vy'], 'g-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Скорость Vy', fontsize=10, fontweight='bold')
plt.ylabel('м/с', fontsize=9)
plt.xlabel('Время (с)', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.subplot(5, 3, 12)
plt.plot(time, df['Vz'], 'b-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Скорость Vz', fontsize=10, fontweight='bold')
plt.ylabel('м/с', fontsize=9)
plt.xlabel('Время (с)', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

# 5) Графики перемещений (X, Y, Z)
plt.subplot(5, 3, 13)
plt.plot(time, df['X'], 'r-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Перемещение X', fontsize=10, fontweight='bold')
plt.ylabel('м', fontsize=9)
plt.xlabel('Время (с)', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.subplot(5, 3, 14)
plt.plot(time, df['Y'], 'g-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Перемещение Y', fontsize=10, fontweight='bold')
plt.ylabel('м', fontsize=9)
plt.xlabel('Время (с)', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.subplot(5, 3, 15)
plt.plot(time, df['Z'], 'b-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Перемещение Z', fontsize=10, fontweight='bold')
plt.ylabel('м', fontsize=9)
plt.xlabel('Время (с)', fontsize=9)
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)

plt.suptitle('АНАЛИЗ ДАННЫХ ДАТЧИКОВ (развертка по осям)', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.subplots_adjust(top=0.95)

# Сохраняем первый график
plt.savefig('sensor_data_components.png', dpi=150, bbox_inches='tight')
plt.show()

# 6) Отдельная фигура: модули
plt.figure(figsize=(14, 4))

# Модуль ускорения
mod_A = np.sqrt(df['Ax']**2 + df['Ay']**2 + df['Az']**2)
plt.subplot(1, 3, 1)
plt.plot(time, mod_A, 'k-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Модуль ускорения |A|', fontsize=12, fontweight='bold')
plt.ylabel('м/с²', fontsize=11)
plt.xlabel('Время (с)', fontsize=11)
plt.xticks(fontsize=10)
plt.yticks(fontsize=10)

# Модуль скорости
mod_V = np.sqrt(df['Vx']**2 + df['Vy']**2 + df['Vz']**2)
plt.subplot(1, 3, 2)
plt.plot(time, mod_V, 'k-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Модуль скорости |V|', fontsize=12, fontweight='bold')
plt.ylabel('м/с', fontsize=11)
plt.xlabel('Время (с)', fontsize=11)
plt.xticks(fontsize=10)
plt.yticks(fontsize=10)

# Модуль перемещения
mod_D = np.sqrt(df['X']**2 + df['Y']**2 + df['Z']**2)
plt.subplot(1, 3, 3)
plt.plot(time, mod_D, 'k-', linewidth=2)
plt.grid(True, alpha=0.3)
plt.title('Модуль перемещения |D|', fontsize=12, fontweight='bold')
plt.ylabel('м', fontsize=11)
plt.xlabel('Время (с)', fontsize=11)
plt.xticks(fontsize=10)
plt.yticks(fontsize=10)

plt.suptitle('МОДУЛИ ВЕКТОРОВ', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.subplots_adjust(top=0.85)

# Сохраняем второй график
plt.savefig('sensor_data_magnitudes.png', dpi=150, bbox_inches='tight')
plt.show()

# Вывод статистики
print("\n" + "="*60)
print("СТАТИСТИКА ДАННЫХ:")
print("="*60)

print("\nУгловые скорости (рад/с):")
print(f"  Wx: мин={df['Wx'].min():.4f}, макс={df['Wx'].max():.4f}, ср={df['Wx'].mean():.4f}")
print(f"  Wy: мин={df['Wy'].min():.4f}, макс={df['Wy'].max():.4f}, ср={df['Wy'].mean():.4f}")
print(f"  Wz: мин={df['Wz'].min():.4f}, макс={df['Wz'].max():.4f}, ср={df['Wz'].mean():.4f}")

print("\nУглы (рад):")
print(f"  Roll:  мин={df['Roll'].min():.4f}, макс={df['Roll'].max():.4f}, ср={df['Roll'].mean():.4f}")
print(f"  Pitch: мин={df['Pitch'].min():.4f}, макс={df['Pitch'].max():.4f}, ср={df['Pitch'].mean():.4f}")
print(f"  Yaw:   мин={df['Yaw'].min():.4f}, макс={df['Yaw'].max():.4f}, ср={df['Yaw'].mean():.4f}")

print("\nУскорения (м/с²):")
print(f"  Ax: мин={df['Ax'].min():.3f}, макс={df['Ax'].max():.3f}, ср={df['Ax'].mean():.3f}")
print(f"  Ay: мин={df['Ay'].min():.3f}, макс={df['Ay'].max():.3f}, ср={df['Ay'].mean():.3f}")
print(f"  Az: мин={df['Az'].min():.3f}, макс={df['Az'].max():.3f}, ср={df['Az'].mean():.3f}")

print("\nСкорости (м/с):")
print(f"  Vx: мин={df['Vx'].min():.3f}, макс={df['Vx'].max():.3f}, ср={df['Vx'].mean():.3f}")
print(f"  Vy: мин={df['Vy'].min():.3f}, макс={df['Vy'].max():.3f}, ср={df['Vy'].mean():.3f}")
print(f"  Vz: мин={df['Vz'].min():.3f}, макс={df['Vz'].max():.3f}, ср={df['Vz'].mean():.3f}")

print("\nПеремещения (м):")
print(f"  X: мин={df['X'].min():.4f}, макс={df['X'].max():.4f}, ср={df['X'].mean():.4f}")
print(f"  Y: мин={df['Y'].min():.4f}, макс={df['Y'].max():.4f}, ср={df['Y'].mean():.4f}")
print(f"  Z: мин={df['Z'].min():.4f}, макс={df['Z'].max():.4f}, ср={df['Z'].mean():.4f}")

print(f"\nОбщее время записи: {time[-1]:.3f} с")
print(f"Частота дискретизации: {1/0.01:.0f} Гц")
print(f"\nГрафики сохранены как:")
print(f"  1. sensor_data_components.png")
print(f"  2. sensor_data_magnitudes.png")