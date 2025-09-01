#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, LaserScan
import random
import math
import time

CRITICAL_DISTANCE=0.3
SAFE_DISTANCE = 2
MAX_DISTANCE = 20.0  # Значение для замены бесконечности
#FRONT_VIEW_ANGLES = list(range(5)) + list(range(295, 300))  # Передний сектор зрения
BACK_VIEW_ANGLES = range(100, 200)  # Задний сектор зрения

class AutoTeleop:
    def __init__(self):
        rospy.init_node('auto_teleop')
        self.cmd_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=10)
        # Данные с сенсоров
        self.sensor_data = {
            'front_0': float('inf'),
            'front_1': float('inf'),
            'front_2': float('inf')
        }
        self.lidar_data = []

        # Подписка на топики
        rospy.Subscriber('/myrobot/sensor/front_0', Range, self.update_sensor_data, callback_args='front_0')
        rospy.Subscriber('/myrobot/sensor/front_1', Range, self.update_sensor_data, callback_args='front_1')
        rospy.Subscriber('/myrobot/sensor/front_2', Range, self.update_sensor_data, callback_args='front_2')
        rospy.Subscriber('/myrobot/rplidar/scan', LaserScan, self.update_lidar_data)

        self.rate = rospy.Rate(10)
        self.is_stuck = False  # Флаг, сигнализирующий, что робот застрял
        self.max = None
        self.several_zones = False
        self.turn_around = False
        # self.room = False

    def update_sensor_data(self, data, sensor_name):
        """Обновление данных с ИК-датчиков."""
        self.sensor_data[sensor_name] = data.range

    def update_lidar_data(self, data):
        """Обновление данных с лидара."""
        self.lidar_data = [r if r < MAX_DISTANCE else MAX_DISTANCE for r in data.ranges]

    def find_zones(self):
        """Анализ данных лидара и создание зон."""
        zones = []
        zone_start = None
        front_zone_start = None
        front_zone_end = None

        for i in range(len(self.lidar_data)):
            if i in BACK_VIEW_ANGLES:
                zone_start = None
                continue  # Игнорируем задний сектор

            if len(zones) == 0 and i < 100 and  self.is_drop(i):
                # Возможная зона спереди
                front_zone_end = i
            #if self.is_drop(i) and zone_start

                # Проверяем резкое снижение и последующее повышение расстояния
            if zone_start is None and self.is_rise(i):
                zone_start = i
            elif zone_start is not None and self.is_drop(i):
                # zones.append((zone_start, i))
                # zone_start = None
                if abs(i - zone_start) > 5:
                    zones.append((zone_start, i))
                zone_start = None

            # Если зона захватывает передний сектор зрения
        if zone_start is not None and front_zone_end is not None:
            #zones.append((zone_start, len(self.lidar_data) - 1))
            front_zone_start = zone_start
            zones.append((front_zone_start, front_zone_end))

        return zones

    def is_drop(self, index):
        """Проверка, является ли текущая точка началом зоны (резкое снижение)."""
        prev_index = index - 1 if index > 0 else len(self.lidar_data) - 1
        return self.lidar_data[prev_index] - self.lidar_data[index] > SAFE_DISTANCE

    def is_rise(self, index):
        """Проверка, является ли текущая точка концом зоны (резкое увеличение)."""
        prev_index = index - 1 if index > 0 else len(self.lidar_data) - 1
        return self.lidar_data[index] - self.lidar_data[prev_index] > SAFE_DISTANCE

    def choose_zone(self, zones):
        if not zones:
            return None, None
        if len(zones) > 1:
            self.several_zones = True
        else:
            self.several_zones = False

        max_diff = float('-inf')
        max_diff_indices = None
        min_diff = float('inf')
        min_diff_indices = None

        # Перебор всех кортежей индексов
        for start, end in zones:
            # Извлечение значений по текущим индексам
            if start > end:
                values = self.lidar_data[start:] + self.lidar_data[:end + 1]
            else:
                values = self.lidar_data[start:end + 1]

            # Находжение максимума и минимума среди текущих значений
            current_max = max(values)
            current_min = min(values)

            # Рассчитываем разницу между максимальным и минимальным значениями
            diff = current_max - current_min

            # Обновление максимума разницы и соответствующих индексов, если найдена новая максимальная разница
            if diff > max_diff:
                max_diff = diff
                max_diff_indices = (start, end)

            # Обновление минимума разницы и соответствующих индексов, если найдена новая минимальная разница
            if diff < min_diff:
                min_diff = diff
                min_diff_indices = (start, end)

        # Возвращаем зоны с максимальной и минимальной разницей между максимальным и минимальным значениями
        if self.several_zones and self.max is None:
            if random.choice([True, False]):
                self.max = True
            else:
                self.max = False
        if self.several_zones and self.max is not None:
            if self.max:
                print("theMAX")
                return max_diff_indices
            else:
                print("THEmin")
                return min_diff_indices
        elif not self.several_zones:
            self.max = None
            return min_diff_indices

    def decide_lidar_motion(self):
        """Логика движения на основе лидара."""
        zones = self.find_zones()
        if not zones:
            return None  # Переключаемся на ИК-датчики
        print("zones:")
        print(zones)
        chosen_zone = self.choose_zone(zones)
        print("chosen zone:")
        print(chosen_zone)
        if not chosen_zone:
            return None

        if (chosen_zone[0] < 150 and chosen_zone[-1] < 150) or (chosen_zone[0] > 150 and chosen_zone[-1] > 150):
            center_index = (chosen_zone[0] + chosen_zone[-1]) / 2
        elif (300 - chosen_zone[0]) > chosen_zone[-1]:
            center_index = chosen_zone[0] + (chosen_zone[-1] + (300 - chosen_zone[0])) / 2
        else:
            center_index = (chosen_zone[0] + chosen_zone[-1] - 300) / 2

        print("Going to:")
        print(center_index)

        #center_angle = (chosen_zone[0] + chosen_zone[1]) // 2
        mult = 1
        twist = Twist()
        if center_index <= 2 or center_index >= 297:
            # Move forward if the center_index approximatelly equall to the current direction
            rospy.loginfo("Lidar: Move forward!")
            twist.linear.x = 1.0
            twist.angular.z = 0.0
        else:
            if center_index >= 150:
                print("Lidar: Right!")
                mult = -1
            else:
                print("Lidar: Left!")
            twist.linear.x = 0.5
            #twist.angular.z = -0.01 * (center_angle - 150)  # Поворачиваем в сторону центра зоны
            twist.angular.z = 0.8 * mult
        return twist

    def turn(self, twist):
        if not (self.sensor_data['front_0'] > CRITICAL_DISTANCE and \
           self.sensor_data['front_1'] > CRITICAL_DISTANCE and \
           self.sensor_data['front_2'] > CRITICAL_DISTANCE):
            twist.linear.x = 0.0
            twist.angular.z = 0.6
        else:
            self.turn_around = False
        return twist

    def decide_ir_motion(self):
        """Логика движения на основе ИК-датчиков."""
        twist = Twist()

        if self.turn_around:
            self.turn(twist)
            
        elif (self.sensor_data['front_0'] < CRITICAL_DISTANCE and \
           self.sensor_data['front_1'] < CRITICAL_DISTANCE and \
           self.sensor_data['front_2'] < CRITICAL_DISTANCE):
            # Застревание в углу
            #twist.linear.x = -0.3
            #twist.angular.z = 0.6
            self.turn_around = True
        elif self.sensor_data['front_0'] < CRITICAL_DISTANCE:
            twist.linear.x = -0.05
            twist.angular.z = 0.5 if self.sensor_data['front_1'] > self.sensor_data['front_2'] else -0.5
        elif self.sensor_data['front_1'] < CRITICAL_DISTANCE:
            twist.linear.x = -0.05
            twist.angular.z = -0.4
        elif self.sensor_data['front_2'] < CRITICAL_DISTANCE:
            twist.linear.x = -0.05
            twist.angular.z = 0.4
        else:
            twist.linear.x = 0.8
            twist.angular.z = 0.0

        return twist


    def run(self):
        # self.room = False
        while not rospy.is_shutdown():
            twist = None
            # print("several zones")
            # print(self.several_zones)
            # print()
            # print("max")
            # print(self.max)
            # Проверка на управление ИК-датчиками
            if any(self.sensor_data[sensor] < CRITICAL_DISTANCE for sensor in self.sensor_data):
                twist = self.decide_ir_motion()

            # Если ИК-датчики свободны, используем лидар
            if twist is None:
                twist = self.decide_lidar_motion()
            if twist is None:
                # self.room = True
                twist = self.decide_ir_motion()
            # print(twist)
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        auto_teleop = AutoTeleop()
        auto_teleop.run()
    except rospy.ROSInterruptException:
        pass
