'''two path select'''
import can
from math import radians, cos, sin, asin, sqrt, degrees, atan2
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
import math
# import cv2
import csv
from pyproj import Proj
import matplotlib.pyplot as plt
# from perception.yolov8_detect import CameraObjectDetector
from can_use import Can_use, ISGSpeedFilter

import pyproj
import time 

import logging
import datetime

timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
log_file_name = f"./run_log/{timestamp}.log"

logging.basicConfig(
    filename=log_file_name,         # 日志输出到当前目录下的 <时间戳>.log 文件
    level=logging.INFO,             # 日志级别：INFO 及以上
    format="%(asctime)s - %(levelname)s - %(message)s"
)

# 车辆参数
VEHICLE_WIDTH = 1.9   # m
VEHICLE_LENGTH = 4.5  # m
WHEEL_FACTOR = 7.2
manual_triggered = False
stop_record = False
mod_666 = 0
mod_AE = 0

lonlat2xy_old = Proj('+proj=tmerc +lon_0=118.8170043 +lat_0=31.8926311 +ellps=WGS84')
def read_csv(csv_file_path):
    traj_data = []
    x = []
    y = []
    # 打开CSV文件并读取内容  
    with open(csv_file_path, mode='r', newline='') as file:  
        csv_reader = csv.reader(file)  
        
        # 跳过标题行（如果有的话）  
        headers = next(csv_reader, None)  # 这行代码会读取第一行，如果第一行是标题则跳过  
        
        # 读取每一行数据并添加到列表中  
        for row in csv_reader:  
            # 将每一行的数据转换为整数或浮点数（根据具体情况选择）  
            # 假设x坐标、y坐标、航向角和速度都是浮点数  
            x_coord = float(row[0])  
            y_coord = float(row[1])
            x_coord_utm, y_coord_utm = lonlat2xy_old(x_coord, y_coord, inverse=False)  
            heading = float(row[2])  
            x.append(x_coord_utm)
            y.append(y_coord_utm)
            # 将这些信息存储为一个列表，并添加到data_list中  
            data_row = [x_coord, y_coord, heading]  
            traj_data.append(data_row)
    plt.scatter(x , y)
    plt.scatter([x[-1]], [y[-1]], color="red") # end
    plt.scatter(12.27, -3.27, color="brown")
    plt.scatter([x[0]], [y[0]], color="black") # start
    plt.title('reference_trajectory_utm')  
    plt.xlabel('longitudinal')  
    plt.ylabel('latitudinal')
    plt.savefig('ref_traj_utm.png')
    return traj_data


# 计算轨迹点与障碍物的距离
def calculate_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

class VehicleTrajectoryFollower:
    def __init__(self, target_index=0):
        """
        初始化，读取主轨迹和备选轨迹点
        :param main_trajectory_csv: 包含主轨迹点的CSV文件路径
        :param alternate_trajectory_csv: 包含备选轨迹点的CSV文件路径
        """
        self.current_trajectory = None
        self.is_using_alternate = False  # 标志当前是否在使用备选轨迹
        self.main_closest_index = 0
        self.alternate_closest_index = 0
        self.wheelbase = 3.5
        self.offset_target_index = 3
        self.target_index = 0
        self.should_stop = False  # 增加停车标志位
        self.obstacle_detected = False  # 标记是否检测到障碍物
        self.previous_turn_angle = 0
        self.far_previous_turn_angle = 0
        self.max_turn_rate = 6  # 限制每次转向角的最大变化速率（度）
        self.far_index = 25  # 远处的目标点，用于控制速度
        self.control_speed_the = 30 #用於判斷遠處目標點和當前head的差值是否超過該值，然後進行速度的處理
    
    

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        return compass_bearing

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # 地球半径，单位为米
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = math.sin(dLat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance
    
    def find_closest_point_index_bank(self, current_lat, current_lon, trajectory=None):
        """
        找到距离当前车辆位置最近的轨迹点索引
        :param current_lat: 车辆当前纬度
        :param current_lon: 车辆当前经度
        :param trajectory: 要查找的轨迹，默认使用当前轨迹
        :return: 距离最近的轨迹点索引
        """
        if trajectory is None:
            trajectory = self.current_trajectory
        closest_index = 0
        min_distance = float('inf')
        if self.closest_index == 0:
            max_bound = len(trajectory)-1
        else:
            max_bound = 200
        for i, (lon, lat, _) in enumerate(trajectory[self.closest_index:self.closest_index+max_bound]):  # 经度在前，纬度在后
            distance = self.calculate_distance(current_lat, current_lon, lat, lon)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        return closest_index+self.closest_index

    
    def find_closest_point_index(self, current_lat, current_lon, trajectory=None,is_main=True):
        """
        找到距离当前车辆位置最近的轨迹点索引
        :param current_lat: 车辆当前纬度
        :param current_lon: 车辆当前经度
        :param trajectory: 要查找的轨迹，默认使用当前轨迹
        :return: 距离最近的轨迹点索引
        """
        if trajectory is None:
            trajectory = self.current_trajectory
        closest_index_temp = 0
        min_distance = float('inf')
        
        if is_main:
            closest_index = self.main_closest_index
        else:
            closest_index = self.alternate_closest_index
        
        if closest_index == 0:
            max_bound = len(trajectory)-1
        else:
            max_bound = 200
        print(max(closest_index,0),min(closest_index+max_bound, len(trajectory)-1))
        for i, (lon, lat, _) in enumerate(trajectory[closest_index:max(closest_index+max_bound,len(trajectory))]):  # 经度在前，纬度在后
            distance = self.calculate_distance(current_lat, current_lon, lat, lon)
            if distance < min_distance:
                min_distance = distance
                closest_index_temp = i
                
        return closest_index_temp+closest_index
    
    
    def adjust_position_to_front_axle(self, rear_lat, rear_lon, heading):
        """
        根据后轴中心的经纬度和heading计算前轴的经纬度
        :param rear_lat: 后轴的纬度
        :param rear_lon: 后轴的经度
        :param heading: 车辆的航向角，相对于正北方向
        :return: 前轴的经纬度 (lat, lon)
        """
        # 先将heading转换为弧度
        heading_rad = math.radians(heading)

        # 计算纬度上的变化，假设1度纬度大约为111,320米
        delta_lat = (self.wheelbase / 6371000) * math.cos(heading_rad)

        # 计算经度上的变化，假设经度的变化随着纬度而变化，纬度越高，1度经度的实际距离越小
        delta_lon = (self.wheelbase / 6371000) * math.sin(heading_rad) / math.cos(math.radians(rear_lat))

        # 计算前轴的经纬度
        front_lat = rear_lat + math.degrees(delta_lat)
        front_lon = rear_lon + math.degrees(delta_lon)

        return front_lat, front_lon
    
    # 用于平滑转角
    def smooth_turn_angle(self, turn_angle):
        # 限制转向角的最大变化速率
        angle_difference = turn_angle - self.previous_turn_angle
        if angle_difference > self.max_turn_rate:
            update_turn_angle = self.previous_turn_angle + 4
        elif angle_difference < -self.max_turn_rate:
            update_turn_angle = self.previous_turn_angle - 4
        else:
            update_turn_angle = turn_angle
        
    

        # 更新上一次的转向角
        self.previous_turn_angle = update_turn_angle
        return turn_angle

    def calculate_turn_angle(self, current_position, current_heading,offset_target_index = None):
        if  self.current_trajectory == None:
            return 'no_current_trajectory'
        
        if offset_target_index is not None:
            target_index_obstacle = offset_target_index
        else:
            target_index_obstacle = self.offset_target_index
        print("==============", target_index_obstacle)
        current_lat, current_lon, _ = current_position
        # 根据后轴的位置和heading调整得到前轴的位置
        front_lat, front_lon = self.adjust_position_to_front_axle(current_lat, current_lon, current_heading)
        
        # 找到距离最近的点的索引
        self.closest_index = self.find_closest_point_index(front_lat, front_lon)
        
        target_index = min(self.closest_index + target_index_obstacle, len(self.current_trajectory) - 1)  # 防止超出范围
        self.target_index = target_index
        next_lon, next_lat, _ = self.current_trajectory[target_index]  # 注意经纬度顺序
        
        # 计算目标点相对当前位置的方位角
        desired_heading = self.calculate_bearing(current_lat, current_lon, next_lat, next_lon)
        # 计算转向角
        turn_angle = (desired_heading - current_heading + 360) % 360
        if turn_angle > 180:
            turn_angle -= 360        
        # 映射到方向盘转角
        if turn_angle * WHEEL_FACTOR > 460:
            turn_angle = 460
        elif turn_angle * WHEEL_FACTOR < -460:
            turn_angle = -460
        else:
            turn_angle = turn_angle * WHEEL_FACTOR    
        
        turn_angle = self.smooth_turn_angle(turn_angle)
        
        return turn_angle

    # 计算期望速度和加速度
    def calculate_speedAndacc(self, turn_angle, current_position, current_speed, is_obstacle = False, points_num_threshold=20):
        if current_speed < 1:
            speed = 20
            acc = 0
            return speed, acc
        if abs(turn_angle) >= 50:
            if current_speed >= 15:
                speed = 10
                acc = -2
            else:
                speed = 10
                acc = 0
            return speed, acc 
        
        current_lat, current_lon, current_heading = current_position
        next_lon, next_lat, _ = self.current_trajectory[min(self.closest_index + self.far_index, len(self.current_trajectory) - 1)]  # 注意经纬度顺序
        
        # 计算目标点相对当前位置的方位角
        far_desired_heading = self.calculate_bearing(current_lat, current_lon, next_lat, next_lon)
        # 计算转向角
        far_turn_angle = (far_desired_heading - current_heading + 360) % 360
        if far_turn_angle > 180:
            far_turn_angle -= 360        
        # 映射到方向盘转角
        if far_turn_angle * WHEEL_FACTOR > 460:
            far_turn_angle = 460
        elif far_turn_angle * WHEEL_FACTOR < -460:
            far_turn_angle = -460
        else:
            far_turn_angle = far_turn_angle * WHEEL_FACTOR    

        if abs(far_turn_angle) >= 40:
            print("=_="*30)
            if current_speed >= 15:
                speed = 10
                acc = -1
            else:
                speed = 10
                acc = 0
        else:
            speed = 20
            acc = 0
        
        if is_obstacle:
            print("find obstacle reduce speed")
            if current_speed >= 15:
                speed = 10
                acc = -3
            else:
                speed = 10
                acc = 0
        return speed, acc    



class Can_use:
    def __init__(self, zone):
        self.bus_ins = can.interface.Bus(channel='can0', bustype='socketcan')
        self.bus_vcu = can.interface.Bus(channel='can1', bustype='socketcan')
        self.ego_lon = 31.8925019
        self.ego_lat = 118.8171577
        self.ego_yaw = 270
        self.ego_v = 3
        self.ego_a = 0
        self.eps_mode = 2
        self.auto_driver_allowed = False

    def read_ins_info(self):
        """获取惯导的主车信息"""
        message_ins = self.bus_ins.recv()
        message_vcu = self.bus_vcu.recv()
        if message_ins is not None and message_ins.arbitration_id == 0x504:
            # 直接获取数据字节
            can_data = message_ins.data
            # 解析前4个字节为纬度
            INS_Latitude = (can_data[0] << 24) | (can_data[1] << 16) | (can_data[2] << 8) | can_data[3]
            # 解析后4个字节为经度
            INS_Longitude = (can_data[4] << 24) | (can_data[5] << 16) | (can_data[6] << 8) | can_data[7]
            INS_Latitude = INS_Latitude * 0.0000001 - 180
            INS_Longitude = INS_Longitude * 0.0000001 - 180

            ego_x = INS_Longitude
            ego_y = INS_Latitude
            self.ego_lon = ego_x
            self.ego_lat = ego_y
            logging.info(f"ego_x:{ego_x},ego_y:{ego_y}")

        if message_ins is not None and message_ins.arbitration_id == 0x505:
            speed_data = message_ins.data
            # 北向速度
            INS_NorthSpd =  (speed_data[0] << 8) | speed_data[1]
            INS_NorthSpd =   INS_NorthSpd * 0.0030517 - 100    # m/s
            INS_NorthSpd *= 3.6
            # 东向速度
            INS_EastSpd =  (speed_data[2] << 8) | speed_data[3]
            INS_EastSpd =   INS_EastSpd * 0.0030517 - 100    # m/s
            INS_EastSpd *= 3.6
            # 地向速度
            INS_ToGroundSpd =  (speed_data[4] << 8) | speed_data[5]
            INS_ToGroundSpd =   INS_ToGroundSpd * 0.0030517 - 100    # m/s
            INS_ToGroundSpd *= 3.6
                    
            speed =  sqrt(INS_EastSpd**2 + INS_NorthSpd**2 + INS_ToGroundSpd**2)
                    
            self.ego_v = speed

        if message_ins is not None and message_ins.arbitration_id == 0x502:
            # self.ego_yaw = angle
            Angle_data = message_ins.data
            HeadingAngle =  (Angle_data[4] << 8) | Angle_data[5]
            HeadingAngle = HeadingAngle * 0.010986 - 360
            self.ego_yaw = HeadingAngle 
        if message_ins is not None and message_ins.arbitration_id == 0x500:
            acc_data = message_ins.data
            # 北向速度
            ACC_X =  (acc_data[0] << 8) | acc_data[1]
            ACC_X =   (ACC_X * 0.0001220703125 - 4) * 9.8   # g
            self.ego_a = ACC_X
        
        if message_vcu is not None and message_vcu.arbitration_id == 0x15C:
            allow_value = message_vcu.data[2] & 0x01
            self.auto_driver_allowed = (allow_value == 1)

        if message_vcu is not None and message_vcu.arbitration_id == 0x124:
            eps_mode = (message_vcu.data[6] >> 4) & 0x03
            self.eps_mode = eps_mode

    def publish_planner_action(self, action, id, action_type, mod, enable):
        """将规划动作发布到CAN"""

        if action_type == "angle":    
            # 数据缩放和转换
            data1 = int((action - (-738)) / 0.1)  # 确保data1根据传入angle正确计算
            data1_high = (data1 >> 8) & 0xFF    # data1的高8位
            data1_low = data1 & 0xFF            # data1的低8位

            data2 = int(mod) & 0x03             # data2缩放到2位范围，0-3
            data3 = int(250 / 10) & 0xFF     # data3缩放到8位范围，0-255, angle_spd=100
            data4 = int(enable) & 0x01          # data4缩放到1位范围，0或1
                
            # 构建发送数据，确保8字节长度
            data = [data1_high, data1_low, data2, data3, data4, 0, 0, 0]

            msg = can.Message(arbitration_id=id, data=data, is_extended_id=False)
            self.bus_vcu.send(msg)
        
        if action_type == "acc":
            auto_drive_cmd_bits = mod & 0x07  # 取最低3位
            # Auto speed cmd（位3-7）
            # 首先对速度进行缩放和偏移
            # 期望速度 单位km/h
            # desired_speed = action[0] 
            desired_speed = 3
            speed_scaled = int(desired_speed) & 0x1F  # 取5位（位3-7）
            # 组合BYTE0
            byte0 = (speed_scaled << 3) | auto_drive_cmd_bits

            # BYTE1-BYTE2（需求方向盘转角）
            # 需要根据具体缩放因子和偏移量进行计算，假设缩放因子为0.1，偏移量为0
            # action[1] = 396
            logging.info(f"final turn angle:{action[1]}")
            angle_scaled = int((action[1] - (-500)) / 0.1) & 0xFFFF  # 16位
            byte1 = (angle_scaled >> 8) & 0xFF  # 高8位
            byte2 = angle_scaled & 0xFF         # 低8位

            # BYTE3（需求制动减速度）
            # 进行缩放和偏移
            acc  =  action[2]
            # acc = 0
            acc_scaled = int((acc - (-4)) / 1) & 0xFF  # 假设缩放因子1，偏移量-4

            # 构建发送数据，剩余字节填充0
            data_666 = [byte0, byte1, byte2, acc_scaled, 0, 0, 0, 0]
            
            msg = can.Message(arbitration_id=id, data=data_666, is_extended_id=False)
            # 发送CAN消息
            self.bus_vcu.send(msg)
            # time.sleep(0.01)
  

