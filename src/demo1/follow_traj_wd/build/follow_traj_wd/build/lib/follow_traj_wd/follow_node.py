import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
import time
import sys
sys.path.append('/home/nvidia/vcii/hezi_ros2/src/demo1/follow_traj_wd/follow_traj_wd')
from follow_demo_2025 import VehicleTrajectoryFollower
from can_use import ISGSpeedFilter
from can_use import Can_use
import logging
from geometry_msgs.msg import PoseArray
import math
import csv  
from pyproj import Proj

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
    return traj_data  


class FollowNode(Node):
    def __init__(self, main_trajectory_csv, alternate_trajectory_csv):
        super().__init__('Follow_node')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            'planner_action', 
            1
        )
        self.vs_subscription = self.create_subscription(
            Float32MultiArray,
            'vehicle_state',
            self.vs_callback,
            1
        )
        self.eps_subscription = self.create_subscription(
            Int32,
            'eps_mode',
            self.eps_callback,
            1
        )
        
        # 订阅到轨迹
        self.trajectory = self.create_subscription(
            PoseArray,
            'trajectory',
            self.trajectory_callback,
            10
        )
        # 用于存储转换后的轨迹点数据 [[x, y, heading], ...]
        self.traj_data = []
        
        self.manual_triggered = True
        self.mode_AE = 1
        self.mode_666 = 1
        self.eps_subscription
        self.vs_subscription  # prevent unused variable warning
        
        
        # 读取 CSV 初始轨迹
        csv_file_path = '/home/nvidia/vcii/follow_trajectory/collect_trajectory/processed_lane_change_right_0404.csv'
        self.main_traj_data = read_csv(csv_file_path)

        csv_file_path = '/home/nvidia/vcii/follow_trajectory/collect_trajectory/processed_lane_change_right_0404.csv'
        self.second_traj_data = read_csv(csv_file_path)


        self.follower = VehicleTrajectoryFollower()
        self.filter = ISGSpeedFilter()
        self.latest_eps_mode = None
        self.can_use = Can_use()
        
    def eps_callback(self, msg):
        self.latest_eps_mode = msg.data
        
    def vs_callback(self, msg):
        if self.latest_eps_mode is None:
            self.get_logger().warn("尚未接收到eps_mode，跳过一次控制")
            return
        self.get_logger().info(f"[vs_callback] Received state: {msg.data}")
        self.get_logger().info(f"[vs_callback] EPS mode: {self.latest_eps_mode}")
        eps_mode = self.latest_eps_mode
        start = time.time()
        for i in range(20):
            self.can_use.read_ins_info()
        # ego_lat = msg.data[0]
        # ego_lon = msg.data[1]
        # ego_yaw = msg.data[2]
        # ego_v = msg.data[3]
        ego_lat = self.can_use.ego_lat
        ego_lon = self.can_use.ego_lon
        ego_yaw = self.can_use.ego_yaw
        ego_v   = self.can_use.ego_v
        if eps_mode != 3 and self.manual_triggered:
            self.mode_AE = 1
            self.mode_666 = 1
        if eps_mode ==3:
            self.mode_AE = 3
            self.mode_666 = 0
            self.manual_triggered = False
        if self.mode_AE == 1 and self.mode_666 == 1:
            if ego_lon is not None and ego_lat is not None:
                turn_angle = self.follower.calculate_turn_angle(
                    (ego_lat, ego_lon, ego_yaw), ego_yaw)
                if turn_angle == "no_current_trajectory":
                    self.get_logger().info(f"no_current_trajectory")
                else:
                    filtered_angle = self.filter.update_speed(turn_angle)
                    desired_speed, desired_acc = self.follower.calculate_speedAndacc(
                            turn_angle, (ego_lat, ego_lon, ego_yaw), ego_v, is_obstacle = False)
                    logging.info(f'trun angle: {turn_angle}, filter angle: {filtered_angle}')
                    self.frame = [float(desired_speed), 
                                float(filtered_angle), 
                                float(desired_acc)]
                    planner_frame = Float32MultiArray()
                    planner_frame.data = self.frame
                    self.get_logger().info(f"[vs_callback] Send frame: {planner_frame.data}")
                    self.publisher_.publish(planner_frame)
        elapsed_time = time.time() - start
        self.get_logger().info(f"calc time:{elapsed_time:.6f} [sec]")

    def trajectory_callback(self, msg: PoseArray):
        """
        将订阅到的 PoseArray 转换为和 CSV 中相同的格式：
        [[x_utm, y_utm, heading], [x_utm, y_utm, heading], ...]
        """
        traj_data = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            
            # 提取四元数 (只关心 z, w，假设 x=y=0)
            qz = pose.orientation.z
            qw = pose.orientation.w
            
            # 反向获取 yaw（弧度）
            yaw_rad = 2.0 * math.atan2(qz, qw)
            
            # 将弧度转为度数
            heading_deg = math.degrees(yaw_rad)
            
            # 归一化到 [0, 360)
            heading_deg = (heading_deg + 360) % 360
            
            traj_data.append([x, y, heading_deg])
            
            # 这里用 self.traj_data 保存最新的轨迹
        self.traj_data = traj_data
        self.follower.current_trajectory = self.traj_data
        # 也可以在这里进行后续处理，如可调用其他函数、做可视化、或记录日志等
        self.get_logger().info(f'Received trajectory with {len(self.traj_data)} points.')

def main(args=None):
    main_trajectory_csv = '/home/renth/follow/collect_trajectory/processed_shiyanzhongxin_0327.csv'
    alternate_trajectory_csv = '/home/renth/follow/collect_trajectory/processed_haima-1119-right.csv'
    rclpy.init(args=args)
    follow_node = FollowNode(main_trajectory_csv, alternate_trajectory_csv)
    rclpy.spin(follow_node)
    FollowNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()