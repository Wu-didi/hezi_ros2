import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import sys

# 添加模块所在路径
sys.path.append('/home/nvidia/vcii/hezi_ros2/src/demo1/follow_traj_re/follow_traj_re')

# from mpc_follower import MPCfollower, State
from follow_demo_mpc_bank import VehicleTrajectoryFollower, ISGSpeedFilter
from can_use import  Can_use
import logging

class FollowNode(Node):
    def __init__(self, main_trajectory_csv):
        super().__init__('Follow_node')
        # 创建发布者
        self.publisher_ = self.create_publisher(Float32MultiArray, 'planner_action', 1)
        
        # 初始化计算相关对象
        self.follower = VehicleTrajectoryFollower(main_trajectory_csv)
        self.filter = ISGSpeedFilter()
        self.can_use = Can_use()

    def compute_and_publish(self):
        """
        手动调用的计算与发布函数：
        1. 读取当前车辆状态（通过 can_use 对象）
        2. 调用 MPCfollower 计算控制指令
        3. 对控制指令进行滤波
        4. 发布最新计算得到的控制指令
        """
        start_time = time.time()
        
        # 模拟连续读取 INS 数据（读取 20 次数据）
        for _ in range(20):
            self.can_use.read_ins_info()
        new_frame = [0,0,0]
        # 调用 MPCfollower 计算控制指令
        if self.can_use.ego_x is not None and self.can_use.ego_y is not None:     
            turn_angle = self.follower.calculate_turn_angle((self.can_use.ego_x, self.can_use.ego_y, self.can_use.ego_yaw), self.can_use.ego_yaw, self.can_use.ego_v)
            # turn_angle = 0
            self.follower.update_target_index((self.can_use.ego_x, self.can_use.ego_y), self.can_use.ego_yaw, self.can_use.ego_v)
            # print("turn_angle=", turn_angle)
            filtered_angle = self.filter.update_speed(turn_angle)
            # print("===========",turn_angle)
            # print("time cost: ",time.time()-t0)
            new_frame = [5.0, float(filtered_angle), 0.0]     
        
        # 构造发布消息（示例中使用固定加速度5.0，实际可根据需求调整）
        planner_frame = Float32MultiArray()
        planner_frame.data = new_frame
        
        # 发布消息
        self.get_logger().info(f"Publishing frame: {planner_frame.data}")
        self.publisher_.publish(planner_frame)
        
        elapsed_time = time.time() - start_time
        self.get_logger().info(f"Calculation time: {elapsed_time:.6f} sec")

def main(args=None):
    rclpy.init(args=args)
    main_trajectory_csv = '/home/nvidia/vcii/follow_trajectory/collect_trajectory/processed_shiyanzhongxin_0327_with_yaw_ck.csv'
    follow_node = FollowNode(main_trajectory_csv)
    
    try:
        # 手动调用计算与发布函数
        # 这里使用一个 while 循环，每隔一定时间调用一次计算与发布
        while rclpy.ok():
            follow_node.compute_and_publish()
            # 根据需要调整循环
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass

    follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
