import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
import time
import sys
sys.path.append('/home/nvidia/vcii/hezi_ros2/src/demo1/follow_traj_re/follow_traj_re')
from mpc_follower import MPCfollower, State
from can_use import ISGSpeedFilter
from can_use import Can_use
import logging

class FollowNode(Node):
    def __init__(self, main_trajectory_csv):
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
        self.manual_triggered = True
        self.mode_AE = 1
        self.mode_666 = 1
        self.eps_subscription
        self.vs_subscription  # prevent unused variable warning
        self.follower = MPCfollower(main_trajectory_csv)
        self.filter = ISGSpeedFilter()
        self.latest_eps_mode = None
        self.can_use = Can_use()
        self.planner_frame = Float32MultiArray()
        self.planner_frame.data = [0.0, 
                                   0.0,
                                   0.0]

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
        print("temp time: ",time.time()-start)
        x = self.can_use.ego_x
        y = self.can_use.ego_y
        yaw = self.can_use.ego_yaw
        v   = self.can_use.ego_v
        state = State(x=x, y=y, yaw=yaw, v=v)
        if eps_mode != 3 and self.manual_triggered:
            self.mode_AE = 1
            self.mode_666 = 1
        if eps_mode == 3:
            self.mode_AE = 3
            self.mode_666 = 0
            self.manual_triggered = False
        if self.mode_AE == 1 and self.mode_666 == 1:
            if x is not None and y is not None:
                t0 = time.time()
                acc, turn_angle = self.follower.act(state, obs_publish=None)
                t1 = time.time()
                print("temp1 time :", t1-t0)
                # self.follower.update_target_index(state)
                filtered_angle = self.filter.update_speed(turn_angle)
                logging.info(f'trun angle: {turn_angle}, filter angle: {filtered_angle}')
                self.frame = [5.0, 
                              float(filtered_angle), 
                              0.0]
                self.planner_frame.data = self.frame
                self.get_logger().info(f"[vs_callback] Send frame: {self.planner_frame.data}")
                self.publisher_.publish(self.planner_frame)
                print("temp2 time :", time.time()-t1)
                return 
        self.publisher_.publish(self.planner_frame)
        elapsed_time = time.time() - start
        self.get_logger().info(f"Calculation time:{elapsed_time:.6f} [sec]")

def main(args=None):
    main_trajectory_csv = '/home/nvidia/vcii/follow_trajectory/collect_trajectory/processed_straight12_17_with_yaw_ck.csv'
    rclpy.init(args=args)
    follow_node = FollowNode(main_trajectory_csv)
    rclpy.spin(follow_node)
    FollowNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()