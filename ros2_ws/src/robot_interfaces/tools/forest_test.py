import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Velocity, Lidar, BnoReading, Odom, GridData
from robot_interfaces.srv import KfsServicePickup, ServiceCall
from std_msgs.msg import Int32
import math

class ForestSimulator(Node):
    def __init__(self):
        super().__init__('forest_simulator')

        # --- Internal Robot State ---
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.vx, self.vy, self.vw = 0.0, 0.0, 0.0
        
        # --- Publishers ---
        self.lidar_pub = self.create_publisher(Lidar, '/lidar', 10)
        self.bno_pub = self.create_publisher(BnoReading, '/current_angle', 10)
        self.odom_pub = self.create_publisher(Odom, '/odom', 10)
        self.grid_pub = self.create_publisher(GridData, '/grid_map', 10)
        self.status_pub = self.create_publisher(Int32, '/bot_status', 10)

        # --- Subscribers (Listen to the Nav Node) ---
        self.vel_sub = self.create_subscription(Velocity, '/velocity_request', self.vel_cb, 10)

        # --- Services (Simulate Hardware Actions) ---
        self.kfs_srv = self.create_service(KfsServicePickup, 'kfs_service', self.handle_kfs)
        self.climb_srv = self.create_service(ServiceCall, 'set_climb_mode', self.handle_climb)

        # --- Simulation Loop ---
        self.timer = self.create_timer(0.1, self.update_physics)
        self.get_logger().info("Forest Simulator Online - Ready for Nav Test")

    def vel_cb(self, msg):
        self.vx = msg.vx
        self.vy = msg.vy
        self.vw = msg.angular_z

    def handle_kfs(self, request, response):
        self.get_logger().info(f"SIMULATOR: Executing KFS Pickup (Mode: {request.pickupstate})")
        # Simulating a 2-second mechanical action
        import time
        time.sleep(1.0) 
        return response # Success by default

    def handle_climb(self, request, response):
        mode = "UP" if request.climb_mode == 1 else "DOWN"
        self.get_logger().info(f"SIMULATOR: Robot is climbing {mode}")
        return response

    def update_physics(self):
        # Update Coordinates based on velocity
        dt = 0.1
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.yaw += self.vw * dt

        # 1. Publish Odom
        o_msg = Odom()
        o_msg.odom_x, o_msg.odom_y, o_msg.current_angle = self.x, self.y, self.yaw
        self.odom_pub.publish(o_msg)

        # 2. Publish BNO
        b_msg = BnoReading()
        b_msg.current_angle = self.yaw
        self.bno_pub.publish(b_msg)

        # 3. Publish Lidar (Simulate 500mm distance until we "move" enough)
        l_msg = Lidar()
        l_msg.lidar_front = 500.0 if self.x < 1.0 else 5.0 # Stop when x > 1m
        l_msg.lidar_back = 500.0
        l_msg.lidar_left = 500.0
        l_msg.lidar_right = 500.0
        self.lidar_pub.publish(l_msg)

        # 4. Periodic Grid and Status (To keep Nav Node alive)
        g_msg = GridData()
        g_msg.layout = [0, 0, 0, 3, 2, 0, 0, 0, 0, 0, 0, 0] # Example layout
        self.grid_pub.publish(g_msg)

def main():
    rclpy.init()
    node = ForestSimulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()