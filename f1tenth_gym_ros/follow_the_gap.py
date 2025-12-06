
# import rospy
import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.context import Context
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class FollowTheGap(Node):

    def __init__(self):
        super().__init__("follow_the_gap")
        # must publish to drive. this is how the kill switch can work when in use. Messages must be stamped

        self.get_logger().info('Follow the gap started.')
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive", 1
        )  
        self.scan = self.create_subscription(
            LaserScan(),
            # if using real car, need Lidar message
            #"/picoScan_23460001/scan/all_segments_echo0",
            "/scan",
            self.lidar_callback,
            1,
        )
        self.scan  # prevents unused variable warning
        self.lidar_fov = np.radians(180)
        self.max_speed = 5

        self.pub_drive(
            0.0, 0.0
        )  # added so the steering angle and speed always reset to 0 before driving

    def preprocess_lidar(self, ranges, window_size=15, max_value=10, avoid=50):
        """Preprocess the LiDAR scan array.
        Args:
            ranges (List[float]): A list of ranges from the LiDAR scan.
            window_size (int): The size of the window for calculating the mean.
            max_value (float): The maximum value reject anything above this.
        Returns:
            List[float]: The preprocessed LiDAR scan array.
        """

        # following code not necessary for sim, because
        # sim lidar does not have noise
        processed_data = ranges

        """
        if len(ranges) > 0:
            # Initialize result list with the same length as ranges
            processed_data = [0] * len(ranges)
            minima = min(ranges)
            loc_minima = ranges.index(minima)
            # Apply moving average with window_size
            for i in range(len(ranges)):
                if ranges[i] > max_value:
                    ranges[i] = 0
                if abs(loc_minima - i) <= (avoid / 2):
                    ranges[i] = 0
                window = ranges[max(0, i - window_size + 1) : i + 1]
                avg = sum(window) / len(window) if window else 0
                processed_data[i] = avg
        else:
            return 1
        """

        return processed_data

    def find_max_gap(self, free_space_ranges):
        """Return the start index & end index of the max gap in free_space_ranges"""
        max_gap_start = 0
        max_gap_end = 0
        max_gap_length = 0


        return max_gap_start, max_gap_end

    def lidar_callback(self, data):
        """Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message"""
        # Convert ranges to a list
        ranges = list(data.ranges)

        # max_range = max(*ranges)
        # max_range_index = ranges.index(max_range)
        # theta = self.lidar_fov * (max_range_index / len(ranges)) - self.lidar_fov / 2

        numerator_r = 0
        denominator_r = 0
        
        for r in ranges:
          index = ranges.index(r)
          theta = self.lidar_fov * (index / len(ranges)) - self.lidar_fov / 2
          
          if abs(theta) > np.radians(50):
            continue
          
          weight_r = r ** -0.7
          numerator_r += weight_r * theta 
          denominator_r += weight_r
          
        steering_angle = -numerator_r / denominator_r
        
        numerator_s = 0
        denominator_s = 0
        
        for r in ranges:
          index = ranges.index(r)
          theta = self.lidar_fov * (index / len(ranges)) - self.lidar_fov / 2
          
          if abs(theta) > np.radians(15):
            continue
          
          numerator_s += r
          denominator_s += 1
          
        speed = numerator_s / denominator_s
        speed = 2.25 * speed ** (1 / 2)
        
        print(speed)
        
        # print(ranges[0])

        # this will make it crash if there are any bends in the track
        # need modifications to this and/or the find_max_gap() method
        # these need to be floats

        self.pub_drive(speed, steering_angle)


    def pub_drive(self, speed, steering_angle):
        # publish drive messages from speed and steering angle
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )  
        drive_msg.drive.speed = speed  #! must be a float
        drive_msg.drive.steering_angle = steering_angle  # same
        self.drive_pub.publish(drive_msg)
        self.get_logger().info(f"Control commands given, Speed: {speed}, Steering Angle: {steering_angle}.")

    def cleanup_on_shutdown(self):
        self.get_logger().info('Performing cleanup actions during shutdown.')
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )  
        drive_msg.drive.speed = 0.0  #! must be a float
        drive_msg.drive.steering_angle = 0.0  # same
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    follow_the_gap = FollowTheGap()

    try:
        # Main loop continues as long as ROS is running
        while rclpy.ok():
            # Your node logic here (e.g., publish commands)
            rclpy.spin_once(follow_the_gap, timeout_sec=0.1)
    except KeyboardInterrupt:
        # This block is executed upon Ctrl+C
        pass
    finally:
        # This block is always executed during shutdown
        follow_the_gap.get_logger().info('Ctrl+C received, stopping car.')
        follow_the_gap.destroy_node()
        follow_the_gap.get_logger().info('Node shut down completely.')
        #rclpy.shutdown()

if __name__ == "__main__":
    main()
