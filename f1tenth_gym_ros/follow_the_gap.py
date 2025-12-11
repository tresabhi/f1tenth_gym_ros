import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class FollowTheGap(Node):

    simulate_noise = False
    noise_mean = 0.0
    noise_std = 0.1

    smooth_noise = True
    smoothing_window_angle = np.radians(5)

    def __init__(self):
        super().__init__("follow_the_gap")

        self.get_logger().info("Follow the gap started.")
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 1)
        self.scan = self.create_subscription(
            LaserScan(), "/scan", self.lidar_callback, 1
        )
        self.scan
        self.lidar_fov = np.radians(180)
        self.max_speed = 5

        self.pub_drive(0.0, 0.0)

    def preprocess_lidar(self, ranges, window_size=15, max_value=10, avoid=50):
        processed_data = ranges
        return processed_data

    def find_max_gap(self, free_space_ranges):
        max_gap_start = 0
        max_gap_end = 0
        max_gap_length = 0

        return max_gap_start, max_gap_end

    def lidar_callback(self, data):
        ranges = list(data.ranges)

        if self.simulate_noise:
            ranges = [
                max(
                    r + np.random.normal(loc=self.noise_mean, scale=self.noise_std), 0.0
                )
                for r in ranges
            ]

        if self.smooth_noise:
            window = int(len(ranges) * (self.smoothing_window_angle / self.lidar_fov))
            smoothed = []

            for i in range(len(ranges)):
                start = max(0, i - window + 1)
                window_slice = ranges[start : i + 1]
                smoothed.append(sum(window_slice) / len(window_slice))

            ranges = smoothed

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
        speed = 1.7 * speed**0.65

        numerator_r = 0
        denominator_r = 0

        for r in ranges:
            index = ranges.index(r)
            theta = self.lidar_fov * (index / len(ranges)) - self.lidar_fov / 2

            if abs(theta) > np.radians(50):
                continue

            weight_r = r**-0.7
            numerator_r += weight_r * theta
            denominator_r += weight_r

        beta = 10.0 * 1.2 ** -(0.9 * speed)
        steering_angle = -beta * numerator_r / denominator_r

        self.pub_drive(speed, steering_angle)

    def pub_drive(self, speed, steering_angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)
        self.get_logger().info(
            f"Control commands given, Speed: {speed}, Steering Angle: {steering_angle}."
        )

    def cleanup_on_shutdown(self):
        self.get_logger().info("Performing cleanup actions during shutdown.")
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    follow_the_gap = FollowTheGap()

    try:
        while rclpy.ok():
            rclpy.spin_once(follow_the_gap, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        follow_the_gap.get_logger().info("Ctrl+C received, stopping car.")
        follow_the_gap.destroy_node()
        follow_the_gap.get_logger().info("Node shut down completely.")


if __name__ == "__main__":
    main()
