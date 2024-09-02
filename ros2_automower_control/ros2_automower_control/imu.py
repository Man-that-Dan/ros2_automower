import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import numpy as np
from mpu6050 import mpu6050
import time

class IMUPublisherNode(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish at 10 Hz
        self.sensor = mpu6050(0x68)
        self.sensor.bus.write_byte_data(self.sensor.address, 0x6B, 0b10001000)
        time.sleep(0.5)
        self.sensor.bus.write_byte_data(self.sensor.address, 0x68, 0b00000111)
        time.sleep(0.5)
        self.sensor.bus.write_byte_data(self.sensor.address, 0x6B, 0x00)
        time.sleep(4)

    def publish_imu_data(self):
        imu_msg = Imu()

        # Set header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        accel, orientation = self.get_imu_data()
        imu_msg.linear_acceleration = accel
        imu_msg.orientation = orientation
   

        # Publish IMU data
        self.publisher.publish(imu_msg)

    def random_orientation(self):
        # Generate a random quaternion for orientation
        rand_quat = np.random.randn(4)
        rand_quat /= np.linalg.norm(rand_quat)
        return Quaternion(x=rand_quat[0], y=rand_quat[1], z=rand_quat[2], w=rand_quat[3])

    def random_angular_velocity(self):
        # Generate random angular velocity (rad/s)
        return Vector3(x=np.random.uniform(-1.0, 1.0),
                       y=np.random.uniform(-1.0, 1.0),
                       z=np.random.uniform(-1.0, 1.0))

    def random_linear_acceleration(self):
        # Generate random linear acceleration (m/s^2)
        return Vector3(x=np.random.uniform(-10.0, 10.0),
                       y=np.random.uniform(-10.0, 10.0),
                       z=np.random.uniform(-10.0, 10.0))
    def get_imu_data(self):
  
        #acelerometer_data = sensor.read_i2c_word(0x41)
        
        accel, gyro, temp = self.sensor.get_all_data()
        accel_vector = Vector3(x=accel["x"], y=accel["y"], z=accel["z"])
        gyro_vector = Vector3(x = gyro["x"], y = gyro["y"], z = gyro["z"])

        return accel_vector, gyro_vector

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()