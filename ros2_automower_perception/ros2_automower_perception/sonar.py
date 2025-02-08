import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
import struct
from sensor_msgs.msg import PointCloud2, PointField
# 16cm hard stop
# 60cm and under register obstacle
GPIO.setmode(GPIO.BOARD)
# front right - J3
#trig_pin = 16  # GPIO14 pin connected to the Trig pin on SR04 module
#echo_pin = 18  # GPIO15 pin connected to the Echo pin on SR04 module

#front left j4
# trig_pin = 22  # GPIO14 pin connected to the Trig pin on SR04 module
# echo_pin = 24  # GPIO15 pin connected to the Echo pin on SR04 module


class Sonar(Node):

    def __init__(self):
        super().__init__('sonar')
        self.declare_parameter('echo_pin', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('trig_pin', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('frame', rclpy.Parameter.Type.STRING)
        self.trig_pin = self.get_parameter('trig_pin').get_parameter_value().INTEGER
        self.echo_pin = self.get_parameter('echo_pin').get_parameter_value().INTEGER
        self.frame = self.get_parameter('frame').get_parameter_value().STRING
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        # Create a publisher for the point cloud
        self.publisher_ = self.create_publisher(PointCloud2, 'ultrasonic_pointcloud', 10)
        
        # Timer to periodically read the sensor and publish the point cloud
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("Ultrasonic PointCloud Node Started")

    def __del__(self):
        # Clean up GPIO on destruction
        GPIO.cleanup()

    def measure_distance(self):
        # Send a 10us pulse to trigger the SR04 module
        
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)
        
        # Measure the duration of the pulse from the Echo pin
        start_time = time.time()
        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()
            
        end_time = time.time()
        while GPIO.input(self.echo_pin) == 1:
            end_time = time.time()
            
        # Calculate the distance based on the duration of the pulse
        duration = end_time - start_time
        distance = duration * (34300 / 2)  # speed of sound in cm/s
        distance = round(distance, 2)  # round to two decimal places
        
        return distance

    def timer_callback(self):
        # Measure the distance using the ultrasonic sensor
        distance = self.measure_distance()
        
        if distance is not None:
            # Create a PointCloud2 message
            pointcloud_msg = self.create_pointcloud(distance)
            
            # Publish the PointCloud2 message
            self.publisher_.publish(pointcloud_msg)
            

    def create_pointcloud(self, distance):
        # Create a PointCloud2 message
        msg = PointCloud2()
        
        # Set the header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        
        # Set the fields (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Set the point step and width
        msg.point_step = 12  # 3 floats (x, y, z) * 4 bytes each
        msg.row_step = msg.point_step
        msg.width = 1
        msg.height = 1
        
        # Set the data (single point at the measured distance)
        point_data = struct.pack('fff', distance, 0.0, 0.0)
        msg.data = point_data
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = Sonar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()