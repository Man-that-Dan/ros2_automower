import rclpy
from rclpy.node import Node
from sensor_msgs import BatteryState
from ina219 import INA219
from ina219 import DeviceRangeError

SHUNT_OHMS = 0.1
class PowerMonitor(Node):

    def __init__(self):
        super().__init__('PowerMonitor')
        self.declare_parameter('battery_type', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('max_voltage', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('min_voltage', rclpy.Parameter.Type.DOUBLE)
        self.battery_type = self.get_parameter('battery_type').get_parameter_value().INTEGER
        self.max_voltage = self.get_parameter('max_voltage').get_parameter_value().DOUBLE
        self.min_voltage = self.get_parameter('min_voltage').get_parameter_value().DOUBLE
        self.ina = INA219(SHUNT_OHMS, busnum=1)
        self.ina.configure()
        self.battery_status_publisher = self.create_publisher(BatteryState, 'battery_state', 10)
        self.battery_timer = self.create_timer(1.0, self.publish_battery_state)

    def publish_battery_state(self):
        voltage, current = self.read()
        batt_status = BatteryState()
        batt_status.voltage = voltage
        batt_status.current = current
        batt_status.percentage = (voltage - self.max_voltage) / (self.max_voltage - self.min_voltage)
        self.battery_status_publisher.publish(batt_status)


    def read(self):
        voltage = self.ina.voltage
        current = self.ina.current()
        return voltage, current
    

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PowerMonitor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
