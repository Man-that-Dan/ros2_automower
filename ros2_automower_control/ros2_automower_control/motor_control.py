import struct
import serial
import time
import rclpy
from simple_pid import PID
from geometry_msgs.msg import Twist


class MotorController(rclpy.Node):
    """MotorController class handles translating values from /cmd_vel to signals
    for motor control board and sending to board

    Parameters:
        kp (double): proportional gain,                          default 1.0
        ki (double): integral gain,                              default 0.1
        kd (double): derivative gain,                            default 0.05
        lower_vel_limit (double):                                default -2
        upper_vel_limit (double):                                default 2
        zero_velocity_pwm (int):  duty cycle for zero velocity   default 1488
        lower_limit_pwm (int):  duty cycle for lower limit       default 876
        upper_limit_pwm (int):  duty cycle for upper limit       default 2100

    """
    def __init__(self):
        super().__init__('motor_controller_node')
        # Initialize Node, Serial, etc...
        self.velocity_setpoint = 0 #initial velocity
        self.declare_parameter('kp', 1.0) #kp gain
        self.declare_parameter('ki', 0.1) #ki gain
        self.declare_parameter('kd', 0.05) #kd gain
        self.declare_parameter('lower_vel_limit', -2) # lower speed limit in m/s
        self.declare_parameter('upper_vel_limit', 2)  # upper speed limit in m/s
        self.declare_parameter('zero_velocity_pwm', 1488)  # duty cycle for zero velocity
        self.declare_parameter('lower_limit_pwm', 876)  # duty cycle for zero velocity
        self.declare_parameter('upper_limit_pwm', 2100)  # duty cycle for zero velocity
        # Initialize PID controllers for each motor
        self.pid_motor1 = PID(self.get_parameter('kp').get_parameter_value(),
                              self.get_parameter('ki').get_parameter_value(),
                              self.get_parameter('kd').get_parameter_value(), setpoint=self.velocity_setpoint)

        self.pid_motor2 = PID(self.get_parameter('kp').get_parameter_value(),
                              self.get_parameter('ki').get_parameter_value(),
                              self.get_parameter('kd').get_parameter_value(), setpoint=self.velocity_setpoint)
        # Limits for motor speed in m/s
        self.pid_motor1.output_limits=(self.get_parameter('lower_vel_limit').get_parameter_value(),
                                    self.get_parameter('upper_vel_limit').get_parameter_value())
        self.pid_motor2.output_limits=(self.get_parameter('lower_vel_limit').get_parameter_value(),
                                    self.get_parameter('upper_vel_limit').get_parameter_value())
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.set_velocity)
        
    def set_velocity(self, twist_msg : Twist):
        """ Set velocity setpoint from twist message 

        Args:
            twist_msg (Twist): twist command with velocity
        """
        self.velocity_setpoint = twist_msg.linear.y
    

    def convert_to_motor_values(self, command):
        """Convert percentage based commands commands from -100 to 100 into
        duty cycle values used by motor control board

        Args:
            command (tuple): tuple of motor commands
        """
        command1 = command[0]
        command2 = command[1]
        zero_vel_pwm = self.get_parameter('zero_velocity_pwm').get_parameter_value()
        lower_vel_pwm = self.get_parameter('lower_limit_pwm').get_parameter_value()
        upper_vel_pwm = self.get_parameter('upper_limit_pwm').get_parameter_value()
        lower_limit = self.get_parameter('lower_vel_limit').get_parameter_value()
        upper_limit = self.get_parameter('upper_vel_limit').get_parameter_value()
        if command1 < 0:
            diff = int(((command1 / lower_limit) * (zero_vel_pwm - lower_vel_pwm)))
            command1 = zero_vel_pwm - diff
        else:
            diff = int(((command1 / upper_limit) * (upper_vel_pwm - zero_vel_pwm)))
            command1 = zero_vel_pwm + diff

        if command2 < 0:
            diff = int(((command2 / lower_limit) * (zero_vel_pwm - lower_vel_pwm)))
            command2 = zero_vel_pwm - diff
        else:
            diff = int(((command2 / upper_limit) * (upper_vel_pwm - zero_vel_pwm)))
            command2 = zero_vel_pwm + diff

        return command1, command2

    def run_pid_loop(self):
        """ run pid loop to achieve set point
        """
        while rclpy.ok():
            type = self.ser.read(1)
            try:
                type = struct.unpack("<c", type)
                type = type[0].decode('utf-8')
            except Exception as e:
                self.get_logger().error(e.__str__())
            if type == 'f':
                feedback = self.ser.read(8)
                feedback = struct.unpack("<hhL", feedback)
                feedback_msg = rclpy.Int32MultiArray(data=list(feedback))
                self.publisher_feedback.publish(feedback_msg)
                current_speeds = self.calculate_current_speeds(feedback)
                # Perform PID control
                setpoint1, setpoint2 = self.convert_to_motor_values([0, 0])  # Assuming the setpoint is 0 for both motors
                control_signal_motor1 = self.pid_motor1(current_speeds[0])
                control_signal_motor2 = self.pid_motor2(current_speeds[1])
                # Apply control signals to motors
                self.apply_control_signals(control_signal_motor1, control_signal_motor2)
            elif type == 'c':
                com_ack = self.ser.read(4)
                com_ack = struct.unpack("<hh", com_ack)
                line = self.ser.readline().decode('utf-8').rstrip()
                self.get_logger().debug(line)
            elif type == 's':        
                line = self.ser.readline().decode('utf-8').rstrip()
                self.get_logger().info(line)
            rclpy.spin_once(self)

    def apply_control_signals(self, control_signal_motor1, control_signal_motor2):
        # Apply control signals to motors
        cmd = struct.pack("<chh", b'c', control_signal_motor1, control_signal_motor2)
        self.ser.write(cmd)
        self.get_logger().debug(f"Sent command: {control_signal_motor1} {control_signal_motor2}")


# import RPi.GPIO as GPIO
# import sys

# ZERO_VELOCITY = 1488

# if __name__ == '__main__':
#     ser = serial.Serial('/dev/ttyACM1', 9600)
#     ser.reset_input_buffer()
#     speed = ZERO_VELOCITY
#     if len(sys.argv) > 1:
#         speed = int(sys.argv[1])
#     cmd = struct.pack("<chh", b'c', speed, speed)
#     print(cmd)
#     time.sleep(1)
#     print("waiting")
#     ser.write(cmd)

  
#     while True:
#         type = ser.read(1)
#         try:
#             type = struct.unpack("<c", type)
#             type = type[0].decode('utf-8')
#         except:
#             pass
#         if type == 'f':
#         #feedback 
#             feedback = ser.read(8)
#             feedback = struct.unpack("<hhL", feedback)
#             print("feedback")
#             print(feedback)
#             #command1 = 0xffff000000000000 & feedback[0]
#             #command1 = (command1 >> 6) & 0xffff

#             #print(command1)
#         elif type == 'c':
#         #command ack
#             com_ack = ser.read(4)
#             com_ack = struct.unpack("<hh", com_ack)
#             print("ack")
#             print(com_ack)
#             #line = ser.readline().decode('utf-8').rstrip()
#             #print(line)
#         elif type == 's':
#         #command ack
            
#             line = ser.readline().decode('utf-8').rstrip()
#             print(line)

# import os

# def PortName(usb, desc):
#   usb = usb[5:]
#   if usb.endswith(":1.0"):
#     usb = usb[:-4]
#   if usb.startswith("."):
#     desc = desc + " then Hub Port " + usb[1:]
#   return desc

# def PortInfo(usb,tag):
#   inf = usb[:usb.find(":")]
#   if os.path.isfile("/sys/bus/usb/devices/"+inf+"/"+tag):
#     with open("/sys/bus/usb/devices/"+inf+"/"+tag,"r") as f:
#       return f.read().strip()
#   return "<no "+tag+">"

# def Main():
#   if not os.path.isdir("/sys/bus/usb/devices"):
#     print("No USB Serial devices attached")
#     return 
#   lst = os.popen("ls -l /sys/bus/usb/devices/").read().split("\n")
#   for s in lst:
#     # lrwxrwxrwx ... -> ... .usb/usb1/1-1/1-1.2/1-1.2:1.0/ttyUSB0
#     # lrwxrwxrwx ... -> ... .usb/usb1/1-1/1-1.3/1-1.3:1.0/ttyUSB1
#     #                                           |_______| |_____|
#     n = s.find(".usb/usb")
#     if n >= 0:
#       n = s.rfind("/")
#       nam = s[n+1:]
#       s = s[:n]
#       n = s.rfind("/")
#       usb = s[n+1:]
#       if   usb.startswith("1-1.2") : port = PortName(usb, "Top Left")
#       elif usb.startswith("1-1.3") : port = PortName(usb, "Bottom Left")
#       elif usb.startswith("1-1.4") : port = PortName(usb, "Top Right")
#       elif usb.startswith("1-1.5") : port = PortName(usb, "Bottom Right")
#       else                         : port = usb
#       vid = PortInfo(usb,"idVendor")
#       pid = PortInfo(usb,"idProduct")
#       mfr = PortInfo(usb,"manufacturer")
#       prd = PortInfo(usb,"product")
#       ver = PortInfo(usb,"version")
#       ser = PortInfo(usb,"serial")
#       print(nam+" "+port+" "+vid+":"+pid+" "+mfr+" "+prd+" "+ver+" "+ser)

# if __name__ == "__main__":
#   Main()
