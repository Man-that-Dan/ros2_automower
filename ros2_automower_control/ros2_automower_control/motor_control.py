from simple_pid import PID
import serial
import struct
import time
import rclpy
#TODO convert to params
ZERO_VELOCITY = 1488
LOWER_LIMIT = 876
UPPER_LIMIT = 2100

class MotorController(rclpy.Node):
    def __init__(self):
        # Initialize Node, Serial, etc...

        # Initialize PID controllers for each motor
        self.pid_motor1 = PID(1, 0.1, 0.05, setpoint=0)  # Example gains, tune accordingly
        self.pid_motor2 = PID(1, 0.1, 0.05, setpoint=0)  # Example gains, tune accordingly
        self.pid_motor1.output_limits = (-100, 100)  # Limits for motor speed
        self.pid_motor2.output_limits = (-100, 100)  # Limits for motor speed

    def convert_to_motor_values(command):
        command1 = command
    def run_pid_loop(self):
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
