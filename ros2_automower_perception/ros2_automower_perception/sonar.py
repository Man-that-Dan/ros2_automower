import RPi.GPIO as GPIO
import time
# 16cm hard stop
# 60cm and under register obstacle
GPIO.setmode(GPIO.BOARD)
# front right - J3
#trig_pin = 16  # GPIO14 pin connected to the Trig pin on SR04 module
#echo_pin = 18  # GPIO15 pin connected to the Echo pin on SR04 module

#front left j4
trig_pin = 22  # GPIO14 pin connected to the Trig pin on SR04 module
echo_pin = 24  # GPIO15 pin connected to the Echo pin on SR04 module
GPIO.setup(trig_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

def distance():
    # Send a 10us pulse to trigger the SR04 module
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)
    
    # Measure the duration of the pulse from the Echo pin
    start_time = time.time()
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
        
    end_time = time.time()
    while GPIO.input(echo_pin) == 1:
        end_time = time.time()
        
    # Calculate the distance based on the duration of the pulse
    duration = end_time - start_time
    distance = duration * 17150  # speed of sound in cm/s
    distance = round(distance, 2)  # round to two decimal places
    
    return distance

# Main loop
try:
    while True:
        
        dist = distance()
        print(f"Distance: {dist} cm")
        time.sleep(0.5)
        
except KeyboardInterrupt:
    GPIO.cleanup()