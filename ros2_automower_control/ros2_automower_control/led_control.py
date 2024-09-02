import RPi.GPIO as GPIO
import time

LED_PIN = 38
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.HIGH)
#h = lgpio.gpiochip_open(0)
#lgpio.gpio_claim_output(h, LED_PIN)
#lgpio.gpio_write(h, LED_PIN, 1)
time.sleep(25)
GPIO.output(LED_PIN, GPIO.LOW)
GPIO.cleanup()
#lgpio.gpio_write(h, LED_PIN, 0)
#lgpio.gpiochip_close(h)