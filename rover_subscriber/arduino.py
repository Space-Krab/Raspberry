import serial
import random
import time

direction = 0

if __name__ == '__main__':
    ser = serial.Serial('/dev/tty.usbserial-110', 9600, timeout=1)
    ser.reset_input_buffer()
    while True:
        speed = random.randint(0, 255)
        direction = 0 if direction == 1 else 1
        #print(speed.to_bytes(1) + direction.to_bytes(1))
        ser.write(speed.to_bytes(1))
        ser.write(direction.to_bytes(1))
        print(ser.readline().decode('utf-8').rstrip())
        time.sleep(2)