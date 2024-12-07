import serial
import time

ser = serial.Serial('/dev/ttyACM0',9600,timeout = 1)
nb = 0

while True :
    ser.write(str(nb).encode('utf-8')+b"\n")
    nb = ser.readline().decode('utf-8').rstrip()
    print(nb)
    if nb != '':
        nb = int(nb)+1
    time.sleep(1)
