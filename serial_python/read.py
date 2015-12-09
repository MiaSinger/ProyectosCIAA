import serial

uart = serial.Serial("/dev/ttyUSB1", 115200)

while True:
    result = uart.read(1)
    print(result)

uart.close()
