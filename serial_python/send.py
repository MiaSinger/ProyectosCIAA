import serial

uart = serial.Serial("/dev/ttyUSB1", 115200)

while True:
    var = input("")
    uart.write(var.encode())

uart.close()
