from prettytable import PrettyTable

import serial
import time
import os

uart = serial.Serial("/dev/ttyUSB1", 115200)

while True:
	os.system("clear")
	var = "p"
	uart.write(var.encode())
	result1 = uart.read(1)
	pulsos = int.from_bytes(result1, byteorder="big")
	result2 = uart.read(1)
	vueltas = int.from_bytes(result2, byteorder="big")

	print("Datos obtenidos del encoder:")
	print("----------------------------\n")
	table = PrettyTable(["Pulsos", "Vueltas"])
	table.add_row([pulsos, vueltas])
	print(table)

	time.sleep(0.250)

uart.close()
