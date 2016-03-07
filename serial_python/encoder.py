from prettytable import PrettyTable

import serial
import time
import os

uart = serial.Serial("/dev/ttyUSB2", 115200)

while True:
	os.system("clear")
	var = "p"
	uart.write(var.encode())
	result1 = uart.read(1)
	pulsos = int.from_bytes(result1, byteorder="big")
	result2 = uart.read(1)
	vueltas = int.from_bytes(result2, byteorder="big")
	result3 = uart.read(1)
	signo = int.from_bytes(result3, byteorder="big")
	
	if signo == 1:
		signo = "-"
	else:
		signo = ""

	print("Datos obtenidos del encoder:")
	print("----------------------------\n")
	table = PrettyTable(["Pulsos", "Vueltas"])
	table.add_row([pulsos, signo + str(vueltas)])
	print(table)

	time.sleep(0.250)

uart.close()
