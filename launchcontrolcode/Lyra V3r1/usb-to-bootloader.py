import serial
import serial.tools.list_ports
import time

ports = serial.tools.list_ports.comports()
baud = 115200
magic_baud = 1200

print("Detected devices")
default_idx = -1
for idx, port in enumerate(ports):
    is_pico = port.vid == 0x2E8A
    print("[{}] {}: {} [{}]".format(idx, port.device, port.description, port.hwid) + (" (PICO!)" if is_pico else ""))
    if is_pico:
        default_idx = idx

if default_idx == -1:
    print("No pico detected")
    exit()

port = ports[default_idx]

counter = 0
device = serial.Serial(port=port.device, baudrate=baud, timeout=0.2)
try:
    for i in range(30):
        line  = device.readline()
        print(line)
        counter += 1
        if counter == 3:
            device.baudrate = magic_baud

except serial.serialutil.SerialException:
    print("Rebooted to bootloader, maybe?")



