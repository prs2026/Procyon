import serial,sys,time,os

import serial.tools.list_ports as port_list

portname = "COM3"

try:
    serialport = serial.Serial()
    serialport.baudrate = 115200
    serialport.bytesize = 8
    serialport.parity = "N"
    serialport.stopbits = 1
    serialport.timeout = 10
except:
     print("serial fail")


print("open ports: ")

ports = list(port_list.comports())
for p in ports:
    print (p)

portname = input("type name of port from list above, ex COM3\n")

print("opening: "+portname)

serialport.port = portname
try:
    serialport.open()

except:
    print("unable to open port")
    sys.exit()

print("reading data")
serialport.write(b'D')
time.sleep(1)
readdata = ""
while serialport.in_waiting > 0:
    readdata = readdata + str(serialport.read_until('done'))
    readdata = readdata.replace('\\n','\n')
readdata = readdata.replace('\\r','')
readdata = readdata.replace("b'echo: D dec: 68 \n",'')
readdata = readdata.replace("68\n",'')
readdata = readdata.replace("dumping data to serial\n",'')
print(readdata)

readtime = str(time.strftime('%m,%d,%Y,%H,%M,%S'))
currentdir = os.path.realpath(__file__)

currentdir = os.path.abspath(os.path.join(currentdir, os.pardir))

path = currentdir + ("\LOGS "+readtime)

print(path)

os.mkdir(path)
rawfilepath = path + "/rawfile.csv"
file = open(rawfilepath,'w+')

file.write(readdata)

file.close()

splitdata = readdata.split("newfile")
flightnum = 1
for x in splitdata:
    x = x.replace("newfile\n",'')
    x = x.replace("\nindex",'index')
    logfilepath = path +('/flight '+str(flightnum) + '.csv')
    logfile = open(logfilepath,'w+')
    logfile.write(x)
    logfile.close()
    flightnum += 1