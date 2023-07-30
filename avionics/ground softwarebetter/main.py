import time

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color,Rectangle
from kivy.properties import ColorProperty
from kivy.clock import Clock
from kivy.uix.textinput import TextInput
from kivy.core.window import Window
 
Window.size = (700, 600)

import serial
from typing import NamedTuple
import datetime
import time



serialport = serial.Serial()
serialport.baudrate = 115200
serialport.bytesize = 8
serialport.parity = "N"
serialport.stopbits = 1
serialport.timeout = 10

launcharmed = False

def getlocaltime():
    localtime = time.localtime()
    localtimename = str(localtime.tm_mon) + "-" + str(localtime.tm_mday) + "-" + str(localtime.tm_year) + "-" + str(localtime.tm_hour) + "-" + str(localtime.tm_min) + "-" + str(localtime.tm_sec)
    return localtimename


def convertMillis(millis):
    seconds = int(millis / 1000) % 60
    minutes = int(millis / (1000 * 60)) % 60
    hours = int(millis / (1000 * 60 * 60)) % 24
    return seconds, minutes, hours


currenttime = time.time()

readstring = ""
filename = "logs/lerolog,"+getlocaltime()+".csv"
with open(filename,"w") as logfile:
    logfile.write("local time, checksum, BEAR uptime,FOX uptime,data age,FOX state, x accel, y accel, z accel, yaw rate, pitch rate, roll rate, pitch, yaw, rol, barometric altitude, absolute velocity, velocity x, velocity y, velocity z, vertical velocity, absolute acceleration,checksum,groundcommand\n")
    global datarec
    datarec = {
        "state": 0,
        "launchcompuptime": 0,
        "flightcompuptime": 0,
        "missiontime": 0,
        "dataage":0,
        "altitude": 0,
        "velocity": 0,
        "verticalvel": 0,
        "pitch": 0,
        "roll": 0,
        "yaw": 0,
        "pitchrate": 0,
        "rollrate": 0,
        "yawrate": 0,
        "x_accel": 0,
        "y_accel": 0,
        "z_accel": 0,
        "x_vel": 0,
        "y_vel": 0,
        "z_vel": 0,
        "flightcompbattery": 0,
        "launchcompbattery": 0,
        "absaccel": 0,
        "missiontime": 0
    }

    launchstarttime = 0.0
    currenttime = 0.0
    currentcom = 0
    global trylaunch
    trylaunch = False
    global nopacket
    nopacket = 0
    class MainWidget(BoxLayout):

        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self.task = Clock.schedule_interval(self.callupdate,0.1)
            port = "COM25"

        def launch(self):
            if self.launcharmed == True:
                print("go for launch")
                global trylaunch
                trylaunch= True
                global launchstarttime
                launchstarttime = time.time()
                self.currentcom = 108
        def abort(self):
            global trylaunch
            trylaunch = False
            launchstarttime = 0
            self.currentcom = 107
            print("abort")

        def zeroimu(self):
            try:
                if serialport.isOpen() == True:
                    serialport.write(bytes(b"z"))
                    print("zeroing imu")
                    self.currentcom = 122
            except:
                pass
        def calibratebarometer(self):
            try:
                if serialport.isOpen() == True:
                    serialport.write(bytes(b'b'))
                    print("baro calibrate")
                    self.currentcom = 98
            except:
                pass
        def beep(self):
            try:
                if serialport.isOpen() == True:
                    serialport.write(bytes(b'w'))
                    print("beep")
                    self.currentcom = 119
            except:
                pass

        def restartfox(self):
            try:
                if serialport.isOpen() == True:
                    serialport.write(bytes(b't'))
                    print("fox restart")
                    self.currentcom = 116
            except:
                pass
        def callupdate(self,dt):
            global nopacket
            if self.ids.arm_text.text == "ARMED" :
                self.launcharmed = True
                self.ids.launchbutton.background_color = (0.9,0.2,0.2,1)
                self.ids.arm_text.background_color = (0.9,0.2,0.2,1)
            else:
                self.launcharmed = False
                self.ids.launchbutton.background_color = (0.1,0.3,0.9,1)
                self.ids.arm_text.background_color = (0.1,0.3,0.9,1)


            try:
                if (serialport.isOpen() == True and serialport.in_waiting >= 1):
                    nopacket = 0
                    readstring = str(serialport.readline())
                    serialport.reset_input_buffer()
                    readstring = readstring.replace("\\r\\n'", "")
                    readstring = readstring.replace("b'","")
                    readstringnew = readstring.split(",")
                    #print(readstringnew)
                    if readstringnew[0] == "101" and readstringnew[22] == "101":
                        print(readstringnew)
                        logfile.write(getlocaltime() + ",")
                        logfile.write(readstring)
                        datarec["launchcompuptime"] = int(readstringnew[1])
                        datarec["flightcompuptime"] = int(readstringnew[2])
                        datarec["dataage"] = int(readstringnew[3])
                        datarec["state"] = int(readstringnew[4])
                        datarec["x_accel"] = float(readstringnew[5])
                        datarec["y_accel"] = float(readstringnew[6])
                        datarec["z_accel"] = float(readstringnew[7])
                        datarec["yawrate"] = float(readstringnew[8])
                        datarec["pitchrate"] = float(readstringnew[9])
                        datarec["rollrate"] = float(readstringnew[10])
                        datarec["pitch"] = float(readstringnew[11])
                        datarec["roll"] = float(readstringnew[12])
                        datarec["pitch"] = float(readstringnew[13])
                        datarec["altitude"] = float(readstringnew[14])
                        datarec["velocity"] = float(readstringnew[19])
                        datarec["x_vel"] = float(readstringnew[16])
                        datarec["y_vel"] = float(readstringnew[17])
                        datarec["z_vel"] = float(readstringnew[18])
                        datarec["verticalvel"] = float(readstringnew[19])
                        datarec["absaccel"] = float(readstringnew[20])
                        datarec["flightcompbattery"] = float(readstringnew[21])
                        #checksum 22
                        datarec["missiontime"] = float(readstringnew[23])
                        
            except:
                print("cant get data")
                nopacket = nopacket + 1
                print(nopacket)
                pass

            if (serialport.isOpen() == True):
                self.ids.connect_button.background_color = (0.1,0.5,0.1)
                self.ids.comport_textinput.background_color = (0.1, 0.5, 0.1)
            else:
                self.ids.connect_button.background_color = (0.1, 0.2, 0.1)
                self.ids.comport_textinput.background_color = (0.1, 0.2, 0.1)
            self.port = self.ids.comport_textinput.text
            uptime = convertMillis(datarec["launchcompuptime"])
            uptimefox = convertMillis(datarec["flightcompuptime"])
            self.ids.bearuptime_label.text = "BEAR uptime: " + str(uptime[2]) + ":" + str(uptime[1]) + ":" + str(uptime[0])
            self.ids.compuptime_label.text = "FOX uptime: " + str(uptimefox[2]) + ":" + str(uptimefox[1]) + ":" + str(uptimefox[0])
            if datarec["dataage"] >= 2000:
                self.ids.data_age_label.text = "Data Stale! \n Data Age: " + str(int(datarec["dataage"] / 1000)) + ":" + str(int(datarec["dataage"] % 1000))
            else:
                self.ids.data_age_label.text = "Data Age: " + str(int(datarec["dataage"]/1000)) + ":" + str(int(datarec["dataage"]%1000))

            self.ids.missiontime_label.text = "Mission Elapsed Time: " + str(int(datarec["missiontime"] / 1000)) + ":" + str(int(datarec["missiontime"] % 1000))

            self.ids.x_accel_label.text = "X Acceleration: " + str(datarec["x_accel"])+ "m/s^2"
            self.ids.y_accel_label.text = "Y Acceleration: " + str(datarec["y_accel"])+ "m/s^2"
            self.ids.z_accel_label.text = "Z Acceleration: " + str(datarec["z_accel"])+ "m/s^2"
            """
            self.ids.x_vel_label.text = "X Velocity: " + str(datarec["x_vel"]) + "m/s"
            self.ids.y_vel_label.text = "Y Velocity: " + str(datarec["y_vel"]) + "m/s"
            self.ids.z_vel_label.text = "Z Velocity: " + str(datarec["z_vel"]) + "m/s"
            """
            self.ids.alt_label.text = "Altitude: \n " + str(datarec["altitude"]) + "m AGL"
            self.ids.vel_label.text = "Vertical Velocity " + str(datarec["velocity"]) + "m/s"

            self.ids.pitch_label.text = "Pitch: " + str(datarec["pitch"])
            self.ids.yaw_label.text = "Yaw: " + str(datarec["yaw"])
            self.ids.roll_label.text = "Roll: " + str(datarec["roll"])
            self.ids.pitchrate_label.text = "Pitch Rate: " + str(datarec["pitchrate"])
            self.ids.yawrate_label.text = "Yaw Rate: " + str(datarec["yawrate"])
            self.ids.rollrate_label.text = "Roll Rate: " + str(datarec["rollrate"])
            self.ids.flightcomp_battery_label.text = "FOX battery state: " + str(datarec["flightcompbattery"])
            if serialport.isOpen() == True:
                if nopacket >= 30:
                    self.ids.state_label.text = "Disconnected"
                    self.statecolor = [0.4, 0.2, 0.2, 1]
                    return
                match datarec["state"]:
                    case -1:
                        self.ids.state_label.text = "Lost Connection to FOX"
                        self.statecolor = [0.5, 0.2, 0.3, 1]
                    case 0:
                        self.ids.state_label.text = "Ground Idle"
                        self.statecolor = [0.4,0.4,1,1]
                    case 1:
                        self.ids.state_label.text = "Ready to Launch"
                        self.statecolor = [0.2, 1, 0.4, 1]
                    case 2:
                        self.ids.state_label.text = "Powered Ascent"
                        self.statecolor = [0.2, 0.8, 0.8, 1]
                    case 3:
                        self.ids.state_label.text = "Unpowered Ascent"
                        self.statecolor = [0.2, 0.8, 0.8, 1]
                    case 4:
                        self.ids.state_label.text = "Ballistic Dscent"
                        self.statecolor = [0.2, 0.8, 0.8, 1]
                    case 5:
                        self.ids.state_label.text = "Under Canopy"
                        self.statecolor = [0.2, 0.8, 0.8, 1]
                    case 6:
                        self.ids.state_label.text = "Landed"
                        self.statecolor = [0.2, 0.8, 0.8, 1]

            else:
                self.ids.state_label.text = "Disconnected"
                self.statecolor = [0.4,0.2,0.2,1]


            currenttime = time.time()
            global trylaunch
            if  currenttime-launchstarttime > 5 and trylaunch == True and self.launcharmed == True:
                serialport.write(b"l")
                print("launching")
                trylaunch = False
            if trylaunch == True and self.launcharmed == True:
                self.ids.launchbutton.text = "Luanch\n" + str(round(5-(currenttime-launchstarttime),2))





        def connecttoserial(self):
            try:
                serialport.port = self.port
                print(self.port)
                if (serialport.isOpen() != True):
                    serialport.open()
                    serialport.write(101)
            except:
                print("port not found")


    class MainApp(App):
        pass

    MainApp().run()

serialport.close()