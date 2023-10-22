from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color,Rectangle,RoundedRectangle
from kivy.properties import ColorProperty
from kivy.clock import Clock
from kivy.uix.textinput import TextInput
from kivy.core.window import Window

Window.size = (600, 800)


import threading,time,serial

quit = False

global prevupdatetime,port,cantopenserial
port = "COM15"
cantopenserial = False
databuf = {
    "startingchecksum" : 0xAB,
    "accelx" : 0,
    "accely" : 0,
    "accelz" : 0,

    "gyrox" : 0,
    "gyroy" : 0,
    "gyroz" : 0,

    "alt" : 0,
    "vvel" : 0,

    "orientationx" : 0,
    "orientationy" : 0,
    "orientationz" : 0,

    "uptime" : 0,
    "errorflagmp" : 0,
    "errorflagnav" : 0,
    "dataage" : 0,
    "selfuptime" : 0,

    "state" : 8,

    "endingchecksum" : 0xCD
}

met = 0

# "AB,%f,%f,%f"//accel
#   ",%f,%f,%f" // gyro
#   ",%f,%f" // alt, vvel
#   ",%f,%f,%f" // orientation
#   ",%d,%d,%d,%d,%d,CD \n", // uptime, errorflagmp, errorflagnav, dataage, selfuptime

statecolors = [
     [0.255, 0.376, 0.459,1],  # 0
     [0.118, 0.369, 0.157,1], #1
     [0.643, 0.671, 0.122,1], #2
     [0.129, 0.2, 0.549,1], #3
     [ 0.216, 0.235, 0.341,1], #4
     
     [0.157, 0.388, 0.18,1], #5
     [0.29, 0.224, 0.09,1], #6
     [0.722, 0.584, 0.518,1], #7
     [0.839, 0.839, 0.788,1] #8
]

statetext = [
     "Ground Idle",  # 0
     "Launch Detect", #1
     "Powered Ascent", #2
     "Unpowered Ascent", #3
     "Ballistic Descent", #4
     "Under Canopy", #5
     "Landed", #6
     "No Connection to LYRA", # 7
     "Disconnected" # 8
]
try:
    serialport = serial.Serial()
    serialport.baudrate = 115200
    serialport.bytesize = 8
    serialport.parity = "N"
    serialport.stopbits = 1
    serialport.timeout = 10
except:
     print("serial fail")


def getserialdata():
    while not quit:
        if serialport.isOpen():
             while serialport.inWaiting() > 0:
                reciveddata = str(serialport.readline())
                #print(reciveddata)
                reciveddata = reciveddata.replace("b'","")
                reciveddata = reciveddata.replace("'","")
                reciveddata = reciveddata.replace(" \\n","")
                #print(reciveddata)
                reciveddata = reciveddata.split(",")
                reciveddata[len(reciveddata)-1] = reciveddata[len(reciveddata)-1].replace("\\r\\n'","")
                print(reciveddata)
                if reciveddata[0] != "AB" or reciveddata[len(reciveddata)-1] != "CD":
                    print("badpacket")
                    break
                #print(reciveddata)
                for i in range(1,len(reciveddata)-1):
                    if databuf[list(databuf)[i]] != '':
                        try:
                            databuf[list(databuf)[i]] = round(float(reciveddata[i]),2)
                        except:
                            print("bad num at index: " + str(i))
                if databuf["dataage"] > 15000:
                    databuf["state"] = 7
                else:
                    databuf["state"] = int(databuf["state"])
        else:
            databuf["state"] = 8
        time.sleep(0.001)
    



try:
    serialthread = threading.Thread(target=getserialdata, daemon=True)
    serialthread.start()
except:
    print("thread fail")

class MainWidget(BoxLayout):
    def updatescreen(self,dt):
        try:
            self.ids.localtime_label.text = time.strftime("Local Time: %Y-%m-%d-%H:%M:%S %z UTC")
            self.ids.met_label.text = "Mission Elapsed Time: " + time.strftime("%H:%M:%S.",time.gmtime(met/1000)) + str(met%1000)
            self.ids.alt_label.text = "Altitude: " + str(databuf["alt"],) + "m AGL"
            self.ids.vvel_label.text = "Vertical Velocity: " + str(databuf["vvel"]) + "m/s"
            #self.ids.batt_label.text = "Battery State: " + str(databuf["flightcompbatt"]) + "V"
            #self.ids.pyro_label.text = "Pyro State: " + str(databuf["flightcompyrostate:"]) + "V"
            self.ids.uptime_label.text = "Uptimes: \n LERO: " + str(databuf["selfuptime"])+ "\n LYRA: " + str(databuf["uptime"])
            self.ids.dataage_label.text = "Data Age: " + str(databuf["dataage"])
            
            self.statecolor = statecolors[databuf["state"]]
            self.ids.state_label.text = statetext[databuf["state"]]
            
            if self.ids.setlaunchaware_input.text == "ARMED":
                
                if self.launcharmed == False:
                    self.ids.setlaunchaware_button.background_color = 0.376, 0.729, 0.212,0.25
                    self.ids.setlaunchaware_input.background_color = 0.376, 0.729, 0.212,0.25 
                self.launcharmed = True
            else:
                if self.launcharmed == True:
                    #print("setting the dang labels over and over again")
                    self.ids.setlaunchaware_button.background_color = 0.922, 0.243, 0.243,0.25
                    self.ids.setlaunchaware_input.background_color = 0.922, 0.243, 0.243,0.25 
                self.launcharmed = False
        finally:
            return
        

        
                  
             

    def startserial(self):
        port = self.ids.serialinput.text
        serialport.port = port
        print("\t\t\t trying to open serial")
        try:
            serialport.open()
            self.ids.serialbutton.text = "Connect to Serial \nOpened Port\n" + port
        except:
            self.ids.serialbutton.text = "Connect to Serial \nCant Open Port \n" + port

    def launch(self):
        if self.launcharmed:
            try:
                serialport.write(b'l')
                print("sent launch command")
            except:
                print("cant send launch command")
        else:
            print("not armed")
    
    def abort(self):
        try:
            serialport.write(b'a')
            print("sent abort command")
        except:
            print("cant send abort command")

    def movedata(self):
        try:
            serialport.write(b'm')
            print("sent movedata command")
        except:
            print("couldnt send movedata command")
        
    def getnewpadoffset(self):
        try:
            serialport.write(b'o')
            print("sent getpadoffset command")
        except:
            print("couldnt send getpadcommand")

    def __init__(self, **kwargs):
            super().__init__(**kwargs)
            Clock.schedule_interval(self.updatescreen, 0.1)
            self.launcharmed = False
    

    



          
          
class MainApp(App):
      pass

try:
     MainApp().run()
finally:
     pass
