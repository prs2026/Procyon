from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color,Rectangle,RoundedRectangle
from kivy.properties import ColorProperty
from kivy.clock import Clock
from kivy.uix.textinput import TextInput
from kivy.core.window import Window

Window.size = (600, 700)


import threading,time,serial

quit = False
global prevupdatetime


def getserialdata():
    while not quit:
        print("getting data")
        time.sleep(0.1)



serialthread = threading.Thread(target=getserialdata, daemon=True)
serialthread.start()

class MainWidget(BoxLayout):
    def updatescreen(dt,self):
        print("screen updat")

    def __init__(self, **kwargs):
            super().__init__(**kwargs)
            Clock.schedule_interval(self.updatescreen, 0.1)

    



          
          
class MainApp(App):
      pass

try:
     MainApp().run()
finally:
     pass