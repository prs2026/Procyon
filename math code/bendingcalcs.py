#You calculate the moment about the CP for each individual component on one side of the CP, and sum those up
#For example, Moment of NC = CNa * Alpha * dynamic pressure * dynamic pressure correction factor * cross section area
#Sorry, normal force of NC
#You then take the normal force of the NC and multiply it by the distance from rocket CP to NC CP

import numpy as np

rocket_centerofgravity = 0.4 #m



class Component():

    def __init__(self, diemsions, station, component_type):
        self.diemsions = diemsions #m 0 = length/root, 1 = diameter/semi span, 2 = second diameter/tip chord, 3 = sweep 
        self.station = station #m
        self.component_type = component_type #1 = body tube 2 = nose cone 3 = transition 4 = trapazoidal fin set
        if component_type == 1: #body tube
            length = self.diemsions[0]
            self.cp = self.station + 0.5*length
        elif component_type == 2: #nosecone
            length = self.diemsions[0]
            self.cp = (2/3)*length
            self.CNa = 2
        elif component_type == 3: #transition
            length = self.diemsions[0]
            diaemter1 = self.diemsions[1]
            diaemter2 = self.diemsions[2]
            self.cp = self.station + (length/3)*((diaemter1+(diaemter2*2))/diaemter1+diaemter2)
        elif component_type == 4: #trapazoidal fin set
            rootchord = self.diemsions[0]
            semispan = self.diemsions[1]
            tipchord = self.diemsions[2]
            sweep = self.diemsions[3]
            midchord = np.sqrt((rootchord^2)+semispan^2)
            self.cp = self.station + ((midchord*(rootchord + (2*tipchord)))/(3*(rootchord+tipchord)))+((1/6)*(rootchord+tipchord - ((rootchord*tipchord)/(rootchord-tipchord))))