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
            self.length = self.diemsions[0]
            self.cp = self.station + 0.5*self.length
        elif component_type == 2: #nosecone
            self.length = self.diemsions[0]
            self.cp = (2/3)*self.length
            self.CNa = 2
        elif component_type == 3: #transition
            self.length = self.diemsions[0]
            self.diameter1 = self.diemsions[1]
            self.diameter2 = self.diemsions[2]
            self.cp = self.station + (self.length/3)*((self.diameter1+(self.diameter2*2))/self.diameter1+self.diameter2)
        elif component_type == 4: #trapazoidal fin set
            self.rootchord = self.diemsions[0]
            self.semispan = self.diemsions[1]
            self.tipchord = self.diemsions[2]
            self.sweep = self.diemsions[3]
            self.midchord = np.sqrt((self.rootchord^2)+self.semispan^2)
            self.cp = self.station + ((self.midchord*(self.rootchord + (2*self.tipchord)))/(3*(self.rootchord+self.tipchord)))+((1/6)*(self.rootchord+self.tipchord - ((self.rootchord*self.tipchord)/(self.rootchord-self.tipchord))))



Nosecone = Component([12,2],0,2)
Bodytube = Component([30,2],12,1)
Fins = Component([6,2.5,4,2],34,4)




