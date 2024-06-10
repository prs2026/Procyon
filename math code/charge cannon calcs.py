#inputs

totallength = 4.3 #total length of charge cannon (in)
innerdia = 0.22  #inner diameter of cannon tube (in)

wantedcharge = 2 #wanted max charge with 10:1 L:D barrel(g)

#dont edit things 

LDratio = totallength/innerdia
radius = innerdia/2

barrellength = innerdia*10

chargelength = totallength-barrellength

maxcharge = round((3.14*chargelength*(radius**2))*16.387,3)*1.72

tubevolume = round((3.14*totallength*(radius**2))*16.387,3)

print("total L:D: "+ str(LDratio))
print("total length: "+ str(totallength) + "in")
print("inner diameter: "+ str(innerdia) + "in")
print("total volume: "+ str(tubevolume) + "cm^3"+"\n")

print("barrel length: "+ str(barrellength) + "in")
print("charge length: "+ str(chargelength) + "in""\n")

print("max charge size: "+ str(maxcharge) + "g")