
#motor inputs
massflow_kgs = 0.0235 #kg
exhaustmolarmass_molg = 29.331000 #g/mol
thrust_N = 38 #N
rocketmass_kg = 0.75 #kg

#tube inputs
tubediameter_m = 0.1143 #m
tubelength_m = 2 #m
intialoffset_m = 0.075 #m

#timestep
timestep_s = 0.005 #s

#fudgefactor
leakagefactor = 0.8


#calc pushing area from diameter
platearea_m2 = 3.14*((tubediameter_m/2)*(tubediameter_m/2))

#init variables
position_m = intialoffset_m
pressure_pa = 0
massintube_kg = 0

#givens
R = 8.31446261815324
velocity_ms = 0
currenttime_s = 0
temp_k = 300

#loop
position_m = intialoffset_m

print("PRESSTHRUST 10 10 P 0 0 Polaris")
while position_m < tubelength_m:
    #update time, mostly for file export reasons
    currenttime_s = currenttime_s+timestep_s
    #calc new volume now that it has moved up
    currentvolume_m3 = (3.14*((tubediameter_m/2)*(tubediameter_m/2))*position_m)

    #add mass from motor to the mass in the tube
    massintube_kg = massintube_kg+(massflow_kgs*timestep_s)

    #calculate the current pressure in the tube
    pressure_pa = ((((1/exhaustmolarmass_molg)*massintube_kg*1000)*R*temp_k)/currentvolume_m3)*leakagefactor
    
    #calc acceleration from thrust 
    thrustaccel_mss = thrust_N/rocketmass_kg
    
    #calc acceleration from the pressure
    pressureforce_N = platearea_m2*pressure_pa
    pressureaccel_mss = pressureforce_N/rocketmass_kg

    #update velocity and position with new acceleration and velocity
    velocity_ms = velocity_ms + (thrustaccel_mss+pressureaccel_mss)*timestep_s
    position_m = position_m + (velocity_ms)*timestep_s

    #uncomment to see rail exit
    print(str(round(currenttime_s,3)) + "s " + str(round(position_m,2))+"m " + str(round(massintube_kg,5))+"kg " + str(round(pressure_pa/6895,2)) + "psi " + str(round(currentvolume_m3,5)) + "m^3 " + str(round(velocity_ms*3.28084,2)) + "ft/s ")

    #uncomment to export for eng
    #print(" " + str(round(currenttime_s,3)) + " " + str(round(pressureforce_N,3)))
print(" " + str(round(currenttime_s+timestep_s,3)) + " 0")
print(";")