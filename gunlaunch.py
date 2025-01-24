
#inputs
massflow_kgs = 0.02267 #kg
exhaustmolarmass_molg = 29.331000 #g/mol
thrust_N = 38 #N
rocketmass_kg = 0.25 #kg

tubediameter_m = 0.075 #m
tubelength_m = 2 #m
intialoffset_m = 0.03 #m

timestep_s = 0.01 #s

#calced inputs
platearea_m2 = 3.14*((tubediameter_m/2)*(tubediameter_m/2))
print(platearea_m2)

position_m = intialoffset_m

velocity_ms = 0

currenttime_s = 0

temp_k = 300

pressure_pa = 0

massintube_kg = 0

R = 8.31446261815324

#loop
position_m = intialoffset_m
while position_m < tubelength_m:
    currenttime_s = currenttime_s+timestep_s
    currentvolume_m3 = (3.14*((tubediameter_m/2)*(tubediameter_m/2))*position_m)

    massintube_kg = massintube_kg+massflow_kgs

    thrustaccel_mss = thrust_N/rocketmass_kg
    
    pressure_pa = (((1/exhaustmolarmass_molg)*massintube_kg*1000)*R*temp_k)/currentvolume_m3
    
    pressureaccel_mss = (platearea_m2*pressure_pa)/rocketmass_kg

    velocity_ms = velocity_ms + (thrustaccel_mss+pressureaccel_mss)*timestep_s
    position_m = position_m + (velocity_ms)*timestep_s

    print(str(round(currenttime_s,2)) + "s " + str(round(position_m,2))+"m " + str(round(massintube_kg,5))+"kg " + str(round(pressure_pa/6895,2)) + "psi " + str(round(currentvolume_m3,5)) + "m^3 " + str(round(velocity_ms*3.28084,2)) + "ft/s ")