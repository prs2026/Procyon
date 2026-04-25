import multiprocessing

from rocketpy.plots.compare import CompareFlights
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.patches import Ellipse
import multiprocessing
import pickle
import datetime

from rocketpy import Environment, SolidMotor, Rocket, Flight, Function

import csv
import pandas as pd

# Launch site
# FAR
# Latitude: 35.3466 N
# Longitude: -117.809 W
# Altitude: 2000 ft

site_lat = 35.3466
site_lon = -117.809 
site_alt = 2000/3.281 #ft

env = Environment()

# # Load the .csv file into the environment
df = pd.read_csv('vertical_profile_2026-01-15_14PST copy.csv')

#print(df)

# Create Function objects to represent the profiles
pressure_func = Function(np.column_stack([df['height'], df['pressure']]))
temperature_func = Function(np.column_stack([df['height'], df['temperature']]))
wind_u_func = Function(np.column_stack([df['height'], df['wind_u']]))
wind_v_func = Function(np.column_stack([df['height'], df['wind_v']]))

# Set up the environment

env.set_atmospheric_model(
    type="custom_atmosphere",
    pressure=pressure_func,
    temperature=temperature_func,
    wind_u=wind_u_func,
    wind_v=wind_v_func,
)

if __name__ == '__main__':
    numsims = 5
    numworkers = 2

    numsims = int(input("How many runs? "))
    numworkers = int(input("How many workers?"))

    weathermodel = input("What weather model?")

    #if  weathermodel == "GFS":
    # env = Environment(latitude=site_lat, longitude=site_lon, elevation=site_alt,date=(2026, 6, 7, 14), # year, month, day, hour
    # timezone="US/Pacific")

    # env.set_atmospheric_model(type="Forecast", file="HIRESW")

        

    # Plot the atmospheric model
    #env_csv.plots.atmospheric_model()

    #GFS - avalible a lot in advance, 18x18km grid
    #HIRESW - 48h prior, 3kmx3km grid


#env.info()

# solution[time_step] = [
#     t,           # 0: Time (s)
#     x,           # 1: X position (m) - East direction
#     y,           # 2: Y position (m) - North direction  
#     z,           # 3: Z position (m) - Altitude
#     vx,          # 4: X velocity (m/s)
#     vy,          # 5: Y velocity (m/s)
#     vz,          # 6: Z velocity (m/s)
#     e0,          # 7: Quaternion component 0
#     e1,          # 8: Quaternion component 1
#     e2,          # 9: Quaternion component 2
#     e3,          # 10: Quaternion component 3
#     omega_x,     # 11: Angular velocity X (rad/s)
#     omega_y,     # 12: Angular velocity Y (rad/s)
#     omega_z      # 13: Angular velocity Z (rad/s)
# ]


#------------------------------------------------------------------------------------MOTORSMOTORSMOTORSMOTORS

BoosterMotor = SolidMotor(
thrust_source="./boostermotor.eng",
dry_mass=0, #kg
dry_inertia=(0.125, 0.125, 0.002), #moi ,kg/m^2
nozzle_radius=25 / 1000, #m
grain_number=1, #m
grain_density=1815, #kg/m^3
grain_outer_radius=27 / 1000, #m
grain_initial_inner_radius= 10.31 / 1000, #m
grain_initial_height=990 / 1000, #m
grain_separation=1 / 1000, #m 
grains_center_of_mass_position=0.45, #m
center_of_dry_mass_position=0.317, #m
nozzle_position=-0.09, #m
burn_time=4, #s
throat_radius=16.5 / 1000,#m
coordinate_system_orientation="nozzle_to_combustion_chamber"
)
#BoosterMotor.draw()
#BoosterMotor.info()

SustainerMotor = SolidMotor(
thrust_source="./chuncmotor.eng",
dry_mass=0, #kg
dry_inertia=(0.125, 0.125, 0.002), #moi ,kg/m^2
nozzle_radius=25 / 1000, #m
grain_number=1, #m
grain_density=1700, #kg/m^3
grain_outer_radius=23.8 / 1000, #m
grain_initial_inner_radius= 9.5 / 1000, #m
grain_initial_height=787 / 1000, #m
grain_separation=1 / 1000, #m 
grains_center_of_mass_position=0.403, #m
center_of_dry_mass_position=0.317, #m
nozzle_position=-0.04, #m
#burn_time=3.5, #s
throat_radius=16.5 / 1000,#m
coordinate_system_orientation="nozzle_to_combustion_chamber",

)
#SustainerMotor.draw()
#SustainerMotor.info()
#SustainerMotor.info()


#------------------------------------------------------------------------------------ROCKETSROCKETSROCKETS
JUMPSustainer = Rocket(
    radius=54 / 2000,
    mass=2.4,
    inertia=(0.217, 0.217, 0.002),
    power_off_drag="./SustainerCDPowerOFF.csv",
    power_on_drag="./SustainerCDPowerON.csv",
    center_of_mass_without_motor=-0.66675,
    coordinate_system_orientation="tail_to_nose"
    )
JUMPSustainer.add_motor(SustainerMotor, position=(-1.5+0.04))

sustainernose_cone = JUMPSustainer.add_nose(
    length=0.27432, kind="conical", position=0
)

sustainerfin_set = JUMPSustainer.add_trapezoidal_fins(
    n=4,
    root_chord=0.2032,
    tip_chord=0.0762,
    span=0.0635,
    position=-1.2446,
    cant_angle=0.01,
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)


cd = 1.8
diameter = 0.6096

area = (diameter/2)*(diameter/2)*3.14
print(area)

main = JUMPSustainer.add_parachute(
    name="main",
    cd_s=area*cd,
    trigger=800,      # ejection altitude in meters
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
    radius=1.5,
    height=1.5,
    porosity=0.0432,
)

cd = 0.7
diameter = 0.1524

area = (diameter/2)*(diameter/2)*3.14
#print(area)

drogue = JUMPSustainer.add_parachute(
    name="drogue",
    cd_s=area,
    trigger="apogee",  # ejection at apogee
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
    radius=1.5,
    height=1.5,
    porosity=0.0432,
)

#JUMPSustainer.draw()
#JUMPSustainer.plots.static_margin()
#JUMPSustainer.all_info()

#------------------------------------------------

JUMPSustainerNOMOTOR = Rocket(
    radius=54 / 2000,
    mass=4.48,
    inertia=(0.217, 0.217, 0.002),
    power_off_drag="./SustainerCDPowerOFF.csv",
    power_on_drag="./SustainerCDPowerON.csv",
    center_of_mass_without_motor=-0.66675,
    coordinate_system_orientation="tail_to_nose"
)

sustainernomotornose_cone = JUMPSustainerNOMOTOR.add_nose(
    length=0.27432, kind="conical", position=0
)

sustainernomotorfinset = JUMPSustainerNOMOTOR.add_trapezoidal_fins(
    n=4,
    root_chord=0.2032,
    tip_chord=0.0762,
    span=0.0635,
    position=-1.2446,
    cant_angle=0.01,
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)




#JUMPSustainerNOMOTOR.draw()
#JUMPSustainerNOMOTOR.plots.static_margin()
#JUMPSustainer.all_info()

#--------------------------------------

JUMPStack = Rocket(
    radius=54 / 2000,
    mass=5.162,
    inertia=(4.04, 4.04, 0.006),
    power_off_drag="./SustainerAndBoosterCDPowerOFF.csv",
    power_on_drag="./SustainerAndBoosterCDPowerON.csv",
    center_of_mass_without_motor=-1.435,
    coordinate_system_orientation="tail_to_nose"
    )
JUMPStack.add_motor(BoosterMotor, position=(-2.732+0.09))

stacknose_cone = JUMPStack.add_nose(
    length=0.27432, kind="conical", position=0
)

sustainerstackfin_set = JUMPStack.add_trapezoidal_fins(
   n=4,
    root_chord=0.2032,
    tip_chord=0.0762,
    span=0.0635,
    position=-1.2446,
    cant_angle=0.01,
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)

isc_transition = JUMPStack.add_tail(
    top_radius=54/2000,   # ~3 inch radius (forward tube)
    bottom_radius=63.5/2000, # ~2.1 inch radius (aft tube)
    length=0.01778,         # 8 cm transition length
    position=-1.4986        # 0.5 m from origin in your coordinate system
)

boosterfin_set = JUMPStack.add_trapezoidal_fins(
    n=4,
    root_chord=0.127,
    tip_chord=0.0762,
    span=0.08255,
    position=-2.6162,
    cant_angle=0.001,
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)
#RelapseStack.draw()
#RelapseStack.plots.static_margin()
#RelapseStack.all_info()



JUMPBooster = Rocket(
    radius=54 / 2000,
    mass= 1.63,
    inertia=(6.321, 6.321, 0.034),
    power_off_drag="BoosterCd.csv",
    power_on_drag="./SustainerAndBoosterCDPowerON.csv",
    center_of_mass_without_motor=-0.7366,
    coordinate_system_orientation="tail_to_nose"
)

boosternose_cone = JUMPBooster.add_nose(
    length=0.1143, kind="conical", position=0
)

boosteronlyfin_set = JUMPBooster.add_trapezoidal_fins(
    n=4,
    root_chord=0.127,
    tip_chord=0.0762,
    span=0.08255,
    position=-2.6162,
    cant_angle=0.001,
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)

cd = 0.9
diameter = 0.508

area = (diameter/2)*(diameter/2)*3.14*cd
#print(area)

boostermain = JUMPBooster.add_parachute(
    name="boostermain",
    cd_s=area,
    trigger="apogee",  # ejection at apogee
    sampling_rate=105,
    lag=0,
    noise=(0, 8.3, 0.5),
    radius=0.5,
    height=0.15,
    porosity=0.0432,
)

#JUMPBooster.draw()

#monte carlo attempts
#just gonna vary launch angle for now

railinclination = [85,5]
railheading = [143,180]


flights = []



#------------------------------------------------------------------------------------SIMSIMSIMSIMSIMSIMSIMSIMS


def quaternion_to_angle_from_vertical(e0, e1, e2, e3):
    """
    Convert quaternion to angle from vertical (zenith angle)
    
    Parameters:
    e0, e1, e2, e3: Quaternion components
    
    Returns:
    angle_degrees: Angle from vertical in degrees
    """
    # Normalize quaternion (just in case)
    norm = np.sqrt(e0**2 + e1**2 + e2**2 + e3**2)
    e0, e1, e2, e3 = e0/norm, e1/norm, e2/norm, e3/norm
    
    # The vertical direction in body frame after rotation
    # Original vertical is [0, 0, 1] (pointing up)
    # Apply quaternion rotation to get current vertical direction
    
    # Quaternion rotation formula for vector [0, 0, 1]
    vx = 2 * (e0*e2 + e1*e3)
    vy = 2 * (e1*e2 - e0*e3)
    vz = e0**2 - e1**2 - e2**2 + e3**2
    
    # Angle from vertical is the angle between [0, 0, 1] and [vx, vy, vz]
    # cos(angle) = dot product with [0, 0, 1] = vz
    angle_rad = np.arccos(np.clip(vz, -1, 1))
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg

def runfullstacksim(val):
    simid = np.random.randint(0,5000)
    print("running sim " + str(simid))
    StackFlight2 = Flight(
        rocket=JUMPStack, 
        environment=env, 
        rail_length=3.048, 
        inclination=np.random.normal(railinclination[0],railinclination[1],1)[0],#np.random.normal(railinclination,inclinationstd,1), 
        heading=np.random.normal(railheading[0],railheading[1],1)[0],
        max_time = BoosterMotor.burn_out_time
        )
    #print(BoosterMotor.burn_out_time)

    #StackFlight2.info()
    #StackFlight2.plots.trajectory_3d()
    flights.append(StackFlight2)

    boosterburnoutangle = quaternion_to_angle_from_vertical(StackFlight2.solution[-1][7],StackFlight2.solution[-1][8],StackFlight2.solution[-1][9],StackFlight2.solution[-1][10])

    #print("booster burnout at: " + str(boosterburnoutangle) + " on sim " + str(simid))

    if boosterburnoutangle > 14:
        return StackFlight2.solution

    maxstagingdelay = 15

    SustainerNOMOTORFlight2 = Flight(
        rocket=JUMPSustainer, 
        environment=env, 
        initial_solution=StackFlight2,
        rail_length=0.01, 
        inclination=StackFlight2.attitude_angle(BoosterMotor.burn_out_time-0.01), 
        heading=StackFlight2.path_angle(BoosterMotor.burn_out_time-0.01),
        max_time = maxstagingdelay+BoosterMotor.burn_out_time
    )

    flights.append(SustainerNOMOTORFlight2)

    sustainerignitionangle = quaternion_to_angle_from_vertical(SustainerNOMOTORFlight2.solution[-1][7],SustainerNOMOTORFlight2.solution[-1][8],SustainerNOMOTORFlight2.solution[-1][9],SustainerNOMOTORFlight2.solution[-1][10])

    #print("sustainer ignition at: " + str(sustainerignitionangle) + " on sim " + str(simid))

    stagingtime = 0

    stagingdelay = 4.5

    stagingindex = 0
    
    for point in SustainerNOMOTORFlight2.solution:
        #print("running staging checker at time " + str(point[0]) + " at velocity " + str(point[6]))
        currentangle = quaternion_to_angle_from_vertical(point[7],point[8],point[9],point[10])
        if point[6] < 700/3.281 or currentangle > 9:
            stagingdelay = point[0]
            
            print("found staging point at " + str(round(stagingdelay,2)) + "s, " + str(round(point[6],3)) + " m/s and " + str(round(currentangle,3)) + " deg; on sim " + str(simid))
            break
        elif point[6] < 300/3.281 or currentangle > 15:
            return SustainerNOMOTORFlight2.solution
    
        stagingindex = stagingindex+1

    #print("sustainer ignition at: " + str(stagingdelay) + " seconds on sim " + str(simid))

    

    sustainerstartcondition = SustainerNOMOTORFlight2.solution[stagingindex][:]
    sustainerstartcondition[0] = 0

    SustainerFlight2 = Flight(
        rocket=JUMPSustainer, 
        environment=env, 
        initial_solution=sustainerstartcondition,
        rail_length=0.01, 
        inclination=SustainerNOMOTORFlight2.attitude_angle(stagingdelay+BoosterMotor.burn_out_time-0.01), 
        heading=SustainerNOMOTORFlight2.path_angle(stagingdelay+BoosterMotor.burn_out_time-0.01),
        #max_time = 15
    )
    for entry in SustainerFlight2.solution:
        entry[0] = entry[0]+stagingdelay+BoosterMotor.burn_out_time

    #flights.append(SustainerFlight2)
    landingpoint = [SustainerFlight2.x_impact,SustainerFlight2.y_impact]
    #landingcoords.append(landingpoint)
    print("ran sim " + str(simid))
    #print(landingpoi
    return SustainerFlight2.solution
    

#runfullstacksim(2)


if __name__ == '__main__':

    
    #------------------------------------------------------------------------------------
    #------------------------------------------------------------------------------------
    #------------------------------------------------------------------------------------
    


    numbers = list(range(1,numsims))

    results = []

    x_data = []
    y_data = []
    x_apogee = []
    y_apogee = []
    z_apogee = []

    ignitionangles = []
    ignitiontimings = []
    ignitionvelocites = []
    maxvelocity = []
    
    # Create a pool of worker processes
    with multiprocessing.Pool(processes=numworkers) as pool:
        
        results = (pool.map(runfullstacksim, numbers))

    print("done with sims")

    #filename = "simdata.pkl" + datetime.today().strftime('%Y-%m-%d %H:%M:%S')

    with open('flight_data.pkl', 'wb') as f:
        pickle.dump(results, f)

    print("results saved to pkl file")