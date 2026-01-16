import multiprocessing

from rocketpy.plots.compare import CompareFlights
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.patches import Ellipse
import multiprocessing
import pickle

from rocketpy import Environment, SolidMotor, Rocket, Flight

import csv

# Launch site
# FAR
# Latitude: 35.3466 N
# Longitude: -117.809 W
# Altitude: 2000 ft

site_lat = 35.3466
site_lon = -117.809 
site_alt = 2000/3.281 #ft

launch_date = [2026,1,18,7,0,0]

env = Environment(latitude=site_lat, longitude=site_lon, elevation=site_alt,date=(2026, 1, 18, 8), # year, month, day, hour
timezone="US/Pacific")


if __name__ == '__main__':
    numsims = 5
    numworkers = 2

    numsims = int(input("How many runs? "))
    numworkers = int(input("How many workers?"))

    if input("Use GFS?") == "yes":
        env.set_atmospheric_model(type="Forecast", file="GFS")


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
    thrust_source="./L1553chasebismuth.eng",
    dry_mass=0, #kg
    dry_inertia=(0.125, 0.125, 0.002), #moi ,kg/m^2
    nozzle_radius=25 / 1000, #m
    grain_number=5, #m
    grain_density=1815, #kg/m^3
    grain_outer_radius=23.81 / 1000, #m
    grain_initial_inner_radius= 10.16 / 1000, #m
    grain_initial_height=137 / 1000, #m
    grain_separation=1 / 1000, #m 
    grains_center_of_mass_position=0.397, #m
    center_of_dry_mass_position=0.317, #m
    nozzle_position=0, #m
    #burn_time=2.1, #s
    throat_radius=16.5 / 1000,#m
    coordinate_system_orientation="nozzle_to_combustion_chamber"
)
#BoosterMotor.draw()
#BoosterMotor.info()

SustainerMotor = SolidMotor(
    thrust_source="./L1081SomeKindaPropellent.eng",
    dry_mass=0, #kg
    dry_inertia=(0.125, 0.125, 0.002), #moi ,kg/m^2
    nozzle_radius=25 / 1000, #m
    grain_number=1, #m
    grain_density=1815, #kg/m^3
    grain_outer_radius=23.8 / 1000, #m
    grain_initial_inner_radius= 8 / 1000, #m
    grain_initial_height=558 / 1000, #m
    grain_separation=1 / 1000, #m 
    grains_center_of_mass_position=0.397, #m
    center_of_dry_mass_position=0.317, #m
    nozzle_position=0, #m
    #burn_time=3.5, #s
    throat_radius=5.77 / 1000,#m
    coordinate_system_orientation="nozzle_to_combustion_chamber",

)


#------------------------------------------------------------------------------------ROCKETSROCKETSROCKETS

RelapseSustainer = Rocket(
    radius=54 / 2000,
    mass=2.05,
    inertia=(6.321, 6.321, 0.034),
    power_off_drag="./CD Power OFF Sustainer Only.csv",
    power_on_drag="./CD Power ON Sustainer only.csv",
    center_of_mass_without_motor=-0.66675,
    coordinate_system_orientation="tail_to_nose"
    )
RelapseSustainer.add_motor(SustainerMotor, position=-1.1938)

sustainernose_cone = RelapseSustainer.add_nose(
    length=0.27432, kind="conical", position=0
)

sustainerfin_set = RelapseSustainer.add_trapezoidal_fins(
    n=4,
    root_chord=0.2032,
    tip_chord=0.0762,
    span=0.06223,
    position=-0.9906,
    cant_angle=0.05,
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)


cd = 1.8
diameter = 0.6096

area = (diameter/2)*(diameter/2)*3.14
#print(area)

main = RelapseSustainer.add_parachute(
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

drogue = RelapseSustainer.add_parachute(
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

#RelapseSustainer.draw()
#RelapseSustainer.plots.static_margin()
#RelapseSustainer.all_info()

#------------------------------------------------

RelapseSustainerNOMOTOR = Rocket(
    radius=54 / 2000,
    mass=3.6,
    inertia=(6.321, 6.321, 0.034),
    power_off_drag="./CD Power OFF Sustainer Only.csv",
    power_on_drag="./CD Power ON Sustainer only.csv",
    center_of_mass_without_motor=-0.66675,
    coordinate_system_orientation="tail_to_nose"
)

sustainernomotornose_cone = RelapseSustainerNOMOTOR.add_nose(
    length=0.27432, kind="conical", position=0
)

sustainernomotorfinset = RelapseSustainerNOMOTOR.add_trapezoidal_fins(
    n=4,
    root_chord=0.2032,
    tip_chord=0.0762,
    span=0.06223,
    position=-0.9906,
    cant_angle=0.05,
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)

#RelapseSustainerNOMOTOR.draw()
#RelapseSustainerNOMOTOR.plots.static_margin()
#RelapseSustainer.all_info()

#--------------------------------------

RelapseStack = Rocket(
    radius=54 / 2000,
    mass=5.162,
    inertia=(6.321, 6.321, 0.034),
    power_off_drag="./FULL STACK CD Power OFF.csv",
    power_on_drag="./FULL STACK CD Power ON.csv",
    center_of_mass_without_motor=-0.9906,
    coordinate_system_orientation="tail_to_nose"
    )
RelapseStack.add_motor(BoosterMotor, position=-2.032)

stacknose_cone = RelapseStack.add_nose(
    length=0.27432, kind="conical", position=0
)

sustainerstackfin_set = RelapseStack.add_trapezoidal_fins(
    n=4,
    root_chord=0.2032,
    tip_chord=0.0762,
    span=0.06223,
    position=-0.9906,
    cant_angle=0.05,
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)

boosterfin_set = RelapseStack.add_trapezoidal_fins(
    n=4,
    root_chord=0.127,
    tip_chord=0.0762,
    span=0.08636,
    position=-1.905,
    cant_angle=0.05,
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)

#RelapseStack.draw()
#RelapseStack.plots.static_margin()
#RelapseStack.all_info()

#monte carlo attempts
#just gonna vary launch angle for now

railinclination = [90,10]
railheading = [270,90]


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

def runsim(val):
    simid = np.random.randint(0,5000)
    print("running sim " + str(simid))
    StackFlight2 = Flight(
        rocket=RelapseStack, 
        environment=env, 
        rail_length=3.048, 
        inclination=np.random.normal(railinclination[0],railinclination[1],1)[0],#np.random.normal(railinclination,inclinationstd,1), 
        heading=np.random.normal(railheading[0],railheading[1],1)[0],
        max_time = BoosterMotor.burn_out_time
        )

    #StackFlight2.info()
    #StackFlight2.plots.trajectory_3d()
    flights.append(StackFlight2)

    boosterburnoutangle = quaternion_to_angle_from_vertical(StackFlight2.solution[-1][7],StackFlight2.solution[-1][8],StackFlight2.solution[-1][9],StackFlight2.solution[-1][10])

    #print("booster burnout at: " + str(boosterburnoutangle) + " on sim " + str(simid))

    if boosterburnoutangle > 19:
        return StackFlight2.solution

    maxstagingdelay = 18

    SustainerNOMOTORFlight2 = Flight(
        rocket=RelapseSustainerNOMOTOR, 
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

    stagingdelay = 12

    stagingindex = 0
    
    for point in SustainerNOMOTORFlight2.solution:
        #print("running staging checker at time " + str(point[0]) + " at velocity " + str(point[6]))
        currentangle = quaternion_to_angle_from_vertical(point[7],point[8],point[9],point[10])
        if point[6] < 700/3.281 or currentangle > 13:
            stagingdelay = point[0]
            
            #print("found staging point at " + str(stagingdelay) + "s, " + str(point[6]) + " m/s and " + str(currentangle) + " deg; on sim " + str(simid))
            break
        elif point[6] < 300/3.281 or currentangle > 20:
            return SustainerNOMOTORFlight2.solution
    
        stagingindex = stagingindex+1

    #print("sustainer ignition at: " + str(stagingdelay) + " seconds on sim " + str(simid))



    sustainerstartcondition = SustainerNOMOTORFlight2.solution[stagingindex][:]
    sustainerstartcondition[0] = 0

    SustainerFlight2 = Flight(
        rocket=RelapseSustainer, 
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
    

#runsim(2)


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
        
        results = (pool.map(runsim, numbers))

    print("done with sims")

    with open('flight_data.pkl', 'wb') as f:
        pickle.dump(results, f)

    print("results saved to pkl file")