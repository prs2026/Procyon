from rocketpy import Environment, SolidMotor, Rocket, Flight, Function
import datetime
import csv
import pandas as pd
import numpy as np
from motordefintions import SustainerMotor,BoosterMotor

# Load sustainer CD
df = pd.read_csv("sustainernewCD Test.CSV")

# RocketPy's Function expects (Mach, CD) pairs as a list of [x, y]
power_off_data = (
    df[["Mach", "CD Power-Off"]]
    .dropna()
    .drop_duplicates(subset="Mach", keep="first")
    .values.tolist()
)
power_on_data  = df[["Mach", "CD Power-On"]].dropna().drop_duplicates(subset="Mach", keep="first").values.tolist()

#print(power_off_data)

# Create RocketPy Functions
cd_power_off = Function(
    source=power_off_data,
    inputs="Mach Number",
    outputs="Drag Coefficient (Power Off)",
    interpolation="linear",
    extrapolation="constant",
)

#cd_power_off.plot()

cd_power_on = Function(
    source=power_on_data,
    inputs="Mach Number",
    outputs="Drag Coefficient (Power On)",
    interpolation="linear",
    extrapolation="constant",
)


# Load Stack CD
df = pd.read_csv("CD Testsustainerandbooster2.CSV")

# RocketPy's Function expects (Mach, CD) pairs as a list of [x, y]
stackpower_off_data = df[["Mach", "CD Power-Off"]].dropna().drop_duplicates(subset="Mach", keep="first").values.tolist()
stackpower_on_data  = df[["Mach", "CD Power-On"]].dropna().drop_duplicates(subset="Mach", keep="first").values.tolist()

# Create RocketPy Functions
cd_power_offstack = Function(
    source=stackpower_off_data,
    inputs="Mach Number",
    outputs="Drag Coefficient (Power Off)",
    interpolation="linear",
    extrapolation="constant",
)

cd_power_onstack = Function(
    source=stackpower_on_data,
    inputs="Mach Number",
    outputs="Drag Coefficient (Power On)",
    interpolation="linear",
    extrapolation="constant",
)



#rocket inputs
sustainerlength = 64.28/39.37 #m
sustainerfinspaceoffaft = 9.9/39.37 #m
sustainernoselength = 10.92/39.37 #m
sustainerdryCG = -0.66675 #m 
sustainerdrymass = 5-2.05 #kg

boosterlength = 60.7/39.37 #m
boosterdrymass = 4.017 #kg


JUMPSustainer = Rocket(
    radius=54 / 2000,
    mass=sustainerdrymass,
    inertia=(0.217, 0.217, 0.002),
    power_off_drag=cd_power_off,
    power_on_drag=cd_power_on,
    center_of_mass_without_motor=-0.66675,
    coordinate_system_orientation="tail_to_nose"
    )
JUMPSustainer.add_motor(SustainerMotor, position=(-1.651+0.04))

sustainernose_cone = JUMPSustainer.add_nose(
    length=10.8/39.37, kind="conical", position=0
)

sustainerfin_set = JUMPSustainer.add_trapezoidal_fins(
    n=4,
    root_chord=8/39.37,
    tip_chord=3/39.37,
    span=2.4/39.37,
    position=-1.38176,
    cant_angle=0.001,
    sweep_length=6/39.37
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)


cd = 1.8
diameter = 0.6096

area = (diameter/2)*(diameter/2)*3.14
#print(area)

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

cd = 0.6
diameter = 0.1016

area = (diameter/2)*(diameter/2)*3.14
#print(area)

drogue = JUMPSustainer.add_parachute(
    name="drogue",
    cd_s=area,
    trigger="apogee",  # ejection at apogee
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
    radius=0.1016,
    height=0.0762,
    porosity=0.0432,
)

#JUMPSustainer.draw()
#JUMPSustainer.plots.static_margin()
#JUMPSustainer.all_info()

#------------------------------------------------

JUMPSustainerNOMOTOR = Rocket(
    radius=54 / 2000,
    mass=sustainerdrymass+SustainerMotor.propellant_mass(0),
    inertia=(0.217, 0.217, 0.002),
    power_off_drag=cd_power_off,
    power_on_drag=cd_power_on,
    center_of_mass_without_motor=-0.66675,
    coordinate_system_orientation="tail_to_nose"
)

sustainernomotornose_cone = JUMPSustainerNOMOTOR.add_nose(
    length=0.27432, kind="conical", position=0
)

sustainernomotorfinset = JUMPSustainerNOMOTOR.add_trapezoidal_fins(
    n=4,
    root_chord=8/39.37,
    tip_chord=3/39.37,
    span=2.4/39.37,
    position=-1.38176,
    cant_angle=0.001,
    sweep_length=6/39.37
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)




#JUMPSustainerNOMOTOR.draw()
#JUMPSustainerNOMOTOR.plots.static_margin()
#JUMPSustainer.all_info()

#--------------------------------------

JUMPStack = Rocket(
    radius=54 / 2000,
    mass=boosterdrymass+sustainerdrymass+SustainerMotor.propellant_mass(0),
    inertia=(4.04, 4.04, 0.006),
    power_off_drag=cd_power_offstack,
    power_on_drag=cd_power_onstack,
    center_of_mass_without_motor=-1.5635224,
    coordinate_system_orientation="tail_to_nose"
    )
JUMPStack.add_motor(BoosterMotor, position=(-3.048+0.19))

stacknose_cone = JUMPStack.add_nose(
    length=0.27432, kind="conical", position=0
)

sustainerstackfin_set = JUMPStack.add_trapezoidal_fins(
    n=4,
    root_chord=8/39.37,
    tip_chord=3/39.37,
    span=2.4/39.37,
    position=-1.38176,
    cant_angle=0.001,
    sweep_length=6/39.37
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)

isc_transition = JUMPStack.add_tail(
    top_radius=54/2000,  
    bottom_radius=63.5/2000, 
    length=0.01778,         # 8 cm transition length
    position=-1.4986        # 0.5 m from origin in your coordinate system
)

boosterfin_set = JUMPStack.add_trapezoidal_fins(
    n=4,
    root_chord=6/39.37,
    tip_chord=3/39.37,
    span=4/39.37,
    position=-2.9192,
    cant_angle=0.001,
    #airfoil=("../data/airfoils/NACA0012-radians.txt","radians"),
)

#JUMPStack.draw()
#JUMPStack.plots.static_margin()
#JUMPStack.all_info()