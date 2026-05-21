from rocketpy import Environment, SolidMotor, Rocket, Flight, Function
import datetime
import csv
import pandas as pd
import numpy as np


BoosterMotor = SolidMotor(
thrust_source="./boostermotorchase.eng",
dry_mass=0, #kg
dry_inertia=(0.125, 0.125, 0.002), #moi ,kg/m^2
nozzle_radius=25 / 1000, #m
grain_number=1, #m
grain_density=1815, #kg/m^3
grain_outer_radius=27 / 1000, #m
grain_initial_inner_radius= 9.8 / 1000, #m
grain_initial_height=1193 / 1000, #m
grain_separation=1 / 1000, #m 
grains_center_of_mass_position=0.45, #m
center_of_dry_mass_position=0.317, #m
nozzle_position=-0.19, #m
#burn_time=2.1, #s
throat_radius=16.5 / 1000,#m
coordinate_system_orientation="nozzle_to_combustion_chamber"
)
#BoosterMotor.draw()
#BoosterMotor.info()

SustainerMotor = SolidMotor(
thrust_source="./L2523.eng",
dry_mass=0, #kg
dry_inertia=(0.125, 0.125, 0.002), #moi ,kg/m^2
nozzle_radius=25 / 1000, #m
grain_number=1, #m
grain_density=1815, #kg/m^3
grain_outer_radius=23.8 / 1000, #m
grain_initial_inner_radius= 10.5 / 1000, #m
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