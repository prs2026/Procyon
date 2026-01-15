from rocketpy import Environment, SolidMotor, Rocket, Flight
import datetime

# Launch site
# FAR
# Latitude: 35.3466 N
# Longitude: -117.809 W
# Altitude: 2000 ft

site_lat = 35.3466
site_lon = -117.809 
site_alt = 2000 #ft

launch_date = [2026,1,18,8,0,0]
timeadj = -4

env = Environment(latitude=site_lat, longitude=site_lon, elevation=site_alt,date=(2026, 1, 18, 8), # year, month, day, hour
    timezone="US/Pacific")

env.set_atmospheric_model(type="Forecast", file="GFS")

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
    burn_time=2.1, #s
    throat_radius=16.5 / 1000,#m
    coordinate_system_orientation="nozzle_to_combustion_chamber"
)

SustainerMotor = SolidMotor(
    thrust_source="./L1081SomeKindaPropellent.eng",
    dry_mass=0, #kg
    dry_inertia=(0.125, 0.125, 0.002), #moi ,kg/m^2
    nozzle_radius=25 / 1000, #m
    grain_number=1, #m
    grain_density=1815, #kg/m^3
    grain_outer_radius=23.81 / 1000, #m
    grain_initial_inner_radius= 8 / 1000, #m
    grain_initial_height=558 / 1000, #m
    grain_separation=1 / 1000, #m 
    grains_center_of_mass_position=0.397, #m
    center_of_dry_mass_position=0.317, #m
    nozzle_position=0, #m
    burn_time=3.5, #s
    throat_radius=5.77 / 1000,#m
    coordinate_system_orientation="nozzle_to_combustion_chamber"
)

Relapse = Rocket(
    radius=54 / 2000,
    mass=7.7,
    inertia=(6.321, 6.321, 0.034),
    power_off_drag="../data/rockets/calisto/powerOffDragCurve.csv",
    power_on_drag="../data/rockets/calisto/powerOnDragCurve.csv",
    center_of_mass_without_motor=0,
    coordinate_system_orientation="tail_to_nose"
    )

env.info()

BoosterMotor.all_info()
SustainerMotor.all_info()