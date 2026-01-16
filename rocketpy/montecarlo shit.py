import multiprocessing
# while thisflightnum < flightnum:
#     runsim()
#     thisflightnum = thisflightnum + 1 
#     print(thisflightnum)

from rocketpy.plots.compare import CompareFlights
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.patches import Ellipse
import multiprocessing

from rocketpy import Environment, SolidMotor, Rocket, Flight


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

#env.info()


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
print(area)

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
print(area)

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
landingcoordsx = []
landingcoordsy = []

# randomnum = np.random.normal(railinclination[0],railinclination[1],1)
# print(randomnum)

def runsim(val):
    print("running sim")
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

    stagingdelay = 8

    SustainerNOMOTORFlight2 = Flight(
        rocket=RelapseSustainerNOMOTOR, 
        environment=env, 
        initial_solution=StackFlight2,
        rail_length=0.01, 
        inclination=StackFlight2.attitude_angle(BoosterMotor.burn_out_time-0.01), 
        heading=StackFlight2.path_angle(BoosterMotor.burn_out_time-0.01),
        max_time = stagingdelay+BoosterMotor.burn_out_time
    )

    flights.append(SustainerNOMOTORFlight2)

    sustainerstartcondition = SustainerNOMOTORFlight2.solution[-2][:]
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
    print("ran sim")
    #print(landingpoint)
    return landingpoint
    

#runsim(2)


if __name__ == '__main__':

    numsims = 100
    numworkers = 20


    numbers = list(range(1,numsims))

    results = []

    x_data = []
    y_data = []
    
    # Create a pool of worker processes
    with multiprocessing.Pool(processes=numworkers) as pool:
        
        results = (pool.map(runsim, numbers))
    print(results)
    for x in results:
        x_data.append(x[0])
        y_data.append(x[1])

    print(landingcoordsx)
    print(landingcoordsy)
    #comparison = CompareFlights(results)
    # #SustainerFlight2.info()
    #comparison.trajectories_3d()
    # comparison.trajectories_2d()
    # Your GPS origin and data
    origin_lat = 35.3466  # Replace with your launch site
    origin_lon = -117.809

    # Convert meters to GPS coordinates
    meters_per_degree_lat = 111320
    meters_per_degree_lon = 111320 * np.cos(np.radians(origin_lat))

    lat_data = [origin_lat + (y / meters_per_degree_lat) for y in y_data]
    lon_data = [origin_lon + (x / meters_per_degree_lon) for x in x_data]

    # Calculate statistics
    mean_lat = np.mean(lat_data)
    mean_lon = np.mean(lon_data)
    mean_x = np.mean(x_data)
    mean_y = np.mean(y_data)
    std_x = np.std(x_data)
    std_y = np.std(y_data)

    # Function to create circle coordinates
    def create_circle(center_lon, center_lat, radius_x, radius_y, num_points=100):
        """Create circle coordinates in lon/lat"""
        angles = np.linspace(0, 2*np.pi, num_points)
        lons = center_lon + (radius_x / meters_per_degree_lon) * np.cos(angles)
        lats = center_lat + (radius_y / meters_per_degree_lat) * np.sin(angles)
        return lons, lats

    # Create KML file
    kml_content = f"""<?xml version="1.0" encoding="UTF-8"?>
    <kml xmlns="http://www.opengis.net/kml/2.2">
    <Document>
        <name>Rocket Landing Points</name>
        <description>Landing point distribution with standard deviation ellipses</description>
        
        <!-- Style for launch site -->
        <Style id="launchStyle">
        <IconStyle>
            <color>ff00ff00</color>
            <scale>1.5</scale>
            <Icon>
            <href>http://maps.google.com/mapfiles/kml/shapes/star.png</href>
            </Icon>
        </IconStyle>
        </Style>
        
        <!-- Style for landing points -->
        <Style id="landingStyle">
        <IconStyle>
            <color>ff0000ff</color>
            <scale>1.0</scale>
            <Icon>
            <href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>
            </Icon>
        </IconStyle>
        </Style>
        
        <!-- Style for mean point -->
        <Style id="meanStyle">
        <IconStyle>
            <color>ff00ffff</color>
            <scale>1.3</scale>
            <Icon>
            <href>http://maps.google.com/mapfiles/kml/shapes/target.png</href>
            </Icon>
        </IconStyle>
        </Style>
        
        <!-- Style for 1 std dev circle -->
        <Style id="sigma1Style">
        <LineStyle>
            <color>ff0000ff</color>
            <width>3</width>
        </LineStyle>
        <PolyStyle>
            <color>220000ff</color>
            <fill>1</fill>
            <outline>1</outline>
        </PolyStyle>
        </Style>
        
        <!-- Style for 2 std dev circle -->
        <Style id="sigma2Style">
        <LineStyle>
            <color>ff0099ff</color>
            <width>3</width>
        </LineStyle>
        <PolyStyle>
            <color>110099ff</color>
            <fill>1</fill>
            <outline>1</outline>
        </PolyStyle>
        </Style>
        
        <!-- Style for 3 std dev circle -->
        <Style id="sigma3Style">
        <LineStyle>
            <color>ff00ffff</color>
            <width>3</width>
        </LineStyle>
        <PolyStyle>
            <color>0800ffff</color>
            <fill>1</fill>
            <outline>1</outline>
        </PolyStyle>
        </Style>
        
        <!-- Launch site -->
        <Placemark>
        <name>Launch Site</name>
        <description>Origin point (0,0)</description>
        <styleUrl>#launchStyle</styleUrl>
        <Point>
            <coordinates>{origin_lon},{origin_lat},0</coordinates>
        </Point>
        </Placemark>
        
        <!-- Mean landing point -->
        <Placemark>
        <name>Mean Landing Point</name>
        <description>Average: {mean_x:.1f}m East, {mean_y:.1f}m North&lt;br/&gt;Std Dev: {std_x:.1f}m x {std_y:.1f}m</description>
        <styleUrl>#meanStyle</styleUrl>
        <Point>
            <coordinates>{mean_lon},{mean_lat},0</coordinates>
        </Point>
        </Placemark>
    """

    # Add standard deviation ellipses
    for sigma, style, name in [(1, 'sigma1Style', '1 Std Dev'), (2, 'sigma2Style', '2 Std Dev'), (3, 'sigma3Style', '3 Std Dev')]:
        lons, lats = create_circle(mean_lon, mean_lat, sigma * std_x, sigma * std_y)
        coords = ' '.join([f"{lon},{lat},0" for lon, lat in zip(lons, lats)])
        
        kml_content += f"""
        <!-- {name} Ellipse -->
        <Placemark>
        <name>{name} Ellipse</name>
        <description>{name}: {sigma * std_x:.1f}m x {sigma * std_y:.1f}m</description>
        <styleUrl>#{style}</styleUrl>
        <Polygon>
            <outerBoundaryIs>
            <LinearRing>
                <coordinates>
                {coords}
                </coordinates>
            </LinearRing>
            </outerBoundaryIs>
        </Polygon>
        </Placemark>
    """

    # Add all landing points
    for i, (lon, lat, x, y) in enumerate(zip(lon_data, lat_data, x_data, y_data), 1):
        kml_content += f"""
        <!-- Landing point {i} -->
        <Placemark>
        <name>Landing {i}</name>
        <description>Offset: {x:.1f}m East, {y:.1f}m North</description>
        <styleUrl>#landingStyle</styleUrl>
        <Point>
            <coordinates>{lon},{lat},0</coordinates>
        </Point>
        </Placemark>
    """

    kml_content += """
    </Document>
    </kml>
    """

    # Save to file with UTF-8 encoding
    filename = "rocket_landing_points.kml"
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(kml_content)

    print(f"KML file saved as '{filename}'")
    print(f"Launch site: {origin_lat:.6f}, {origin_lon:.6f}")
    print(f"Mean landing: {mean_lat:.6f}, {mean_lon:.6f} ({mean_x:.1f}m E, {mean_y:.1f}m N)")
    print(f"Standard deviation: {std_x:.1f}m (E-W) x {std_y:.1f}m (N-S)")
    print(f"Total points: {len(lat_data)}")